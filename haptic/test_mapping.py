"""
Self-test for the mapping maths. Needs no hardware.

    python test_mapping.py

Checks the convention against a hand-built Rx.Ry.Rz, then the two clutches,
the signed projection, the fences, and that each cycle's pen turn splits
into tip up (psi), tip right (phi) and twist (theta_n), added up per cycle.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from mapping import (MapConfig, PoseMap, drag_band, euler_xyz_deg, joint_turn,
                     matrix_from_euler_xyz_deg, split_turn)


class FakeSample:
    """Just enough of touch.Sample for the mapper."""
    def __init__(self, rotation=None, position=None, buttons=0,
                 joints=None, gimbal=None):
        self.rotation = np.eye(3) if rotation is None else rotation
        self.position = np.zeros(3) if position is None else np.asarray(position, float)
        self.buttons  = buttons
        self.joints   = joints        # arm, radians - broken-hinge mode only
        self.gimbal   = gimbal        # wrist, radians


def _rx(a): c, s = np.cos(a), np.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def _ry(a): c, s = np.cos(a), np.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def _rz(a): c, s = np.cos(a), np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])


def _turn(deg, axis):
    return Rotation.from_rotvec(np.radians(deg) * np.asarray(axis, float)).as_matrix()


def check(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")
    if not cond:
        raise AssertionError(name)


def test_convention():
    """matrix_from_euler_xyz_deg must be exactly Rx(psi).Ry(phi).Rz(theta_n)."""
    for psi, phi, thn in [(10, 20, 30), (-25, 5, -40), (30, -30, 45), (0, 0, 0)]:
        want = _rx(np.radians(psi)) @ _ry(np.radians(phi)) @ _rz(np.radians(thn))
        got  = matrix_from_euler_xyz_deg(psi, phi, thn)
        check(f"Rx.Ry.Rz matches for ({psi},{phi},{thn})",
              np.allclose(want, got, atol=1e-12))


def test_roundtrip():
    for psi, phi, thn in [(10, 20, 30), (-25, 5, -40), (29, -29, 44)]:
        R = matrix_from_euler_xyz_deg(psi, phi, thn)
        (a, b, c), locked = euler_xyz_deg(R)
        check(f"roundtrip ({psi},{phi},{thn})",
              np.allclose([a, b, c], [psi, phi, thn], atol=1e-9) and not locked)


def test_gimbal_lock_flagged():
    _, locked = euler_xyz_deg(matrix_from_euler_xyz_deg(0, 89.99, 0))
    check("phi near 90 deg raises the lock flag", locked)
    _, locked = euler_xyz_deg(matrix_from_euler_xyz_deg(0, 30, 0))
    check("phi at 30 deg does not", not locked)


# Pens held in different ways. The Touch's frame: +X right, +Y up, +Z
# towards the user; at identity the tip points away from the user (-Z).
AWAY   = np.eye(3)                                   # tip away, level
DOWN45 = _turn(-45, [1, 0, 0])                       # tip away and down 45
DOWN   = _turn(-90, [1, 0, 0])                       # tip straight down


def _tip(R):
    return R @ np.array([0.0, 0.0, -1.0])


def test_split_turn_directions():
    """Up/right are the ROOM's, read against where the tip actually goes."""
    for name, P in (("level", AWAY), ("45 down", DOWN45), ("straight down", DOWN)):
        x = np.array([1.0, 0, 0]) - (_tip(P) @ [1.0, 0, 0]) * _tip(P)
        x /= np.linalg.norm(x)
        right_axis = np.cross(_tip(P), x)
        # Swing the tip 10 deg towards +X (right).
        u, r, t, _ = split_turn(P, _turn(10, right_axis) @ P)
        check(f"{name}: tip right reads as right only ({u:+.2f} {r:+.2f} {t:+.2f})",
              abs(r - 10) < 1e-9 and abs(u) < 1e-9 and abs(t) < 1e-9)
        check(f"{name}: and the tip really did go right",
              (_tip(_turn(10, right_axis) @ P) - _tip(P))[0] > 0)
        # Tip up: about the room's right axis.
        u, r, t, _ = split_turn(P, _turn(10, x) @ P)
        check(f"{name}: tip up reads as up only ({u:+.2f} {r:+.2f} {t:+.2f})",
              abs(u - 10) < 1e-9 and abs(r) < 1e-9 and abs(t) < 1e-9)
        if name != "straight down":     # straight down, "up" is "away"
            check(f"{name}: and the tip really did go up",
                  _tip(_turn(10, x) @ P)[1] > _tip(P)[1])


def test_split_turn_level_pen_in_room_terms():
    """Plain words, pen level and pointing away: tip up is +rotation about
    the room's X, tip right is -rotation about the room's Y (up)."""
    u, r, t, _ = split_turn(AWAY, _turn(8, [1, 0, 0]))
    check("tip up -> up", abs(u - 8) < 1e-9 and abs(r) < 1e-9)
    check("and it lifted the tip", _tip(_turn(8, [1, 0, 0]))[1] > 0)
    u, r, t, _ = split_turn(AWAY, _turn(-8, [0, 1, 0]))
    check("tip right -> right", abs(r - 8) < 1e-9 and abs(u) < 1e-9)
    check("and it moved the tip right", _tip(_turn(-8, [0, 1, 0]))[0] > 0)


# --------------------------------------------------------- rotation clutch
def test_rotation_clutch_scales():
    # tremor off: this is about the ratio, and the band is tested below.
    m = PoseMap(MapConfig(angle_ratio=2.0, tremor_deg=0.0))
    m.engage_rotation(FakeSample())
    cmd = m.update(FakeSample(rotation=_turn(20, [1, 0, 0])))
    check("20 deg of wrist roll at 2:1 gives 10 deg of psi",
          abs(cmd.psi_deg - 10.0) < 1e-6)
    check("and nothing on the other two axes",
          abs(cmd.phi_deg) < 1e-6 and abs(cmd.theta_n_deg) < 1e-6)


def test_rotation_engage_never_steps():
    """Engaging must command exactly the pose already held."""
    m = PoseMap()
    m.pose_deg[:] = (12.0, -7.0, 20.0)
    tilted = matrix_from_euler_xyz_deg(33, -18, 70)         # stylus somewhere random
    m.engage_rotation(FakeSample(rotation=tilted))
    cmd = m.update(FakeSample(rotation=tilted))
    check("no jump at engage",
          np.allclose([cmd.psi_deg, cmd.phi_deg, cmd.theta_n_deg], [12, -7, 20], atol=1e-9))


def test_rotation_released_holds():
    m = PoseMap(MapConfig(tremor_deg=0.0))
    m.engage_rotation(FakeSample())
    m.update(FakeSample(rotation=_turn(20, [1, 0, 0])))
    m.release_rotation()
    cmd = m.update(FakeSample(rotation=np.eye(3)))
    check("releasing freezes the pose regardless of stylus motion",
          abs(cmd.psi_deg - 10.0) < 1e-6 and not cmd.rot_engaged)


# ------------------------------------------------------ translation clutch
def test_translation_is_signed():
    m = PoseMap(MapConfig(tool_ratio=3.0))
    # Stylus at identity, so its -Z body axis points along world -Z.
    m.engage_translation(FakeSample(position=[0, 0, 0]))
    fwd = m.update(FakeSample(position=[0, 0, -30]))        # push along the tip
    check("pushing along the stylus advances the tool",
          abs(fwd.s_mm - 30.0) < 1e-9 and abs(fwd.tool_mm - 10.0) < 1e-9)
    back = m.update(FakeSample(position=[0, 0, +30]))       # pull the other way
    check("pulling back retracts it (this is why it is not Euclidean)",
          abs(back.s_mm + 30.0) < 1e-9 and abs(back.tool_mm + 10.0) < 1e-9)


def test_sideways_motion_is_ignored():
    m = PoseMap()
    m.engage_translation(FakeSample(position=[0, 0, 0]))
    cmd = m.update(FakeSample(position=[40, 25, 0]))        # across the pointing axis
    check("motion across the latched axis does not move the tool",
          abs(cmd.s_mm) < 1e-9 and abs(cmd.tool_mm) < 1e-9)


def test_pointing_axis_stays_latched():
    """Rotating while translating must not make the projection drift."""
    m = PoseMap(MapConfig(tool_ratio=1.0))
    m.engage_translation(FakeSample(position=[0, 0, 0]))
    straight = m.update(FakeSample(position=[0, 0, -20]))
    turned   = m.update(FakeSample(rotation=_turn(90, [1, 0, 0]), position=[0, 0, -20]))
    check("same travel gives the same tool however the stylus is turned",
          abs(straight.tool_mm - turned.tool_mm) < 1e-12)


def test_wall_position_tracks_tool():
    cfg = MapConfig(tool_ratio=3.0, tool_limit_mm=20.0)
    m = PoseMap(cfg)
    m.engage_translation(FakeSample())
    lo, hi = m.travel_limits_mm()
    check("wall at +/-60 mm of stylus when the tool is centred",
          abs(lo + 60.0) < 1e-9 and abs(hi - 60.0) < 1e-9)
    m.tool_mm = 10.0
    m.engage_translation(FakeSample())
    lo, hi = m.travel_limits_mm()
    check("wall moves in once the tool is already advanced",
          abs(lo + 90.0) < 1e-9 and abs(hi - 30.0) < 1e-9)


# ------------------------------------------------------------ independence
def test_clutches_are_independent():
    """The point of two buttons: aim without inserting, insert without re-aiming."""
    m = PoseMap(MapConfig(angle_ratio=2.0, tool_ratio=3.0, tremor_deg=0.0))

    m.engage_rotation(FakeSample())
    cmd = m.update(FakeSample(rotation=_turn(20, [1, 0, 0]), position=[0, 0, -60]))
    check("rotation clutch alone moves the pose", abs(cmd.psi_deg - 10.0) < 1e-6)
    check("rotation clutch alone leaves the tool untouched", abs(cmd.tool_mm) < 1e-12)
    m.release_rotation()

    m.engage_translation(FakeSample(position=[0, 0, 0]))
    cmd = m.update(FakeSample(rotation=_turn(80, [0, 1, 0]), position=[0, 0, -30]))
    check("translation clutch alone moves the tool", abs(cmd.tool_mm - 10.0) < 1e-9)
    check("translation clutch alone leaves the pose untouched",
          abs(cmd.psi_deg - 10.0) < 1e-6 and abs(cmd.phi_deg) < 1e-12)


def test_neither_clutch_holds_everything():
    m = PoseMap()
    m.pose_deg[:] = (5.0, -3.0, 11.0)
    m.tool_mm = 7.0
    cmd = m.update(FakeSample(rotation=_turn(45, [1, 1, 0]), position=[70, 50, -30]))
    check("with both released nothing moves",
          np.allclose([cmd.psi_deg, cmd.phi_deg, cmd.theta_n_deg], [5, -3, 11])
          and abs(cmd.tool_mm - 7.0) < 1e-12
          and not cmd.rot_engaged and not cmd.trans_engaged)


# ------------------------------------------------------------ tremor band
def test_drag_band_ignores_small_movement():
    for demand in (0.0, 0.4, -0.9, 1.0):
        check(f"held still at demand {demand:+.1f} inside a 1 deg band",
              drag_band(demand, 0.0, 1.0) == 0.0)


def test_drag_band_drags_rather_than_steps():
    """The reason this is not a plain deadband."""
    held = drag_band(5.0, 0.0, 1.0)
    check("escaping the band leaves the held value trailing by the band",
          abs(held - 4.0) < 1e-12)
    check("a plain deadband would have jumped the whole 5 deg", held != 5.0)
    check("and it keeps trailing as the demand grows",
          abs(drag_band(6.0, held, 1.0) - 5.0) < 1e-12)


def test_drag_band_reversal_costs_two_bands():
    held = drag_band(5.0, 0.0, 1.0)          # 4.0, trailing below
    check("reversing within two bands does nothing",
          drag_band(3.5, held, 1.0) == held)
    check("past two bands it picks up on the other side",
          abs(drag_band(2.5, held, 1.0) - 3.5) < 1e-12)


def test_drag_band_zero_is_off():
    check("band 0 passes the demand straight through",
          drag_band(0.123, 99.0, 0.0) == 0.123)


def test_tremor_suppresses_jitter_through_the_mapper():
    cfg = MapConfig(angle_ratio=1.0, tremor_deg=1.0)
    m = PoseMap(cfg)
    m.engage_rotation(FakeSample())
    # Half a degree of wobble, back and forth, is what a held hand does.
    seen = set()
    for deg in (0.4, -0.4, 0.3, -0.5, 0.45, -0.2):
        seen.add(round(m.update(FakeSample(rotation=_turn(deg, [1, 0, 0]))).psi_deg, 9))
    check(f"0.5 deg of wobble moves the command not at all ({seen})", seen == {0.0})
    cmd = m.update(FakeSample(rotation=_turn(6.0, [1, 0, 0])))
    check("a real 6 deg movement gets through, trailing by the band",
          abs(cmd.psi_deg - 5.0) < 1e-6)


def test_tremor_applied_before_the_fence():
    """A hand resting on the fence must not have its jitter let through."""
    cfg = MapConfig(angle_ratio=1.0, tremor_deg=1.0, psi_limit_deg=30.0)
    m = PoseMap(cfg)
    m.engage_rotation(FakeSample())
    m.update(FakeSample(rotation=_turn(60.0, [1, 0, 0])))      # hard on the fence
    at_fence = m.pose_deg[0]
    check("sitting on the fence", abs(at_fence - 30.0) < 1e-9)
    still = m.update(FakeSample(rotation=_turn(59.6, [1, 0, 0]))).psi_deg
    check("jitter while against the fence does not move the command",
          abs(still - at_fence) < 1e-9)


def test_clamping():
    cfg = MapConfig(angle_ratio=1.0, tool_ratio=1.0, tool_limit_mm=20.0)
    m = PoseMap(cfg)
    m.engage_rotation(FakeSample())
    m.engage_translation(FakeSample())
    cmd = m.update(FakeSample(rotation=_turn(80, [1, 0, 0]), position=[0, 0, -100]))
    check("tool clamps at its limit", abs(cmd.tool_mm - 20.0) < 1e-9)
    check("and says so", "tool" in cmd.clamped)
    check("psi clamps at 30 deg", abs(cmd.psi_deg - 30.0) < 1e-9)
    check("and says so", "psi" in cmd.clamped)


# ------------------------------------------------------------ rig resync
def test_sync_moves_the_wall_to_the_real_tool():
    """The bug this exists for: commanded +15, rig stopped at +3."""
    m = PoseMap(MapConfig(tool_ratio=3.0, tool_limit_mm=15.0))
    m.engage_translation(FakeSample())
    m.update(FakeSample(position=[0, 0, -45]))               # PC now says +15
    m.release_translation()
    check("accepted with no clutch held", m.sync_from_rig((0, 0, 0), 3.0))
    m.engage_translation(FakeSample())
    lo, hi = m.travel_limits_mm()
    check(f"36 mm of forward stylus travel left, not 0 ({hi:.1f})",
          abs(hi - 36.0) < 1e-9 and abs(lo + 54.0) < 1e-9)


def test_sync_refused_while_engaged():
    m = PoseMap()
    m.engage_rotation(FakeSample())
    check("sync refused with a clutch held",
          not m.sync_from_rig((10, 0, 0), 5.0))
    check("and nothing changed",
          abs(m.pose_deg[0]) < 1e-12 and abs(m.tool_mm) < 1e-12)


def test_synced_outside_the_fence_does_not_snap_back():
    m = PoseMap(MapConfig(angle_ratio=1.0, tremor_deg=0.0, theta_n_limit_deg=45.0))
    m.sync_from_rig((0, 0, 60.0), 0.0)                      # a MOVE left it at 60
    m.engage_rotation(FakeSample())
    held = m.update(FakeSample()).theta_n_deg
    check(f"engage holds 60, no step to the 45 fence ({held:.3f})",
          abs(held - 60.0) < 1e-9)
    out = m.update(FakeSample(rotation=_turn(5, [0, 0, 1]))).theta_n_deg
    check("cannot go further out", abs(out - 60.0) < 1e-9)
    # The 5 deg pushed outward was dropped, so the full 15 deg back counts.
    back = m.update(FakeSample(rotation=_turn(-10, [0, 0, 1]))).theta_n_deg
    check(f"can come back in ({back:.3f})", abs(back - 45.0) < 1e-6)


def test_synced_past_180_does_not_wrap():
    m = PoseMap(MapConfig(angle_ratio=1.0, tremor_deg=0.0, theta_n_limit_deg=720.0))
    m.sync_from_rig((0, 0, 200.0), 0.0)
    m.engage_rotation(FakeSample())
    cmd = m.update(FakeSample(rotation=_turn(3, [0, 0, 1])))
    check(f"theta_n 200 + 3 reads 203, not -157 ({cmd.theta_n_deg:.3f})",
          abs(cmd.theta_n_deg - 203.0) < 1e-6)


# ------------------------------- tip up / tip right / twist, turns added up
def test_twist_is_theta_n_however_the_pen_is_held():
    """A twist about the pen's own long axis is theta_n, even with the pen
    tilted well away from the room's axes."""
    m = PoseMap(MapConfig(angle_ratio=2.0, tremor_deg=0.0))
    held = _turn(40, [1, 0, 0]) @ _turn(-25, [0, 1, 0])     # pen tipped two ways
    m.engage_rotation(FakeSample(rotation=held))
    cmd = m.update(FakeSample(rotation=held @ _turn(20, [0, 0, 1])))
    check(f"20 deg twist at 2:1 gives theta_n 10, nothing else "
          f"({cmd.psi_deg:+.3f} {cmd.phi_deg:+.3f} {cmd.theta_n_deg:+.3f})",
          np.allclose([cmd.psi_deg, cmd.phi_deg, cmd.theta_n_deg], [0, 0, 10], atol=1e-9))


def _pose(cmd):
    return np.array([cmd.psi_deg, cmd.phi_deg, cmd.theta_n_deg])


def test_tip_up_is_psi_tip_right_is_phi():
    """As measured on the rig: +psi tilts the needle up, +phi tilts it right.
    Small steps, as the 10 ms loop would see them."""
    for name, P in (("level", AWAY), ("45 down", DOWN45)):
        m = PoseMap(MapConfig(angle_ratio=2.0, tremor_deg=0.0))
        m.engage_rotation(FakeSample(rotation=P))
        for i in range(1, 21):                       # tip up 10 deg in 0.5s
            cmd = m.update(FakeSample(rotation=_turn(0.5 * i, [1, 0, 0]) @ P))
        check(f"{name}: tip up 10 deg -> psi +5 only {np.round(_pose(cmd), 3)}",
              np.allclose(_pose(cmd), [5, 0, 0], atol=1e-6))

        m = PoseMap(MapConfig(angle_ratio=2.0, tremor_deg=0.0))
        m.engage_rotation(FakeSample(rotation=P))
        R = P
        for _ in range(20):                          # tip right 10 deg
            d = _tip(R)
            x = np.array([1.0, 0, 0]) - d[0] * d
            R = _turn(0.5, np.cross(d, x / np.linalg.norm(x))) @ R
            cmd = m.update(FakeSample(rotation=R))
        check(f"{name}: tip right 10 deg -> phi +5 only {np.round(_pose(cmd), 3)}",
              np.allclose(_pose(cmd), [0, 5, 0], atol=1e-6))
        check(f"{name}: and the pen tip did end up to the right", _tip(R)[0] > 0.15)


def test_tip_directions_ignore_the_pens_roll():
    """The flaw in the first version: rolling the pen in the fingers turned
    left/right into up/down. Tip up must be psi however the pen is rolled."""
    for roll in (0, 45, 90, 180):
        P = _turn(roll, [0, 0, 1])                   # level, rolled
        m = PoseMap(MapConfig(angle_ratio=1.0, tremor_deg=0.0))
        m.engage_rotation(FakeSample(rotation=P))
        cmd = m.update(FakeSample(rotation=_turn(6, [1, 0, 0]) @ P))
        check(f"rolled {roll:3d}: tip up -> psi only {np.round(_pose(cmd), 6)}",
              np.allclose(_pose(cmd), [6, 0, 0], atol=1e-9))


def test_tip_directions_ignore_theta_n():
    """The other half of that flaw: with the platform spun round, tip right
    must still be phi, not some mix of psi and phi."""
    for thn in (0.0, 90.0, 200.0):
        m = PoseMap(MapConfig(angle_ratio=1.0, tremor_deg=0.0,
                              theta_n_limit_deg=720.0))
        m.sync_from_rig((0.0, 0.0, thn), 0.0)
        m.engage_rotation(FakeSample())
        cmd = m.update(FakeSample(rotation=_turn(-6, [0, 1, 0])))    # tip right
        check(f"theta_n {thn:5.0f}: tip right -> phi only {np.round(_pose(cmd), 6)}",
              np.allclose(_pose(cmd), [0, 6, thn], atol=1e-9))


def test_no_dead_zone_off_a_fence():
    """The reason for adding up turns: overshoot past a fence is dropped."""
    m = PoseMap(MapConfig(angle_ratio=1.0, tremor_deg=0.0, psi_limit_deg=30.0))
    m.engage_rotation(FakeSample())
    m.update(FakeSample(rotation=_turn(50, [1, 0, 0])))      # 20 deg past the fence
    check("on the fence", abs(m.pose_deg[0] - 30.0) < 1e-9)
    back = m.update(FakeSample(rotation=_turn(45, [1, 0, 0]))).psi_deg
    check(f"5 deg back moves it 5 deg straight away ({back:.3f})",
          abs(back - 25.0) < 1e-6)


def test_slow_movement_survives_the_tremor_band():
    """100 turns of 0.06 deg - each far inside a 1 deg band - must still
    add up to a movement. A band applied per turn would lose all of it."""
    m = PoseMap(MapConfig(angle_ratio=1.0, tremor_deg=1.0))
    m.engage_rotation(FakeSample())
    for i in range(1, 101):
        cmd = m.update(FakeSample(rotation=_turn(0.06 * i, [1, 0, 0])))
    check(f"6 deg done slowly gets through, trailing by the band ({cmd.psi_deg:.3f})",
          abs(cmd.psi_deg - 5.0) < 1e-6)


def test_tremor_band_still_reaches_the_fence():
    m = PoseMap(MapConfig(angle_ratio=1.0, tremor_deg=1.0, psi_limit_deg=30.0))
    m.engage_rotation(FakeSample())
    for d in range(1, 41):
        cmd = m.update(FakeSample(rotation=_turn(float(d), [1, 0, 0])))
    check(f"the output gets all the way to 30, not 29 ({cmd.psi_deg:.3f})",
          abs(cmd.psi_deg - 30.0) < 1e-9)


# ------------------------------------------------------ broken-hinge mode
def _js(elbow=0.0, w1=0.0, w3=0.0, rotation=None, position=None):
    """A sample with joints in DEGREES for readability (stored as radians)."""
    return FakeSample(rotation=rotation, position=position,
                      joints=np.radians([0.0, 0.0, elbow]),
                      gimbal=np.radians([w1, 45.0, w3]))


def test_joint_turn_gains_and_coupling():
    z = np.zeros(3)
    u, r, t = joint_turn(z, z, np.radians([0, 0, -5]), z)
    check(f"elbow -5 deg -> up +5 ({u:+.2f})", abs(u - 5) < 1e-9 and r == 0 and t == 0)
    u, r, t = joint_turn(z, z, z, np.radians([10, 0, 0]))
    check(f"wrist1 +10 -> right -6.2 ({r:+.2f})", abs(r + 6.2) < 1e-9 and u == 0)
    # left/right as the stuck hinge makes it: wrist 3 dragged 0.40 per deg
    u, r, t = joint_turn(z, z, z, np.radians([10, 0, 4]))
    check(f"the 0.40 coupling is taken back out: no twist ({t:+.3f})", abs(t) < 1e-9)
    u, r, t = joint_turn(z, z, z, np.radians([0, 0, 8]))
    check(f"a plain twist of wrist 3 +8 -> twist -8 ({t:+.2f})", abs(t + 8) < 1e-9)


def test_joint_turn_wraps():
    u, r, t = joint_turn(np.zeros(3), np.radians([0, 0, 179.0]),
                         np.zeros(3), np.radians([0, 0, -179.0]))
    check(f"wrist 3 from +179 to -179 is a 2 deg step, not 358 ({t:+.1f})",
          abs(t + 2.0) < 1e-9)


def test_hinge_mode_drives_from_joints():
    m = PoseMap(MapConfig(angle_ratio=2.0, tremor_deg=0.0, hinge_workaround=True))
    m.engage_rotation(_js())
    cmd = m.update(_js(elbow=-10.0))
    check(f"elbow -10 at 2:1 -> psi +5 only {np.round(_pose(cmd), 3)}",
          np.allclose(_pose(cmd), [5, 0, 0], atol=1e-9))
    cmd = m.update(_js(elbow=-10.0, w1=10.0, w3=4.0))
    check(f"then a coupled left/right -> phi only {np.round(_pose(cmd), 3)}",
          np.allclose(_pose(cmd), [5, -3.1, 0], atol=1e-9))


def test_hinge_mode_ignores_the_wrong_orientation():
    """The point of the mode: the device's orientation is wrong with the
    hinge sensor dead, so it must not move anything by itself."""
    m = PoseMap(MapConfig(tremor_deg=0.0, hinge_workaround=True))
    m.engage_rotation(_js())
    cmd = m.update(_js(rotation=_turn(20, [1, 0, 0])))
    check("orientation alone moves nothing", np.allclose(_pose(cmd), 0, atol=1e-12))


def test_hinge_mode_holds_psi_while_translating():
    m = PoseMap(MapConfig(tremor_deg=0.0, angle_ratio=1.0, hinge_workaround=True))
    m.engage_rotation(_js())
    m.engage_translation(_js())
    cmd = m.update(_js(elbow=-12.0, position=[0, 0, -15]))
    check(f"pushing the pen swings the elbow, but psi holds ({cmd.psi_deg:+.2f})",
          abs(cmd.psi_deg) < 1e-12 and cmd.tool_mm > 0)


def test_hinge_mode_switches_without_a_jump():
    cfg = MapConfig(tremor_deg=0.0, angle_ratio=1.0)
    m = PoseMap(cfg)
    m.engage_rotation(_js())
    m.update(_js(elbow=-30.0, rotation=_turn(3, [1, 0, 0])))    # orientation mode
    before = _pose(m.update(_js(elbow=-30.0, rotation=_turn(3, [1, 0, 0]))))
    cfg.hinge_workaround = True                                  # switch mid-stroke
    after = _pose(m.update(_js(elbow=-30.0, rotation=_turn(3, [1, 0, 0]))))
    check(f"switching modes mid-stroke is not a step {np.round(before, 3)} -> "
          f"{np.round(after, 3)}", np.allclose(before, after, atol=1e-12))


if __name__ == "__main__":
    print("\n  mapping self-test\n")
    for fn in [test_convention, test_roundtrip, test_gimbal_lock_flagged,
               test_split_turn_directions,
               test_split_turn_level_pen_in_room_terms,
               test_rotation_clutch_scales, test_rotation_engage_never_steps,
               test_rotation_released_holds,
               test_translation_is_signed, test_sideways_motion_is_ignored,
               test_pointing_axis_stays_latched, test_wall_position_tracks_tool,
               test_clutches_are_independent, test_neither_clutch_holds_everything,
               test_drag_band_ignores_small_movement,
               test_drag_band_drags_rather_than_steps,
               test_drag_band_reversal_costs_two_bands,
               test_drag_band_zero_is_off,
               test_tremor_suppresses_jitter_through_the_mapper,
               test_tremor_applied_before_the_fence,
               test_clamping,
               test_sync_moves_the_wall_to_the_real_tool,
               test_sync_refused_while_engaged,
               test_synced_outside_the_fence_does_not_snap_back,
               test_synced_past_180_does_not_wrap,
               test_twist_is_theta_n_however_the_pen_is_held,
               test_tip_up_is_psi_tip_right_is_phi,
               test_tip_directions_ignore_the_pens_roll,
               test_tip_directions_ignore_theta_n,
               test_joint_turn_gains_and_coupling, test_joint_turn_wraps,
               test_hinge_mode_drives_from_joints,
               test_hinge_mode_ignores_the_wrong_orientation,
               test_hinge_mode_holds_psi_while_translating,
               test_hinge_mode_switches_without_a_jump,
               test_no_dead_zone_off_a_fence,
               test_slow_movement_survives_the_tremor_band,
               test_tremor_band_still_reaches_the_fence]:
        print(f"\n  {fn.__name__}")
        fn()
    print("\n  all checks passed\n")
