"""
Self-test for the mapping maths. Needs no hardware.

    python test_mapping.py

Checks the convention against a hand-built Rx.Ry.Rz, then the two clutches,
the axis-angle scaling, the signed projection and the fences.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from mapping import (MapConfig, PoseMap, drag_band, euler_xyz_deg,
                     matrix_from_euler_xyz_deg, scale_rotation)


class FakeSample:
    """Just enough of touch.Sample for the mapper."""
    def __init__(self, rotation=None, position=None, buttons=0):
        self.rotation = np.eye(3) if rotation is None else rotation
        self.position = np.zeros(3) if position is None else np.asarray(position, float)
        self.buttons  = buttons


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


def test_scale_rotation():
    half = scale_rotation(_turn(60, [0, 0, 1]), 0.5)
    rv = Rotation.from_matrix(half).as_rotvec()
    check("halving a 60 deg rotation gives 30 deg",
          abs(np.degrees(np.linalg.norm(rv)) - 30.0) < 1e-9)
    check("axis is preserved",
          np.allclose(rv / np.linalg.norm(rv), [0, 0, 1.0], atol=1e-9))


def test_euler_scaling_would_be_wrong():
    """The reason scale_rotation exists, made explicit.

    Halving the three Euler terms separately is a DIFFERENT rotation from
    halving the axis-angle. Near zero they agree; at wrist-sized angles they
    do not, and the platform would go somewhere the operator did not point.
    """
    proper = scale_rotation(matrix_from_euler_xyz_deg(50, 40, 60), 0.5)
    naive  = matrix_from_euler_xyz_deg(25, 20, 30)
    diff = np.degrees(Rotation.from_matrix(proper.T @ naive).magnitude())
    check(f"Euler-halving differs by {diff:.1f} deg at (50,40,60)", diff > 5.0)


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
    back = m.update(FakeSample(rotation=_turn(-10, [0, 0, 1]))).theta_n_deg
    check("can come back in", abs(back - 50.0) < 1e-6)


def test_synced_past_180_does_not_wrap():
    m = PoseMap(MapConfig(angle_ratio=1.0, tremor_deg=0.0, theta_n_limit_deg=720.0))
    m.sync_from_rig((0, 0, 200.0), 0.0)
    m.engage_rotation(FakeSample())
    cmd = m.update(FakeSample(rotation=_turn(3, [0, 0, 1])))
    check(f"theta_n 200 + 3 reads 203, not -157 ({cmd.theta_n_deg:.3f})",
          abs(cmd.theta_n_deg - 203.0) < 1e-6)


if __name__ == "__main__":
    print("\n  mapping self-test\n")
    for fn in [test_convention, test_roundtrip, test_gimbal_lock_flagged,
               test_scale_rotation, test_euler_scaling_would_be_wrong,
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
               test_synced_past_180_does_not_wrap]:
        print(f"\n  {fn.__name__}")
        fn()
    print("\n  all checks passed\n")
