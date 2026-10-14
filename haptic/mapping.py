"""
Stylus pose -> robot pose. The whole mapping, and nothing else.

Orientation
    The rig's solver uses R = Rx(psi) . Ry(phi) . Rz(theta_n), intrinsic
    Tait-Bryan X-Y'-Z'' - stated in 3R1T_Kin_3R.mh and matched term by term
    by the 1T solver. scipy spells that convention "XYZ" (uppercase means
    intrinsic), so the two agree by construction here.

    >>> TIP UP IS +PSI, TIP RIGHT IS +PHI, TWIST IS THETA_N. <<<

    Orientation is CLUTCHED, on its own button, and on every press the pen
    counts as being at zero wherever it is held. From then on, each cycle's
    turn of the pen (a rotation - scipy holds it as a quaternion) is split
    three ways:

        tip moving up / down        as seen from in front of the Touch   psi
        tip moving right / left     as seen from in front of the Touch   phi
        twist about the pen's own long axis                          theta_n

    and each part is added to its angle. That matches the rig as seen from
    the operator's side of the bench (measured 2026-10-14): +psi tilts the
    needle up, +phi tilts it right.

    >>> UP AND RIGHT ARE THE ROOM'S, NOT THE PEN'S. <<<
    The first version of this measured tip movement about the pen's own
    axes and applied it about the platform's own axes. Twist was right, but
    which way counted as "left" then depended on how the pen happened to be
    rolled in the fingers and how far theta_n had been spun - left/right
    came out as up/down. So up/down and left/right are taken against the
    Touch's own X (to the right), whatever the pen's roll or theta_n.

    It ADDS UP SMALL TURNS, one per cycle, rather than measuring the total
    turn since the press. The point is the fences: the overshoot past a
    limit is thrown away as it happens, so reversing moves the platform
    straight off the limit instead of first having to unwind the overshoot.
    The cost is path dependence - wave the pen in a loop and come back, and
    the platform ends a hair from where it started. Nobody can feel it, and
    with every press starting from zero there is no "home" for the pen to
    come back to anyway.

    >>> BROKEN-HINGE MODE (MapConfig.hinge_workaround) <<<
    This Touch's wrist pitch sensor is dead, so while it is away for repair
    the three parts come from three Touch joints instead of the pen's
    orientation. See MapConfig for the figures and where they came from.

    Engaging never produces a step, which matters for more than ergonomics:
    3R1T_Kin_3R.mh warns that its continuity tracker picks whichever IK
    branch is nearer to last cycle's answer, so a step input makes it
    confidently return the wrong pose.

Translation
    Clutched on a SEPARATE button, so the tool can be aimed without being
    driven and driven without being re-aimed. On engage we latch the anchor
    point and the direction the stylus is pointing; thereafter the signal is
    the SIGNED projection of travel onto that direction, so push advances
    and pull retracts. A plain Euclidean distance would be unsigned, and the
    needle could never come back out.

    The pointing direction is latched at the moment of engage and does NOT
    follow the stylus afterwards. Holding both clutches at once is allowed,
    and with a swimming axis the projection would drift as you rotated.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

# Beyond this the XYZ decomposition degenerates (phi -> +/-90 deg) and psi
# and theta_n stop being separable.
_GIMBAL_LOCK_SIN = 0.9995


@dataclass
class MapConfig:
    # Reductions: stylus units per robot unit. 2.0 means 2 deg of wrist per
    # 1 deg of platform, so the fence is reached at twice the wrist angle.
    angle_ratio: float = 2.0
    tool_ratio:  float = 3.0

    # Robot fences. These mirror LIM_* in 3R1T_Config.mh; keep them in step.
    psi_limit_deg:     float = 30.0
    phi_limit_deg:     float = 30.0
    theta_n_limit_deg: float = 90.0     # PC-only fence; the rig allows +/-720
    tool_limit_mm:     float = 20.0

    # Tremor band for the three angles, in degrees. A hand holding a stylus
    # still is not still. Comes from USR_TELE_TREMOR so the panel can change
    # it live. Zero disables it.
    tremor_deg: float = 1.0

    # Which body axis of the stylus points out of the tip, in the stylus
    # frame. VERIFY THIS ON THE BENCH - run stage0.py --axes, hold the
    # stylus pointing away from you, and see which axis follows it.
    stylus_axis: tuple = (0.0, 0.0, -1.0)

    # Fixed rotation from device world frame to robot base frame. Used by
    # TRANSLATION only - rotation is split into tip up / tip right / twist
    # and needs no frame of its own.
    align: tuple = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

    # Direction of each part of the pen's turn: (tip up -> psi, tip right ->
    # phi, twist -> theta_n). +1 or -1. Set from the rig on 2026-10-14; if a
    # direction ever comes out backwards, flip it here and nowhere else.
    turn_signs: tuple = (1.0, 1.0, 1.0)

    # >>> BROKEN-HINGE MODE. TEMPORARY, UNTIL THE TOUCH IS REPAIRED. <<<
    #
    # This Touch's wrist pitch sensor reads a constant 45 deg (found
    # 2026-10-14), so the pen's orientation as the device reports it is
    # wrong whenever the hinge is anywhere else, and up/down can only come
    # from swinging the forearm. In this mode the turn is read straight off
    # three Touch joints instead of the pen's orientation:
    #
    #     up     = joint_gains[0] x change of arm joint 3 (the elbow)
    #     right  = joint_gains[1] x change of wrist joint 1
    #     twist  = joint_gains[2] x (change of wrist joint 3
    #                                - twist_coupling x change of wrist joint 1)
    #
    # and the GUI holds arm joints 1 and 2 while the rotation clutch is
    # down, so the forearm can only swing about the elbow.
    #
    # Every figure here was fitted from lock_probe_joint.csv (firm joint
    # lock, 2026-10-14), against the orientation mapping as the reference
    # for direction and size:
    #     up    = -0.86 x d(joint 3), corr -0.99 - taken as -1.0, since 1 deg
    #             of forearm is 1 deg of tip
    #     right = -0.62 x d(wrist 1), corr -0.99
    #     twist = -0.99 x d(wrist 3 - 0.40 wrist 1), corr -1.00
    #     wrist 3 moves +0.40 deg per deg of wrist 1 during left/right: the
    #     stuck hinge makes a left/right swing drag the twist joint with it
    # With these, left/right lands 86% on phi (48% from orientation) and
    # up/down 67% on psi (49%). Twist is 82% either way.
    hinge_workaround: bool = False
    joint_gains: tuple = (-1.0, -0.62, -1.0)
    twist_coupling: float = 0.40

    @classmethod
    def from_config_mh(cls, path, **overrides) -> "MapConfig":
        """Take the fences from 3R1T_Config.mh instead of duplicating them.

        The PC clamps before sending and the MACS clamps again on arrival.
        Two copies of the same number in two languages is exactly the kind
        of thing that drifts, and a PC fence looser than the rig's would
        send commands the rig silently refuses.

        theta_n is deliberately NOT taken from the file. LIM_THETA_N_CDEG is
        +/-720 deg because the mechanism has an infinite-roll axis; the limit
        that actually binds is how far a wrist turns, which is a mapping
        choice and lives here.
        """
        import re
        from pathlib import Path
        text = Path(path).read_text(errors="replace")

        def define(name: str):
            m = re.search(rf"^#define\s+{name}\s+(\d+)", text, re.MULTILINE)
            if not m:
                raise ValueError(f"{name} not found in {path}")
            return int(m.group(1))

        fields = dict(
            psi_limit_deg = define("LIM_PSI_CDEG") / 100.0,
            phi_limit_deg = define("LIM_PHI_CDEG") / 100.0,
            tool_limit_mm = define("LIM_TOOL_MM100") / 100.0,
            tremor_deg    = define("C_TELE_TREMOR_DEF") / 100.0,
        )
        fields.update(overrides)
        return cls(**fields)


@dataclass
class Command:
    psi_deg:       float = 0.0
    phi_deg:       float = 0.0
    theta_n_deg:   float = 0.0
    tool_mm:       float = 0.0
    rot_engaged:   bool  = False
    trans_engaged: bool  = False
    s_mm:          float = 0.0        # raw signed travel along the latched axis
    clamped:       tuple = ()         # names of axes sitting on a fence
    gimbal_lock:   bool  = False


def euler_xyz_deg(R: np.ndarray) -> tuple[np.ndarray, bool]:
    """Decompose to (psi, phi, theta_n) in degrees, intrinsic XYZ.

    Returns the angles and a flag saying whether the decomposition was near
    its singularity, where psi and theta_n stop being independent.
    """
    locked = abs(float(R[0, 2])) > _GIMBAL_LOCK_SIN
    return Rotation.from_matrix(R).as_euler("XYZ", degrees=True), locked


def matrix_from_euler_xyz_deg(psi: float, phi: float, theta_n: float) -> np.ndarray:
    return Rotation.from_euler("XYZ", [psi, phi, theta_n], degrees=True).as_matrix()


# The Touch's +X: to the right, seen from in front of the device.
_ROOM_RIGHT = np.array([1.0, 0.0, 0.0])
_ROOM_TOWARD_USER = np.array([0.0, 0.0, 1.0])


def split_turn(R_prev: np.ndarray, R_now: np.ndarray, right_prev=None,
               tip_axis=(0.0, 0.0, -1.0)):
    """Split the pen's turn from R_prev to R_now into (up, right, twist) [deg].

        up      the tip moving up        - turn about the room's right axis,
                                           squared off against the pen
        right   the tip moving right     - turn about the axis at right
                                           angles to that and to the pen
        twist   about the pen's own long axis, positive the way the right
                hand curls with the thumb pointing BACK up the pen (away
                from the tip). tip_axis is MapConfig.stylus_axis.

    The three axes are at right angles, so the three parts add back up to
    the whole turn. They are meant for the small turn of one 10 ms cycle;
    for a big turn the split is only approximate.

    Also returns the right axis used, to pass back as `right_prev` next
    cycle. Only the pen pointing almost straight left or right leaves the
    room's right axis undefined, and then the last one is reused.
    """
    w = Rotation.from_matrix(R_now @ R_prev.T).as_rotvec()     # room frame [rad]
    d = R_prev @ np.asarray(tip_axis, dtype=float)             # where the tip points
    d = d / np.linalg.norm(d)
    z = -d
    x = _ROOM_RIGHT - (_ROOM_RIGHT @ d) * d
    n = float(np.linalg.norm(x))
    if n < 0.3:
        if right_prev is not None:
            x = np.asarray(right_prev, dtype=float)
        else:
            # No history either: square off against "toward the user"
            # instead, which the pen cannot also be pointing along.
            x = np.cross(_ROOM_TOWARD_USER, d)
            x = x / np.linalg.norm(x)
    else:
        x = x / n
    a = np.cross(d, x)            # turning about this moves the tip towards +x
    up, right, twist = np.degrees([w @ x, w @ a, w @ z])
    return float(up), float(right), float(twist), x


def _wrap_deg(d: float) -> float:
    """A joint step folded into -180..180, so a joint reading that wraps
    round from +180 to -180 reads as a small step and not a whole turn."""
    return (d + 180.0) % 360.0 - 180.0


def joint_turn(joints_prev, gimbal_prev, joints_now, gimbal_now,
               gains=(-1.0, -0.62, -1.0), coupling=0.40):
    """The broken-hinge mode's (up, right, twist) [deg] - see MapConfig.

    joints are the Touch's three ARM joint angles, gimbal its three WRIST
    joint angles, both in radians as the device reports them.
    """
    d_elbow = _wrap_deg(np.degrees(joints_now[2] - joints_prev[2]))
    d_w1    = _wrap_deg(np.degrees(gimbal_now[0] - gimbal_prev[0]))
    d_w3    = _wrap_deg(np.degrees(gimbal_now[2] - gimbal_prev[2]))
    g_up, g_right, g_twist = gains
    return (float(g_up * d_elbow),
            float(g_right * d_w1),
            float(g_twist * (d_w3 - coupling * d_w1)))


def drag_band(demand: float, held: float, band: float) -> float:
    """Hysteresis band, for suppressing hand tremor.

    Nothing within `band` of the held value moves it at all - which is what
    stops the rig buzzing while somebody holds the stylus still.

    >>> IT DRAGS. IT DOES NOT STEP. <<<

    Once the demand escapes the band the held value is pulled along behind
    it, staying exactly `band` away. A plain deadband would instead sit
    still and then jump by a whole band the moment it broke, so a 1 deg
    band would mean 1 deg steps - worse than the tremor it was fixing. The
    cost of dragging is a permanent lag of one band in the direction of
    travel, and two bands of dead zone when reversing.
    """
    if band <= 0.0:
        return demand
    if demand - held > band:
        return demand - band
    if held - demand > band:
        return demand + band
    return held


def _clamp(value: float, limit: float, name: str, hit: list,
           held: float = 0.0) -> float:
    """Fence at +/-limit - but never pull back towards it.

    After sync_from_rig the held value can already be OUTSIDE the fence: a
    trajectory left theta_n at 200 deg, say. A plain clamp would turn the
    first update into a 155 deg step back to the fence. Instead a held value
    outside the fence becomes the fence on that side, so the operator can
    come in from it freely and cannot go further out.
    """
    hi = max(limit, held)
    lo = min(-limit, held)
    if value > hi:
        hit.append(name)
        return hi
    if value < lo:
        hit.append(name)
        return lo
    return value


class PoseMap:
    """Holds the commanded robot pose and advances it from stylus motion.

    Two independent clutches. Either, both or neither may be engaged, and
    the commanded pose persists across cycles - releasing freezes, the next
    press continues from where it was left, or from where the rig actually
    got to if the caller syncs it first (sync_from_rig).
    """

    def __init__(self, cfg: MapConfig | None = None):
        self.cfg  = cfg or MapConfig()
        self._A   = np.array(self.cfg.align, dtype=float)
        self._axis = np.array(self.cfg.stylus_axis, dtype=float)
        self._axis /= np.linalg.norm(self._axis)
        self.pose_deg = np.zeros(3)      # psi, phi, theta_n - commanded
        self.tool_mm  = 0.0
        self.rot_engaged   = False
        self.trans_engaged = False
        # Rotation clutch state. _R_prev is the pen last cycle, so each
        # cycle's turn is measured against it. _raw is what the turns add up
        # to BEFORE the tremor band - see update(). _right is split_turn's
        # last right axis.
        self._R_prev = None
        self._right = None
        self._raw = np.zeros(3)
        # The same for broken-hinge mode: last cycle's arm and wrist joints.
        self._j_prev = self._g_prev = None
        self._P0 = self._u = None        # translation clutch anchors, base frame
        self._P0_dev = self._u_dev = None  # the same, device frame, for the wall
        self._tool0 = None
        self._s = 0.0

    # ------------------------------------------------------- resync
    def sync_from_rig(self, pose_deg, tool_mm) -> bool:
        """Restart the commanded pose from where the rig actually is.

        Releasing both clutches stops the rig where it has got to, which
        after a fast move is short of what was commanded. The rig re-bases
        on the next press from its real position; this makes the PC do the
        same, so its fences and the haptic wall are measured from the real
        pose and not from a target that was never reached.

        Only while neither clutch is held - mid-follow, changing these would
        be sent as a step. Returns whether it synced.
        """
        if self.rot_engaged or self.trans_engaged:
            return False
        self.pose_deg[:] = [float(v) for v in pose_deg]
        self.tool_mm = float(tool_mm)
        return True

    # ------------------------------------------------------ rotation clutch
    def engage_rotation(self, sample):
        # The pen counts as zero from here: only turns from now on count.
        self._R_prev = np.asarray(sample.rotation, dtype=float)
        self._right = None
        self._j_prev, self._g_prev = self._joints_of(sample)
        self._raw = self.pose_deg.copy()
        self.rot_engaged = True

    def release_rotation(self):
        self.rot_engaged = False

    @staticmethod
    def _joints_of(sample):
        """The sample's arm and wrist joints, or (None, None) for a sample
        that does not carry them (the tests' fakes)."""
        j, g = getattr(sample, "joints", None), getattr(sample, "gimbal", None)
        if j is None or g is None:
            return None, None
        return np.array(j, dtype=float), np.array(g, dtype=float)

    # --------------------------------------------------- translation clutch
    def engage_translation(self, sample):
        R = self._A @ sample.rotation
        self._P0 = self._A @ sample.position
        # Latched here and deliberately NOT updated while engaged: if it
        # followed the stylus, rotating with both clutches held would make
        # the projection drift.
        u = R @ self._axis
        self._u = u / np.linalg.norm(u)
        # The same anchor and axis in DEVICE coordinates. The servo thread
        # renders the wall in raw device space and knows nothing about the
        # robot's base frame, so it needs these rather than the base-frame
        # pair above. Identical while `align` is the identity; not once it
        # is not, which is exactly when a duplicated derivation would rot.
        self._P0_dev = np.asarray(sample.position, dtype=float)
        u_dev = sample.rotation @ self._axis
        self._u_dev = u_dev / np.linalg.norm(u_dev)
        self._tool0 = self.tool_mm
        self.trans_engaged = True

    @property
    def wall_anchor_device(self) -> np.ndarray:
        return self._P0_dev

    @property
    def wall_axis_device(self) -> np.ndarray:
        return self._u_dev

    def release_translation(self):
        self.trans_engaged = False
        self._s = 0.0

    def travel_limits_mm(self) -> tuple[float, float]:
        """Signed stylus travel along the latched axis that reaches the tool
        fence, given where the tool was when the clutch was pressed.

        This is where the virtual wall goes. It MOVES with every press,
        because it depends on the tool position at engage.
        """
        if not self.trans_engaged:
            return (0.0, 0.0)
        r, lim = self.cfg.tool_ratio, self.cfg.tool_limit_mm
        # Same never-pull-back rule as _clamp: a tool synced to outside the
        # fence gets its wall where it already is, not behind the stylus.
        lo, hi = min(-lim, self._tool0), max(lim, self._tool0)
        return (r * (lo - self._tool0), r * (hi - self._tool0))

    # ----------------------------------------------------------- the mapping
    def update(self, sample) -> Command:
        cfg = self.cfg
        hit: list[str] = []
        locked = False
        psi, phi, theta_n = self.pose_deg
        tool = self.tool_mm

        if self.rot_engaged:
            R = np.asarray(sample.rotation, dtype=float)
            # This cycle's turn of the pen, split into tip up / tip right /
            # twist, each scaled and added to its own angle. Added straight
            # onto the angles, so theta_n counts on past 180 with no wrap.
            #
            # Both sources are tracked every cycle, whichever is in use, so
            # the mode can be switched mid-stroke without a jump.
            up, right, twist, self._right = split_turn(
                self._R_prev, R, self._right, cfg.stylus_axis)
            self._R_prev = R
            j, g = self._joints_of(sample)
            if cfg.hinge_workaround:
                if self._j_prev is not None and j is not None:
                    up, right, twist = joint_turn(self._j_prev, self._g_prev, j, g,
                                                  cfg.joint_gains, cfg.twist_coupling)
                else:
                    up = right = twist = 0.0          # no joints yet: hold
                # Pushing the pen along its axis swings the elbow too, so
                # with the translation clutch also held the elbow is not
                # up/down. Hold psi while translating.
                if self.trans_engaged:
                    up = 0.0
            self._j_prev, self._g_prev = j, g
            k = 1.0 / cfg.angle_ratio
            su, sr, st = cfg.turn_signs
            rp = self._raw[0] + su * up    * k
            rh = self._raw[1] + sr * right * k
            rn = self._raw[2] + st * twist * k

            # >>> THE TREMOR BAND WORKS ON THE TOTAL, NOT ON EACH TURN. <<<
            # A turn per 10 ms cycle is far smaller than a 1 deg band, so a
            # band applied per turn would throw away every slow movement.
            # So the turns add up in _raw, unfiltered, and the band sits
            # between _raw and what is sent.
            #
            # _raw is fenced one band WIDER than the limit. The output trails
            # _raw by up to a band, so fenced at the limit itself the output
            # could never quite reach it. The overshoot past that is
            # dropped here, every cycle - which is what makes reversing off
            # a fence immediate.
            band = cfg.tremor_deg
            rp = _clamp(rp, cfg.psi_limit_deg + band,     "psi",     hit, self._raw[0])
            rh = _clamp(rh, cfg.phi_limit_deg + band,     "phi",     hit, self._raw[1])
            rn = _clamp(rn, cfg.theta_n_limit_deg + band, "theta_n", hit, self._raw[2])
            self._raw[:] = (rp, rh, rn)

            held = self.pose_deg
            psi     = drag_band(rp, held[0], band)
            phi     = drag_band(rh, held[1], band)
            theta_n = drag_band(rn, held[2], band)
            psi     = _clamp(psi,     cfg.psi_limit_deg,     "psi",     hit, held[0])
            phi     = _clamp(phi,     cfg.phi_limit_deg,     "phi",     hit, held[1])
            theta_n = _clamp(theta_n, cfg.theta_n_limit_deg, "theta_n", hit, held[2])
            self.pose_deg[:] = (psi, phi, theta_n)

        if self.trans_engaged:
            P = self._A @ sample.position
            self._s = float((P - self._P0) @ self._u)
            tool = _clamp(self._tool0 + self._s / cfg.tool_ratio,
                          cfg.tool_limit_mm, "tool", hit, self.tool_mm)
            self.tool_mm = tool

        return Command(psi, phi, theta_n, tool,
                       self.rot_engaged, self.trans_engaged,
                       self._s, tuple(dict.fromkeys(hit)), locked)
