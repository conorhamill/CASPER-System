"""
Stylus pose -> robot pose. The whole mapping, and nothing else.

Orientation
    The rig's solver uses R = Rx(psi) . Ry(phi) . Rz(theta_n), intrinsic
    Tait-Bryan X-Y'-Z'' - stated in 3R1T_Kin_3R.mh and matched term by term
    by the 1T solver. scipy spells that convention "XYZ" (uppercase means
    intrinsic), so the two agree by construction here.

    The Touch and the robot base are assumed to have IDENTICAL orientation.
    If the device is ever moved, put the alignment rotation in MapConfig
    rather than fixing it up downstream.

    Orientation is CLUTCHED, on its own button. On engage we capture the
    stylus rotation and the robot pose; thereafter only the delta is
    applied. That matters for more than ergonomics: 3R1T_Kin_3R.mh warns
    that its continuity tracker picks whichever IK branch is nearer to last
    cycle's answer, so a step input makes it confidently return the wrong
    pose. Clutching means engaging never produces a step.

    The delta is scaled in AXIS-ANGLE, not by halving the three Euler terms
    separately. Those are not the same operation and they diverge badly by
    the time the wrist is 60 degrees over.

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
    theta_n_limit_deg: float = 45.0
    tool_limit_mm:     float = 20.0

    # Which body axis of the stylus points out of the tip, in the stylus
    # frame. VERIFY THIS ON THE BENCH - run stage0.py --axes, hold the
    # stylus pointing away from you, and see which axis follows it.
    stylus_axis: tuple = (0.0, 0.0, -1.0)

    # Fixed rotation from device world frame to robot base frame.
    align: tuple = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

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


def scale_rotation(R: np.ndarray, factor: float) -> np.ndarray:
    """Take `factor` times the rotation R, about the same axis.

    Axis-angle is the only form in which "half a rotation" means what it
    sounds like. Scaling Euler angles individually is a different operation
    that happens to agree near zero.
    """
    return Rotation.from_rotvec(Rotation.from_matrix(R).as_rotvec() * factor).as_matrix()


def _clamp(value: float, limit: float, name: str, hit: list) -> float:
    if value > limit:
        hit.append(name)
        return limit
    if value < -limit:
        hit.append(name)
        return -limit
    return value


class PoseMap:
    """Holds the commanded robot pose and advances it from stylus motion.

    Two independent clutches. Either, both or neither may be engaged, and
    the commanded pose persists across cycles - releasing freezes, the next
    press continues from where it was left.
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
        self._R0 = self._pose0 = None    # rotation clutch anchors
        self._P0 = self._u = None        # translation clutch anchors, base frame
        self._P0_dev = self._u_dev = None  # the same, device frame, for the wall
        self._tool0 = None
        self._s = 0.0

    # ------------------------------------------------------ rotation clutch
    def engage_rotation(self, sample):
        self._R0    = self._A @ sample.rotation
        self._pose0 = self.pose_deg.copy()
        self.rot_engaged = True

    def release_rotation(self):
        self.rot_engaged = False

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
        return (r * (-lim - self._tool0), r * (lim - self._tool0))

    # ----------------------------------------------------------- the mapping
    def update(self, sample) -> Command:
        cfg = self.cfg
        hit: list[str] = []
        locked = False
        psi, phi, theta_n = self.pose_deg
        tool = self.tool_mm

        if self.rot_engaged:
            R = self._A @ sample.rotation
            # Delta in the world/base frame, so a wrist rotation about base X
            # becomes a platform rotation about base X.
            dR = scale_rotation(R @ self._R0.T, 1.0 / cfg.angle_ratio)
            target = dR @ matrix_from_euler_xyz_deg(*self._pose0)
            (psi, phi, theta_n), locked = euler_xyz_deg(target)
            psi     = _clamp(psi,     cfg.psi_limit_deg,     "psi",     hit)
            phi     = _clamp(phi,     cfg.phi_limit_deg,     "phi",     hit)
            theta_n = _clamp(theta_n, cfg.theta_n_limit_deg, "theta_n", hit)
            self.pose_deg[:] = (psi, phi, theta_n)

        if self.trans_engaged:
            P = self._A @ sample.position
            self._s = float((P - self._P0) @ self._u)
            tool = _clamp(self._tool0 + self._s / cfg.tool_ratio,
                          cfg.tool_limit_mm, "tool", hit)
            self.tool_mm = tool

        return Command(psi, phi, theta_n, tool,
                       self.rot_engaged, self.trans_engaged,
                       self._s, tuple(hit), locked)
