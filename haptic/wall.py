"""
The haptic guides: the virtual wall at the ends of the tool travel, and the
pen lock that holds the pen still while it is being turned.

    rotation clutch alone     LOCK. A spring holds the pen's position at
                              where it was when the clutch went down, so
                              the hand can only turn the pen. The gimbal
                              has no motors, so turning stays free.
    translation clutch        END WALLS only. The pen is free in every
    (or both)                 direction until the tool reaches its fence,
                              then pushed back along the latched axis.

A sideways "rail" on translation was tried on 2026-10-14 and taken out the
same day: the operator wanted the pen free, with only the end stops.

Like the wall, the lock is soft - see the figures below. It steadies the
hand; a deliberate shove goes through it.

THE END WALLS

The insertion axis is the one place a hard stop is actually possible: it is
driven by stylus TRANSLATION, and translation is the part of the Touch that
has motors behind it. The tilt and yaw fences cannot be walled at all - the
gimbal is sensed but unactuated - so those stay clamped-and-announced.

wall_force is scalar, along the latched pointing direction; end_wall_force
multiplies it by the unit vector. Plain floats throughout, so all of it can
be tested properly and called from the servo thread without numpy.

    s          signed travel along the latched axis        [mm]
    lo, hi     where the wall sits, from PoseMap           [mm]
    stiffness  wall spring rate                            [N/mm]
    damping    opposes motion while penetrating            [N.s/mm]
    s_dot      speed along the axis                        [mm/s]

WHAT THIS WALL CAN AND CANNOT BE

Read off this Touch, by the driver's own account:

    max force             3.30 N      peak, briefly
    max CONTINUOUS force  0.88 N      what you get if you lean on it
    max stiffness         0.50 N/mm   above this it will not stay stable
    max damping           0.003 N.s/mm

So at the stiffest the device will tolerate, 0.88 N of sustained push
arrives after 1.8 mm of penetration and the 3.3 N peak after 6.6 mm. That
is the wall: a firm spring with a few millimetres of give, which fades if
you lean on it. It is not a steel stop and no amount of tuning makes it
one - past 0.50 N/mm it buzzes rather than stiffens.

The defaults below sit at 60% of the device's stiffness ceiling and 67% of
its damping ceiling, which is a sane place to start rather than a limit.
"""
from __future__ import annotations

import math

# Conservative starting point: 0.30 N/mm reaches the 3.3 N ceiling after
# about 11 mm of penetration, which is firm without being anywhere near the
# stability limit.
DEFAULT_STIFFNESS = 0.30      # [N/mm]
DEFAULT_DAMPING   = 0.002     # [N.s/mm]  = 2 N per m/s
DEFAULT_MAX_FORCE = 3.0       # [N]  under the device's 3.3 N nominal

# The pen lock.
#
# >>> CAPPED LOWER THAN THE WALL, AND ON PURPOSE. <<<
# The end walls are only touched at the ends of travel. The lock is loaded
# the whole time the clutch is held, by however much the hand drifts, and the
# device is only good for 0.88 N continuous. 2 N arrives at 8 mm of drift,
# which is already far more than a hand turning a pen should wander.
DEFAULT_LOCK_STIFFNESS = 0.25     # [N/mm]
DEFAULT_LOCK_DAMPING   = 0.002    # [N.s/mm]
DEFAULT_LOCK_MAX_FORCE = 2.0      # [N]

# The JOINT lock: springs on chosen arm joints, in joint space. Exists
# because this Touch's wrist pitch sensor is dead (2026-10-14), so up/down
# has to come from the arm's third joint - which a position lock would hold.
#
# The arm links are about 133 mm, so 1 rad of joint is ~133 mm at the pen.
# 3000 mNm/rad is then ~0.17 N/mm there - a third of the device's 0.5 N/mm
# stability ceiling. 250 mNm is ~1.9 N at the pen: inside the 3.3 N peak but
# above the 0.88 N continuous rating, which is fine for a spring that only
# loads when pushed off its anchor; lean on it for long and the driver's own
# thermal protection cuts force output.
#
# 2026-10-14: raised from 1000 mNm/rad and 120 mNm after the first probe -
# the operator could push joint 1 twelve degrees off with no effort.
DEFAULT_JOINT_STIFFNESS = 3000.0  # [mNm/rad]
DEFAULT_JOINT_MAX_TORQUE = 250.0  # [mNm] per joint
# A locked joint further than this from its anchor means the lock is not
# holding it - most likely pushing the wrong way. The servo loop drops the
# lock rather than keep pushing.
JOINT_LOCK_TRIP_RAD = math.radians(20.0)


def wall_force(s: float, lo: float, hi: float,
               stiffness: float = DEFAULT_STIFFNESS,
               damping: float = DEFAULT_DAMPING,
               s_dot: float = 0.0,
               f_max: float = DEFAULT_MAX_FORCE) -> float:
    """Force along the latched axis. Negative pushes back towards -axis.

    Zero inside [lo, hi]. Outside, a spring-damper that only ever pushes the
    operator back OUT.
    """
    if lo <= s <= hi:
        return 0.0

    if s > hi:
        f = -stiffness * (s - hi) - damping * s_dot
        # >>> NEVER PULL THE OPERATOR FURTHER IN. <<<
        # Retreating fast while still penetrating makes -damping * s_dot
        # large and POSITIVE, which without this clamp would suck the stylus
        # deeper into the wall - the classic way a damped virtual wall turns
        # into a virtual magnet.
        f = min(f, 0.0)
    else:
        f = stiffness * (lo - s) - damping * s_dot
        f = max(f, 0.0)

    return max(-f_max, min(f_max, f))


def cap(fx: float, fy: float, fz: float, f_max: float):
    """Scale a force down to f_max in MAGNITUDE, keeping its direction.
    Clipping each component instead would bend it off its line."""
    m = math.sqrt(fx * fx + fy * fy + fz * fz)
    if m > f_max > 0.0:
        k = f_max / m
        return fx * k, fy * k, fz * k
    return fx, fy, fz


def lock_force(ex: float, ey: float, ez: float,
               vx: float, vy: float, vz: float,
               stiffness: float = DEFAULT_LOCK_STIFFNESS,
               damping: float = DEFAULT_LOCK_DAMPING,
               f_max: float = DEFAULT_LOCK_MAX_FORCE):
    """Spring back to the anchor. e = position - anchor [mm], v [mm/s].

    Unlike the wall this is two-sided, so the damping term only ever
    opposes motion and cannot turn into a magnet.
    """
    return cap(-stiffness * ex - damping * vx,
               -stiffness * ey - damping * vy,
               -stiffness * ez - damping * vz, f_max)


def end_wall_force(ex: float, ey: float, ez: float,
                   vx: float, vy: float, vz: float,
                   ux: float, uy: float, uz: float,
                   lo: float, hi: float,
                   stiffness: float = DEFAULT_STIFFNESS,
                   damping: float = DEFAULT_DAMPING,
                   f_max: float = DEFAULT_MAX_FORCE):
    """The end walls as a force vector: wall_force along u, nothing in any
    other direction - sideways the pen is free. u must be a unit vector.

    Returns (fx, fy, fz, s, f_axial) - s and the axial force for display.
    """
    s     = ex * ux + ey * uy + ez * uz
    s_dot = vx * ux + vy * uy + vz * uz
    f_ax  = wall_force(s, lo, hi, stiffness, damping, s_dot, f_max)
    return f_ax * ux, f_ax * uy, f_ax * uz, s, f_ax


def joint_lock_torque(e1: float, e2: float, e3: float,
                      m1: float, m2: float, m3: float,
                      stiffness: float = DEFAULT_JOINT_STIFFNESS,
                      t_max: float = DEFAULT_JOINT_MAX_TORQUE,
                      sign: float = 1.0):
    """Spring torque back to the anchor on each locked joint [mNm].

    e = joint angle - anchor [rad]; m = 1.0 to lock that joint, 0.0 to leave
    it free. `sign` flips the convention if the device turns out to drive
    positive torque towards negative angle.
    """
    def one(e, m):
        if m == 0.0:
            return 0.0
        t = -sign * stiffness * e
        return max(-t_max, min(t_max, t))
    return one(e1, m1), one(e2, m2), one(e3, m3)


def rate_limit_vec(tx: float, ty: float, tz: float,
                   px: float, py: float, pz: float, max_step: float):
    """rate_limit for a force vector: the CHANGE is capped in magnitude, so
    switching between the lock, the walls and off is never a kick."""
    dx, dy, dz = tx - px, ty - py, tz - pz
    m = math.sqrt(dx * dx + dy * dy + dz * dz)
    if m > max_step:
        k = max_step / m
        return px + dx * k, py + dy * k, pz + dz * k
    return tx, ty, tz


def rate_limit(target: float, previous: float, max_step: float) -> float:
    """Limit how fast the commanded force may change, per tick.

    Guards against a discontinuity kicking the device - on the first tick
    after the wall parameters move, or if a clutch is pressed somewhere
    unexpected. At 1 kHz, 0.05 N per tick is 50 N/s, which reaches full
    scale in about 60 ms: fast enough to feel instant, slow enough not to
    snap the arm out of your hand.
    """
    delta = target - previous
    if delta > max_step:
        return previous + max_step
    if delta < -max_step:
        return previous - max_step
    return target
