"""
The virtual wall at the ends of the tool travel.

The insertion axis is the one place a hard stop is actually possible: it is
driven by stylus TRANSLATION, and translation is the part of the Touch that
has motors behind it. The tilt and yaw fences cannot be walled at all - the
gimbal is sensed but unactuated - so those stay clamped-and-announced.

Everything here is scalar, along the latched pointing direction. The caller
multiplies by the unit vector. Keeping it scalar means it can be tested
properly and called from the servo thread without allocating.

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

# Conservative starting point: 0.30 N/mm reaches the 3.3 N ceiling after
# about 11 mm of penetration, which is firm without being anywhere near the
# stability limit.
DEFAULT_STIFFNESS = 0.30      # [N/mm]
DEFAULT_DAMPING   = 0.002     # [N.s/mm]  = 2 N per m/s
DEFAULT_MAX_FORCE = 3.0       # [N]  under the device's 3.3 N nominal


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
