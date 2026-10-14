"""
Self-test for the virtual wall. Needs no hardware.

    python test_wall.py
"""
from __future__ import annotations

import math

from wall import (DEFAULT_LOCK_MAX_FORCE, DEFAULT_LOCK_STIFFNESS, joint_lock_torque,
                  DEFAULT_MAX_FORCE, end_wall_force, lock_force, rate_limit,
                  rate_limit_vec, wall_force)


def _mag(f):
    return math.sqrt(f[0] ** 2 + f[1] ** 2 + f[2] ** 2)


def check(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")
    if not cond:
        raise AssertionError(name)


def test_free_inside():
    for s in (-60.0, -30.0, 0.0, 30.0, 60.0):
        check(f"no force at s={s:+.0f} inside [-60, 60]",
              wall_force(s, -60.0, 60.0) == 0.0)


def test_pushes_back_at_each_end():
    f = wall_force(65.0, -60.0, 60.0, stiffness=0.3)
    check("past the far wall, force is negative (pushes back)", f < 0.0)
    check("and is stiffness x penetration", abs(f + 0.3 * 5.0) < 1e-12)

    f = wall_force(-65.0, -60.0, 60.0, stiffness=0.3)
    check("past the near wall, force is positive", f > 0.0)
    check("and is stiffness x penetration", abs(f - 0.3 * 5.0) < 1e-12)


def test_grows_with_penetration():
    a = abs(wall_force(62.0, -60.0, 60.0))
    b = abs(wall_force(66.0, -60.0, 60.0))
    check("deeper penetration means more force", b > a)


def test_clamped_to_max():
    f = wall_force(1000.0, -60.0, 60.0, stiffness=0.3)
    check(f"clamped to {DEFAULT_MAX_FORCE} N however deep", abs(f) <= DEFAULT_MAX_FORCE + 1e-12)
    check("and is at the ceiling", abs(abs(f) - DEFAULT_MAX_FORCE) < 1e-12)


def test_damping_opposes_entry():
    still  = wall_force(62.0, -60.0, 60.0, s_dot=0.0)
    diving = wall_force(62.0, -60.0, 60.0, s_dot=200.0)
    check("pushing further in adds resistance", abs(diving) > abs(still))


def test_damping_never_pulls_you_in():
    """The classic damped-wall failure: a virtual wall that becomes a magnet."""
    f = wall_force(62.0, -60.0, 60.0, stiffness=0.3, damping=0.01, s_dot=-500.0)
    check("retreating fast from the far wall never gives a positive force", f <= 0.0)
    f = wall_force(-62.0, -60.0, 60.0, stiffness=0.3, damping=0.01, s_dot=+500.0)
    check("retreating fast from the near wall never gives a negative force", f >= 0.0)


def test_asymmetric_walls():
    """After a clutch press with the tool already advanced the walls are not
    symmetric - see PoseMap.travel_limits_mm."""
    check("free inside an offset window", wall_force(-80.0, -90.0, 30.0) == 0.0)
    check("blocked past its short end", wall_force(35.0, -90.0, 30.0) < 0.0)
    check("free where a symmetric wall would have blocked",
          wall_force(-85.0, -90.0, 30.0) == 0.0)


def test_rate_limit():
    check("small steps pass through", rate_limit(0.30, 0.28, 0.05) == 0.30)
    check("big rises are capped", abs(rate_limit(3.0, 0.0, 0.05) - 0.05) < 1e-12)
    check("big falls are capped", abs(rate_limit(-3.0, 0.0, 0.05) + 0.05) < 1e-12)
    check("converges", abs(rate_limit(0.05, 0.05, 0.05) - 0.05) < 1e-12)


def test_rate_limit_reaches_full_scale_quickly():
    f, ticks = 0.0, 0
    while abs(f - DEFAULT_MAX_FORCE) > 1e-9 and ticks < 10_000:
        f = rate_limit(DEFAULT_MAX_FORCE, f, 0.05)
        ticks += 1
    check(f"0 to {DEFAULT_MAX_FORCE} N takes {ticks} ticks ({ticks} ms at 1 kHz)",
          ticks <= 100)


# ------------------------------------------------------------------ lock
def test_lock_pulls_back_to_the_anchor():
    k = DEFAULT_LOCK_STIFFNESS
    f = lock_force(2.0, -1.0, 0.0, 0, 0, 0)
    check("no force at the anchor", lock_force(0, 0, 0, 0, 0, 0) == (0.0, 0.0, 0.0))
    check("2 mm right, 1 mm down pulls left and up by k x drift",
          abs(f[0] + 2.0 * k) < 1e-12 and abs(f[1] - 1.0 * k) < 1e-12 and f[2] == 0.0)


def test_lock_capped_in_magnitude_not_per_axis():
    f = lock_force(100.0, 100.0, 0.0, 0, 0, 0)
    check(f"capped at {DEFAULT_LOCK_MAX_FORCE} N",
          abs(_mag(f) - DEFAULT_LOCK_MAX_FORCE) < 1e-9)
    check("and still pointing straight back at the anchor", abs(f[0] - f[1]) < 1e-12)


# ------------------------------------------------------------ end walls
def test_end_walls_free_inside():
    fx, fy, fz, s, f_ax = end_wall_force(0, 0, -25.0, 0, 0, 0, 0, 0, -1.0, -60, 60)
    check(f"25 mm along the axis, inside the walls: no force (s={s:+.0f})",
          (fx, fy, fz) == (0.0, 0.0, 0.0) and abs(s - 25.0) < 1e-12)


def test_end_walls_free_sideways():
    """No rail: off the axis sideways is free, however far."""
    fx, fy, fz, _, _ = end_wall_force(30.0, -20.0, -25.0, 0, 0, 0, 0, 0, -1.0, -60, 60)
    check("30 mm sideways: no force", (fx, fy, fz) == (0.0, 0.0, 0.0))


def test_end_wall_pushes_back_along_the_axis():
    fx, fy, fz, s, f_ax = end_wall_force(5.0, 0, -65.0, 0, 0, 0, 0, 0, -1.0, -60, 60)
    check("past the far end the wall pushes back along -u (i.e. +z) only",
          f_ax < 0.0 and fz > 0.0 and fx == 0.0 and fy == 0.0)
    f = end_wall_force(0, 0, -1000.0, 0, 0, 0, 0, 0, -1.0, -60, 60)[:3]
    check(f"and never more than {DEFAULT_MAX_FORCE} N",
          _mag(f) <= DEFAULT_MAX_FORCE + 1e-9)


def test_joint_lock_torque():
    t = joint_lock_torque(0.05, -0.02, 0.3, 1.0, 1.0, 0.0, stiffness=1000.0, t_max=120.0)
    check("held joints spring back: +0.05 rad -> -50 mNm, -0.02 -> +20",
          abs(t[0] + 50.0) < 1e-9 and abs(t[1] - 20.0) < 1e-9)
    check("the free joint gets nothing, however far it moves", t[2] == 0.0)
    t = joint_lock_torque(1.0, 0, 0, 1.0, 0, 0, stiffness=1000.0, t_max=120.0)
    check("capped at 120 mNm", abs(t[0] + 120.0) < 1e-9)
    t = joint_lock_torque(0.05, 0, 0, 1.0, 0, 0, stiffness=1000.0, sign=-1.0)
    check("sign flips the convention", abs(t[0] - 50.0) < 1e-9)


def test_rate_limit_vec():
    f = rate_limit_vec(3.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.05)
    check("a big change is capped in magnitude", abs(_mag(f) - 0.05) < 1e-12)
    check("along the direction of the change", abs(f[0] / f[1] - 0.75) < 1e-12)
    check("a small change passes through",
          rate_limit_vec(0.01, 0, 0, 0, 0, 0, 0.05) == (0.01, 0, 0))


if __name__ == "__main__":
    print("\n  wall self-test\n")
    for fn in [test_free_inside, test_pushes_back_at_each_end,
               test_grows_with_penetration, test_clamped_to_max,
               test_damping_opposes_entry, test_damping_never_pulls_you_in,
               test_asymmetric_walls, test_rate_limit,
               test_rate_limit_reaches_full_scale_quickly,
               test_lock_pulls_back_to_the_anchor,
               test_lock_capped_in_magnitude_not_per_axis,
               test_end_walls_free_inside, test_end_walls_free_sideways,
               test_end_wall_pushes_back_along_the_axis,
               test_joint_lock_torque, test_rate_limit_vec]:
        print(f"\n  {fn.__name__}")
        fn()
    print("\n  all checks passed\n")
