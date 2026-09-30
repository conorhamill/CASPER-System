"""
Self-test for the virtual wall. Needs no hardware.

    python test_wall.py
"""
from __future__ import annotations

from wall import DEFAULT_MAX_FORCE, rate_limit, wall_force


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


if __name__ == "__main__":
    print("\n  wall self-test\n")
    for fn in [test_free_inside, test_pushes_back_at_each_end,
               test_grows_with_penetration, test_clamped_to_max,
               test_damping_opposes_entry, test_damping_never_pulls_you_in,
               test_asymmetric_walls, test_rate_limit,
               test_rate_limit_reaches_full_scale_quickly]:
        print(f"\n  {fn.__name__}")
        fn()
    print("\n  all checks passed\n")
