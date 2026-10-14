"""
Lock probe - which Touch joints move for each pen movement, with a lock ON.

    python lock_probe.py                    JOINT lock: arm joints 1 and 2 held,
                                            joint 3 free (lock_probe_joint.csv)
    python lock_probe.py --mode position    the old pen-position lock
                                            (lock_probe.csv)

The haptic device only: nothing is sent to the controller and the robot is
not involved. Force output IS enabled - the lock is the thing being tested -
so the arm will push back while the rotation clutch is held.

The joint mode exists because this Touch's wrist pitch sensor reads nothing
(2026-10-14): up/down has to come from arm joint 3 instead, so joints 1 and
2 are held and joint 3 is left free to swing.

The run is timed and tells you when to change movement:

     0 -  5 s   get ready: hold the ROTATION clutch (front button)
     5 - 15 s   TWIST the pen back and forth          (theta_n)
    15 - 25 s   tip the pen LEFT and RIGHT            (phi)
    25 - 35 s   tip the pen UP and DOWN               (psi)

The lock goes on when the rotation clutch goes down and off when it comes
up, exactly as in the GUI. Every sample logs the three arm joints, the three
wrist (gimbal) joints, the reported position, how far it has been pulled
off the lock's anchor, the lock force or joint torques, and how the mapping
split the turn.
The screen shows only the instructions and whether each part recorded.

    q   stop early
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from mapping import MapConfig, PoseMap, split_turn
from touch import BUTTON_1, Touch

try:
    import msvcrt
except ImportError:                                  # not Windows
    msvcrt = None

HERE = Path(__file__).resolve().parent

PHASES = (                # (name, seconds, what to do)
    ("ready",      5.0, "HOLD THE ROTATION CLUTCH (front button) and keep it held"),
    ("twist",     10.0, "TWIST the pen back and forth"),
    ("left_right", 10.0, "tip the pen LEFT and RIGHT"),
    ("up_down",   10.0, "move the pen UP and DOWN"),
)


def poll_key() -> str:
    if msvcrt and msvcrt.kbhit():
        try:
            return msvcrt.getch().decode(errors="ignore").lower()
        except Exception:
            return ""
    return ""


def phase_at(t: float):
    end = 0.0
    for name, length, what in PHASES:
        end += length
        if t < end:
            return name, what, length
    return None, None, 0.0


def say_recorded(name: str, samples: int, held: int):
    if name == "ready" or samples == 0:
        return
    pct = 100.0 * held / samples
    if pct >= 80.0:
        print(f"  OK - {name} recorded ({pct:.0f}% with the clutch held)")
    else:
        print(f"  !! {name} recorded, but the clutch was only held {pct:.0f}% "
              f"of the time - that part may need doing again")


def run(dev: Touch, log_path: str, rate_hz: float, mode: str):
    mapper = PoseMap(MapConfig.from_config_mh(HERE.parent / "main" / "3R1T_Config.mh"))

    dev.lock_enabled = True
    dev.enable_force()
    dev.start_wall_loop()
    time.sleep(0.1)          # let the servo loop fill the buffers snapshot() reads

    handle = open(log_path, "w", newline="")
    writer = csv.writer(handle)
    writer.writerow(["t_s", "phase", "rot_clutch",
                     "px_mm", "py_mm", "pz_mm",
                     "anchor_x", "anchor_y", "anchor_z", "off_mm",
                     "fx_n", "fy_n", "fz_n", "force_n",
                     "tau1_mnm", "tau2_mnm", "tau3_mnm",
                     "arm1_anchor_deg", "arm2_anchor_deg", "arm3_anchor_deg",
                     "arm3_change_deg",
                     "arm1_deg", "arm2_deg", "arm3_deg",
                     "wrist1_deg", "wrist2_deg", "wrist3_deg",
                     "qx", "qy", "qz", "qw",
                     "up_deg", "right_deg", "twist_deg",
                     "psi_deg", "phi_deg", "theta_n_deg"])

    period = 1.0 / rate_hz
    prev_rot = False
    anchor = np.zeros(3)
    j_anchor = np.zeros(3)
    tripped_said = False
    R_prev, right = None, None
    shown, samples, held, total = None, 0, 0, 0
    t0 = time.perf_counter()

    try:
        while True:
            loop = time.perf_counter()
            t = loop - t0
            phase, what, length = phase_at(t)
            if poll_key() == "q":
                print("\n  stopped early")
                break
            if phase != shown:
                say_recorded(shown, samples, held)
                if phase is None:
                    break
                print("\a")                               # beep
                print(f"  >>> {what}   ({length:.0f} s)")
                shown, samples, held = phase, 0, 0

            s = dev.snapshot()
            rot = bool(s.buttons & BUTTON_1)
            if rot and not prev_rot:
                anchor = np.array(s.position, dtype=float)
                j_anchor = np.array(s.joints, dtype=float)
                if mode == "joint":
                    dev.set_joint_lock(j_anchor, (True, True, False))
                else:
                    dev.set_lock(anchor)
                mapper.engage_rotation(s)
                R_prev, right = np.array(s.rotation), None
                print("      clutch down - lock ON")
            elif prev_rot and not rot:
                dev.clear_wall()
                mapper.release_rotation()
                print("      clutch up - lock off")
            prev_rot = rot
            if dev.joint_lock_tripped and not tripped_said:
                tripped_said = True
                print("  !! the joint lock let go - a held joint was pulled more than")
                print("     20 deg off. Carry on; the log will show where.")

            up = rt = tw = 0.0
            if rot and R_prev is not None:
                up, rt, tw, right = split_turn(R_prev, s.rotation, right,
                                               mapper.cfg.stylus_axis)
                R_prev = np.array(s.rotation)
            cmd = mapper.update(s)

            off = float(np.linalg.norm(s.position - anchor)) if rot else 0.0
            q = Rotation.from_matrix(s.rotation).as_quat()
            writer.writerow([f"{t:.4f}", phase, int(rot),
                             *(f"{v:.3f}" for v in s.position),
                             *(f"{v:.3f}" for v in anchor), f"{off:.3f}",
                             f"{dev._fx:.4f}", f"{dev._fy:.4f}", f"{dev._fz:.4f}",
                             f"{dev.force_n:.4f}",
                             *(f"{v:.2f}" for v in dev.joint_torque_mnm),
                             *(f"{v:.3f}" for v in np.degrees(j_anchor)),
                             f"{np.degrees(s.joints[2] - j_anchor[2]) if rot else 0.0:.3f}",
                             *(f"{v:.3f}" for v in np.degrees(s.joints)),
                             *(f"{v:.3f}" for v in np.degrees(s.gimbal)),
                             *(f"{v:.6f}" for v in q),
                             f"{up:.4f}", f"{rt:.4f}", f"{tw:.4f}",
                             f"{cmd.psi_deg:.3f}", f"{cmd.phi_deg:.3f}",
                             f"{cmd.theta_n_deg:.3f}"])
            samples += 1
            held += int(rot)
            total += 1

            slack = period - (time.perf_counter() - loop)
            if slack > 0:
                time.sleep(slack)
    except KeyboardInterrupt:
        print("\n  stopped early")
    finally:
        dev.clear_wall()
        dev.stop_wall_loop()           # winds the force down before letting go
        handle.close()

    print("\a")
    print(f"  DONE - you can let go. {total} samples saved to {log_path}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--mode", choices=("joint", "position"), default="joint",
                   help="joint: hold arm joints 1 and 2, free 3 (default); "
                        "position: hold the pen's position")
    p.add_argument("--log", default=None,
                   help="CSV path (default: lock_probe_joint.csv or "
                        "lock_probe.csv next to this script)")
    p.add_argument("--rate", type=float, default=100.0, help="loop rate [Hz]")
    args = p.parse_args()
    log = args.log or str(HERE / ("lock_probe_joint.csv" if args.mode == "joint"
                                  else "lock_probe.csv"))

    with Touch() as dev:
        if not dev.is_live():
            print("\n  The Touch is not returning a valid orientation - check its")
            print("  power supply (separate from USB).")
            return 1
        what = ("arm joints 1 and 2 held, joint 3 free" if args.mode == "joint"
                else "pen position held")
        print(f"\n  Lock probe ({what}) - Touch connected.")
        print("  The arm will push back while the rotation clutch is held;")
        print("  that is the lock. q stops early.")
        run(dev, log, args.rate, args.mode)
    return 0


if __name__ == "__main__":
    sys.exit(main())
