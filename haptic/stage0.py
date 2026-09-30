"""
Stage 0 - the mapping on the bench, with no robot attached.

Reads the Touch, runs the full clutch-and-scale mapping, prints what it
would command, and logs every sample to CSV. Nothing is sent anywhere and
force output is never enabled, so the arm stays limp.

    python stage0.py                 run the mapping
    python stage0.py --axes          identify which stylus axis points out
    python stage0.py --seconds 60 --log run3.csv

Controls
    button 1 (front)   ROTATION clutch. Hold to aim.
    button 2 (rear)    TRANSLATION clutch. Hold to drive the tool.
                       Either, both or neither. Aim without inserting,
                       insert without re-aiming.
    z                  zero the commanded pose back to home
    q                  quit

What to look for
    - the "stylus" half of the line is live whether or not a clutch is held,
      so a still display never means a dead device
    - with ROT held, each wrist axis should move one robot axis
    - "s" is signed: push along the stylus and it grows, pull and it shrinks
    - a * marks an axis sitting on its fence
"""
from __future__ import annotations

import argparse
import csv
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

from mapping import MapConfig, PoseMap, euler_xyz_deg
from touch import BUTTON_1, BUTTON_2, CALIBRATION_NAMES, Touch

try:
    import msvcrt
except ImportError:                                  # not Windows
    msvcrt = None


def poll_key() -> str:
    if msvcrt and msvcrt.kbhit():
        try:
            return msvcrt.getch().decode(errors="ignore").lower()
        except Exception:
            return ""
    return ""


def show_axes(dev: Touch, seconds: float):
    """Print the stylus body axes in world coordinates.

    Hold the stylus pointing away from you and see which column tracks that
    direction. That column is the one to put in MapConfig.stylus_axis - with
    a minus sign if it points back at you.
    """
    print("\n  Hold the stylus pointing away from you. The axis that follows")
    print("  the tip is the one for MapConfig.stylus_axis.\n")
    print(f"  {'+X body axis':>26} {'+Y body axis':>26} {'+Z body axis':>26}")
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < seconds:
        R = dev.read().rotation
        line = "  " + " ".join(
            f"({R[0, i]:+5.2f},{R[1, i]:+5.2f},{R[2, i]:+5.2f})".rjust(26)
            for i in range(3))
        print(line, end="\r", flush=True)
        if poll_key() == "q":
            break
        time.sleep(0.05)
    print()


def run(dev: Touch, cfg: MapConfig, seconds: float, log_path: str | None, rate_hz: float):
    mapper = PoseMap(cfg)
    writer = handle = None
    if log_path:
        handle = open(log_path, "w", newline="")
        writer = csv.writer(handle)
        writer.writerow(["t_s", "buttons", "rot_engaged", "trans_engaged",
                         "px_mm", "py_mm", "pz_mm",
                         "qx", "qy", "qz", "qw",
                         "psi_deg", "phi_deg", "theta_n_deg", "tool_mm",
                         "s_mm", "clamped", "servo_hz"])

    period = 1.0 / rate_hz
    t0 = time.perf_counter()
    prev = 0
    n = 0

    print("\n  button 1 = ROTATION clutch   button 2 = TRANSLATION clutch")
    print("  z = zero pose    q = quit")
    print(f"  ratios {cfg.angle_ratio:g}:1 angle, {cfg.tool_ratio:g}:1 tool   "
          f"fences psi/phi +/-{cfg.psi_limit_deg:g} theta_n +/-{cfg.theta_n_limit_deg:g} "
          f"tool +/-{cfg.tool_limit_mm:g} mm\n")

    try:
        while time.perf_counter() - t0 < seconds:
            loop_start = time.perf_counter()
            s = dev.read()
            t = loop_start - t0

            key = poll_key()
            if key == "q":
                break
            if key == "z" and not (mapper.rot_engaged or mapper.trans_engaged):
                mapper.pose_deg[:] = 0.0
                mapper.tool_mm = 0.0
                print(f"\n  [{t:6.2f}s] pose zeroed")

            rot_down   = bool(s.buttons & BUTTON_1)
            trans_down = bool(s.buttons & BUTTON_2)
            rot_was    = bool(prev & BUTTON_1)
            trans_was  = bool(prev & BUTTON_2)
            prev = s.buttons

            if rot_down and not rot_was:
                mapper.engage_rotation(s)
                print(f"\n  [{t:6.2f}s] ROTATION engaged    from psi={mapper.pose_deg[0]:+.2f} "
                      f"phi={mapper.pose_deg[1]:+.2f} thn={mapper.pose_deg[2]:+.2f}")
            elif rot_was and not rot_down:
                mapper.release_rotation()
                print(f"\n  [{t:6.2f}s] ROTATION released   held psi={mapper.pose_deg[0]:+.2f} "
                      f"phi={mapper.pose_deg[1]:+.2f} thn={mapper.pose_deg[2]:+.2f}")

            if trans_down and not trans_was:
                mapper.engage_translation(s)
                lo, hi = mapper.travel_limits_mm()
                u = mapper._u
                print(f"\n  [{t:6.2f}s] TRANSLATION engaged  anchor "
                      f"({s.position[0]:+.1f},{s.position[1]:+.1f},{s.position[2]:+.1f}) mm  "
                      f"along ({u[0]:+.2f},{u[1]:+.2f},{u[2]:+.2f})  "
                      f"wall at s = {lo:+.0f} .. {hi:+.0f} mm")
            elif trans_was and not trans_down:
                mapper.release_translation()
                print(f"\n  [{t:6.2f}s] TRANSLATION released tool={mapper.tool_mm:+.2f} mm")

            cmd = mapper.update(s)
            n += 1

            def mark(name: str) -> str:
                return "*" if name in cmd.clamped else " "

            # The raw stylus is shown whether or not a clutch is engaged.
            # With both released the commanded pose correctly sits still, and
            # a frozen display is indistinguishable from a dead device -
            # which is exactly the confusion this line exists to prevent.
            (sr, sp, sy), _ = euler_xyz_deg(s.rotation)
            print(f"  stylus {s.position[0]:+6.1f}{s.position[1]:+6.1f}{s.position[2]:+6.1f} "
                  f"rpy {sr:+6.1f}{sp:+6.1f}{sy:+6.1f} || "
                  f"psi {cmd.psi_deg:+6.2f}{mark('psi')}"
                  f"phi {cmd.phi_deg:+6.2f}{mark('phi')}"
                  f"thn {cmd.theta_n_deg:+7.2f}{mark('theta_n')}"
                  f"tool {cmd.tool_mm:+6.2f}{mark('tool')}| "
                  f"s {cmd.s_mm:+6.1f} | "
                  f"{'ROT' if cmd.rot_engaged else '   '} "
                  f"{'TRA' if cmd.trans_engaged else '   '}"
                  f"{' LOCK' if cmd.gimbal_lock else '     '} | "
                  f"{s.rate_hz:4.0f}Hz", end="\r", flush=True)

            if writer:
                q = Rotation.from_matrix(s.rotation).as_quat()
                writer.writerow([f"{t:.4f}", s.buttons,
                                 int(cmd.rot_engaged), int(cmd.trans_engaged),
                                 f"{s.position[0]:.4f}", f"{s.position[1]:.4f}",
                                 f"{s.position[2]:.4f}",
                                 f"{q[0]:.6f}", f"{q[1]:.6f}", f"{q[2]:.6f}", f"{q[3]:.6f}",
                                 f"{cmd.psi_deg:.4f}", f"{cmd.phi_deg:.4f}",
                                 f"{cmd.theta_n_deg:.4f}", f"{cmd.tool_mm:.4f}",
                                 f"{cmd.s_mm:.4f}", "|".join(cmd.clamped),
                                 f"{s.rate_hz:.0f}"])

            slack = period - (time.perf_counter() - loop_start)
            if slack > 0:
                time.sleep(slack)
    except KeyboardInterrupt:
        pass
    finally:
        if handle:
            handle.close()

    elapsed = time.perf_counter() - t0
    print(f"\n\n  {n} samples in {elapsed:.1f} s ({n / elapsed:.0f} Hz outer loop)")
    if log_path:
        print(f"  logged to {log_path}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--axes", action="store_true",
                   help="print stylus body axes instead of running the mapping")
    p.add_argument("--seconds", type=float, default=300.0)
    p.add_argument("--rate", type=float, default=100.0, help="outer loop rate [Hz]")
    p.add_argument("--log", default=None, help="CSV path")
    p.add_argument("--angle-ratio", type=float, default=2.0)
    p.add_argument("--tool-ratio", type=float, default=3.0)
    args = p.parse_args()

    with Touch() as dev:
        info = dev.info
        ws = info["workspace"]
        print(f"\n  {info['model']}  driver {info['driver']}  serial {info['serial']}")
        print(f"  workspace x[{ws['x'][0]:.0f},{ws['x'][1]:.0f}] "
              f"y[{ws['y'][0]:.0f},{ws['y'][1]:.0f}] "
              f"z[{ws['z'][0]:.0f},{ws['z'][1]:.0f}] mm   "
              f"max force {info['max_force_N']:.2f} N")

        print("  checking the device is producing data ...", end=" ", flush=True)
        if not dev.is_live():
            print("NO")
            print("\n  The device is not returning a valid orientation - every value")
            print("  reads zero. The servo loop is running but nothing is arriving")
            print("  from the hardware. Check the Touch's power supply (separate")
            print("  from USB), then run Touch_Diagnostic.exe to confirm.")
            return 1
        print("yes")

        cal = dev.read().calibration
        if CALIBRATION_NAMES.get(cal) != "OK":
            print(f"  calibration is {CALIBRATION_NAMES.get(cal, cal)} - "
                  f"dock the stylus in the inkwell")

        if args.axes:
            show_axes(dev, args.seconds)
            return 0

        cfg = MapConfig(angle_ratio=args.angle_ratio, tool_ratio=args.tool_ratio)
        run(dev, cfg, args.seconds, args.log, args.rate)
    return 0


if __name__ == "__main__":
    sys.exit(main())
