"""
Stage 1 - the mapping, plus a real virtual wall at the ends of tool travel.

Same mapping as stage0, but force output is ENABLED. The arm is no longer
limp: hold the stylus before starting. Still nothing is sent to the robot.

    python stage1.py
    python stage1.py --stiffness 0.5 --log wall1.csv
    python stage1.py --stiffness 0.1 --damping 0            compare feels

Controls
    button 1 (front)   ROTATION clutch. Hold to aim. No force on this -
                       the gimbal has no motors, so tilt and yaw can only
                       be clamped and announced, never walled.
    button 2 (rear)    TRANSLATION clutch. Hold to drive the tool. The wall
                       lives on this axis.
    z                  zero the commanded pose
    q                  quit (winds the force down first)

Tuning
    Start at the default 0.30 N/mm. Raise --stiffness until the wall buzzes
    or screeches, then back off by about a third. That buzz is the device
    and the 1 kHz loop hitting their stability limit, and it arrives well
    before the 3.3 N the motors can deliver - so a virtual wall is always a
    firm spring with a few mm of give, never a steel stop.
"""
from __future__ import annotations

import argparse
import csv
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

import wall as wallmod
from mapping import MapConfig, PoseMap, euler_xyz_deg
from stage0 import poll_key
from touch import BUTTON_1, BUTTON_2, CALIBRATION_NAMES, Touch


def run(dev: Touch, cfg: MapConfig, seconds: float, log_path: str | None, rate_hz: float):
    mapper = PoseMap(cfg)
    writer = handle = None
    if log_path:
        handle = open(log_path, "w", newline="")
        writer = csv.writer(handle)
        writer.writerow(["t_s", "buttons", "rot_engaged", "trans_engaged",
                         "px_mm", "py_mm", "pz_mm", "qx", "qy", "qz", "qw",
                         "psi_deg", "phi_deg", "theta_n_deg", "tool_mm",
                         "s_mm", "wall_lo_mm", "wall_hi_mm", "force_N",
                         "clamped", "servo_hz"])

    period = 1.0 / rate_hz
    t0 = time.perf_counter()
    prev = 0
    lo = hi = 0.0
    n = 0

    print("\n  button 1 = ROTATION (no force)   button 2 = TRANSLATION (walled)")
    print("  z = zero pose    q = quit")
    print(f"  wall {dev._w_k:g} N/mm, damping {dev._w_b:g} N.s/mm, "
          f"ceiling {dev._w_fmax:g} N\n")

    try:
        while time.perf_counter() - t0 < seconds:
            loop_start = time.perf_counter()
            # The wall loop is already refreshing the buffers at 1 kHz.
            s = dev.snapshot()
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
                print(f"\n  [{t:6.2f}s] ROTATION engaged")
            elif rot_was and not rot_down:
                mapper.release_rotation()
                print(f"\n  [{t:6.2f}s] ROTATION released   psi={mapper.pose_deg[0]:+.2f} "
                      f"phi={mapper.pose_deg[1]:+.2f} thn={mapper.pose_deg[2]:+.2f}")

            if trans_down and not trans_was:
                mapper.engage_translation(s)
                lo, hi = mapper.travel_limits_mm()
                # s = 0 at engage and the tool is always inside its fence, so
                # lo <= 0 <= hi: the wall can never be placed behind the hand.
                dev.set_wall(mapper.wall_anchor_device, mapper.wall_axis_device, lo, hi)
                u = mapper.wall_axis_device
                print(f"\n  [{t:6.2f}s] TRANSLATION engaged  along "
                      f"({u[0]:+.2f},{u[1]:+.2f},{u[2]:+.2f})  "
                      f"wall at s = {lo:+.0f} .. {hi:+.0f} mm")
            elif trans_was and not trans_down:
                mapper.release_translation()
                dev.clear_wall()
                print(f"\n  [{t:6.2f}s] TRANSLATION released tool={mapper.tool_mm:+.2f} mm")

            cmd = mapper.update(s)
            force = dev.wall_force_n
            n += 1

            def mark(name: str) -> str:
                return "*" if name in cmd.clamped else " "

            bar = ""
            if abs(force) > 0.01:
                blocks = min(10, int(abs(force) / dev._w_fmax * 10) + 1)
                bar = ("<" if force > 0 else ">") * blocks
            (sr, sp, sy), _ = euler_xyz_deg(s.rotation)
            print(f"  stylus {s.position[0]:+6.1f}{s.position[1]:+6.1f}{s.position[2]:+6.1f} "
                  f"rpy {sr:+6.1f}{sp:+6.1f}{sy:+6.1f} || "
                  f"psi {cmd.psi_deg:+6.2f}{mark('psi')}"
                  f"phi {cmd.phi_deg:+6.2f}{mark('phi')}"
                  f"thn {cmd.theta_n_deg:+7.2f}{mark('theta_n')}"
                  f"tool {cmd.tool_mm:+6.2f}{mark('tool')}| "
                  f"s {cmd.s_mm:+6.1f} | "
                  f"{force:+5.2f}N {bar:<11}| "
                  f"{'ROT' if cmd.rot_engaged else '   '} "
                  f"{'TRA' if cmd.trans_engaged else '   '} | "
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
                                 f"{cmd.s_mm:.4f}", f"{lo:.2f}", f"{hi:.2f}",
                                 f"{force:.4f}", "|".join(cmd.clamped),
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
    p.add_argument("--seconds", type=float, default=300.0)
    p.add_argument("--rate", type=float, default=100.0)
    p.add_argument("--log", default=None)
    p.add_argument("--angle-ratio", type=float, default=2.0)
    p.add_argument("--tool-ratio", type=float, default=3.0)
    p.add_argument("--stiffness", type=float, default=wallmod.DEFAULT_STIFFNESS,
                   help="N/mm; raise until it buzzes, then back off a third")
    p.add_argument("--damping", type=float, default=wallmod.DEFAULT_DAMPING,
                   help="N.s/mm")
    p.add_argument("--max-force", type=float, default=wallmod.DEFAULT_MAX_FORCE,
                   help="N; the device nominal is 3.3")
    p.add_argument("--yes", action="store_true", help="skip the confirmation")
    args = p.parse_args()

    with Touch() as dev:
        info = dev.info
        print(f"\n  {info['model']}  driver {info['driver']}  serial {info['serial']}")
        print(f"  device limits: peak {info['max_force_N']:.2f} N, "
              f"continuous {info['max_cont_force_N']:.2f} N, "
              f"stiffness {info['max_stiffness']:.2f} N/mm, "
              f"damping {info['max_damping']:.3f} N.s/mm")

        print("  checking the device is producing data ...", end=" ", flush=True)
        if not dev.is_live():
            print("NO")
            print("\n  The device is not returning a valid orientation. Check the")
            print("  Touch's power supply, then run Touch_Diagnostic.exe.")
            return 1
        print("yes")

        cal = dev.read().calibration
        if CALIBRATION_NAMES.get(cal) != "OK":
            print(f"  calibration is {CALIBRATION_NAMES.get(cal, cal)} - "
                  f"dock the stylus in the inkwell")

        if args.stiffness > info["max_stiffness"]:
            print(f"\n  ! stiffness {args.stiffness:g} N/mm is above the device's stated")
            print(f"    maximum of {info['max_stiffness']:.2f} N/mm. Expect buzzing, not a")
            print(f"    harder wall.")
        if args.damping > info["max_damping"]:
            print(f"\n  ! damping {args.damping:g} is above the device's stated maximum "
                  f"of {info['max_damping']:.3f}")

        print("\n  >>> FORCE OUTPUT IS ABOUT TO BE ENABLED. <<<")
        print(f"      The arm will stop being limp. Ceiling {args.max_force:.1f} N.")
        print("      Take hold of the stylus before continuing.")
        if not args.yes:
            try:
                input("\n      Enter to arm, Ctrl-C to abort: ")
            except (KeyboardInterrupt, EOFError):
                print("\n  aborted - force never enabled")
                return 0

        dev.set_wall_gains(stiffness=args.stiffness, damping=args.damping,
                           max_force=args.max_force)
        dev.enable_force(args.max_force)
        dev.start_wall_loop()
        print("  force enabled, wall loop running")

        cfg = MapConfig(angle_ratio=args.angle_ratio, tool_ratio=args.tool_ratio)
        try:
            run(dev, cfg, args.seconds, args.log, args.rate)
        finally:
            dev.stop_wall_loop()
            print("  force wound down")
    return 0


if __name__ == "__main__":
    sys.exit(main())
