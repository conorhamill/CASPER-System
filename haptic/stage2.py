"""
Stage 2 - the Touch drives the rig's TARGET POSE. Nothing moves.

Reads the stylus, runs the mapping, and streams the result into
USR_TGT_PSI/PHI/THETA_N/TOOL over Ethernet, ~115 Hz. The ApossIDE can stay
connected on USB throughout.

    python stage2.py                    stream, no force on the stylus
    python stage2.py --wall             also render the insertion wall
    python stage2.py --log run1.csv
    python stage2.py --dry-run          mapping only, nothing sent

>>> WHY NOTHING MOVES. <<<

The rig acts on a target only when USR_COMMAND asks it to. This program
NEVER writes USR_COMMAND - not once, anywhere - so the targets sit there
being overwritten and the motors are untouched. You can watch the pose
follow your hand in macs_monitor and the machine will not twitch.

The corollary is that this proves the mapping, the units, the fences and
the link, and NOT the motion. Making it move needs the Teleop state in the
.mc, with its own entry conditions, slew limiter and heartbeat watchdog.

Controls
    button 1   ROTATION clutch
    button 2   TRANSLATION clutch
    z          zero the commanded pose        q  quit
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

from mapping import MapConfig, PoseMap, euler_xyz_deg
from macs_tcp import MacsTcp, MacsTcpError, discover_ip
from stage0 import poll_key
from touch import BUTTON_1, BUTTON_2, Touch
import wall as wallmod

CONFIG_MH = Path(__file__).resolve().parent.parent / "main" / "3R1T_Config.mh"

# USER_PARAM indices, from 3R1T_Config.mh. Targets are contiguous, which is
# what lets them go out as one run of writes.
USR_TGT_PSI, USR_TGT_PHI, USR_TGT_THETA_N, USR_TGT_TOOL = 10, 11, 12, 13
USR_STATE, USR_MSG = 2, 4
USR_AX1_CDEG, USR_AX2_CDEG, USR_AX3_CDEG, USR_AXT_UU = 50, 51, 52, 53
USR_IK_SINGULAR = 54


def run(dev: Touch, m: MacsTcp | None, cfg: MapConfig, args):
    mapper = PoseMap(cfg)
    writer = handle = None
    if args.log:
        handle = open(args.log, "w", newline="")
        writer = csv.writer(handle)
        writer.writerow(["t_s", "rot_engaged", "trans_engaged",
                         "psi_deg", "phi_deg", "theta_n_deg", "tool_mm", "s_mm",
                         "clamped", "sent_ok", "send_ms",
                         "ax1_cdeg", "ax2_cdeg", "ax3_cdeg", "axt_uu", "ik_singular"])

    period = 1.0 / args.rate
    t0 = time.perf_counter()
    prev = 0
    n = sent = failed = 0
    status = [0, 0, 0, 0, 0]
    send_ms = 0.0
    rate_est = 0.0

    print(f"\n  button 1 = ROTATION   button 2 = TRANSLATION   z = zero   q = quit")
    print(f"  fences psi/phi +/-{cfg.psi_limit_deg:g} deg  theta_n +/-"
          f"{cfg.theta_n_limit_deg:g} deg  tool +/-{cfg.tool_limit_mm:g} mm"
          f"   (psi/phi/tool read from {CONFIG_MH.name})")
    print(f"  ratios {cfg.angle_ratio:g}:1 angle, {cfg.tool_ratio:g}:1 tool")
    print(f"  {'DRY RUN - nothing sent' if m is None else 'streaming to USR_TGT_* - USR_COMMAND is never written'}\n")

    try:
        while time.perf_counter() - t0 < args.seconds:
            loop = time.perf_counter()
            s = dev.snapshot() if args.wall else dev.read()
            t = loop - t0

            key = poll_key()
            if key == "q":
                break
            if key == "z" and not (mapper.rot_engaged or mapper.trans_engaged):
                mapper.pose_deg[:] = 0.0
                mapper.tool_mm = 0.0
                print(f"\n  [{t:6.2f}s] pose zeroed")

            rot, tra = bool(s.buttons & BUTTON_1), bool(s.buttons & BUTTON_2)
            rot_was, tra_was = bool(prev & BUTTON_1), bool(prev & BUTTON_2)
            prev = s.buttons

            if rot and not rot_was:
                mapper.engage_rotation(s)
                print(f"\n  [{t:6.2f}s] ROTATION engaged")
            elif rot_was and not rot:
                mapper.release_rotation()
                print(f"\n  [{t:6.2f}s] ROTATION released")
            if tra and not tra_was:
                mapper.engage_translation(s)
                lo, hi = mapper.travel_limits_mm()
                if args.wall and m is not None or args.wall:
                    dev.set_wall(mapper.wall_anchor_device,
                                 mapper.wall_axis_device, lo, hi)
                print(f"\n  [{t:6.2f}s] TRANSLATION engaged  wall s = {lo:+.0f}..{hi:+.0f} mm")
            elif tra_was and not tra:
                mapper.release_translation()
                if args.wall:
                    dev.clear_wall()
                print(f"\n  [{t:6.2f}s] TRANSLATION released  tool={mapper.tool_mm:+.2f} mm")

            cmd = mapper.update(s)
            n += 1

            # ---- send the four targets --------------------------------
            ok = True
            if m is not None:
                t_send = time.perf_counter()
                try:
                    # cdeg for the three angles, 0.01 mm for the tool - the
                    # units USR_TGT_* already use, so no conversion lives on
                    # the MACS side.
                    m.write_params(USR_TGT_PSI, [
                        round(cmd.psi_deg * 100),
                        round(cmd.phi_deg * 100),
                        round(cmd.theta_n_deg * 100),
                        round(cmd.tool_mm * 100)])
                    sent += 1
                except MacsTcpError as e:
                    ok = False
                    failed += 1
                    if failed <= 3:
                        print(f"\n  [{t:6.2f}s] send failed: {e}")
                send_ms = (time.perf_counter() - t_send) * 1000

                if n % args.status_every == 0:
                    try:
                        status = m.read_params(USR_AX1_CDEG, USR_IK_SINGULAR)
                    except MacsTcpError:
                        pass

            rate_est = n / max(t, 1e-6)

            def mark(k):
                return "*" if k in cmd.clamped else " "

            print(f"  psi {cmd.psi_deg:+6.2f}{mark('psi')}"
                  f"phi {cmd.phi_deg:+6.2f}{mark('phi')}"
                  f"thn {cmd.theta_n_deg:+7.2f}{mark('theta_n')}"
                  f"tool {cmd.tool_mm:+6.2f}{mark('tool')}| "
                  f"{'ROT' if cmd.rot_engaged else '   '} "
                  f"{'TRA' if cmd.trans_engaged else '   '} || "
                  f"IK ax {status[0]:+7d}{status[1]:+7d}{status[2]:+7d} "
                  f"tool {status[3]:+6d} sing {status[4]} | "
                  f"{send_ms:5.1f}ms {rate_est:5.1f}Hz", end="\r", flush=True)

            if writer:
                writer.writerow([f"{t:.4f}", int(cmd.rot_engaged), int(cmd.trans_engaged),
                                 f"{cmd.psi_deg:.4f}", f"{cmd.phi_deg:.4f}",
                                 f"{cmd.theta_n_deg:.4f}", f"{cmd.tool_mm:.4f}",
                                 f"{cmd.s_mm:.4f}", "|".join(cmd.clamped),
                                 int(ok), f"{send_ms:.3f}", *status])

            slack = period - (time.perf_counter() - loop)
            if slack > 0:
                time.sleep(slack)
    except KeyboardInterrupt:
        pass
    finally:
        if handle:
            handle.close()

    el = time.perf_counter() - t0
    print(f"\n\n  {n} cycles in {el:.1f} s  ({n/el:.1f} Hz)")
    if m is not None:
        print(f"  sent {sent}, failed {failed}")
    if args.log:
        print(f"  logged to {args.log}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--tcp", nargs="?", const="auto", default="auto", metavar="IP",
                   help="controller address; bare or omitted = discover")
    p.add_argument("--dry-run", action="store_true", help="do not send anything")
    p.add_argument("--wall", action="store_true",
                   help="also render the insertion wall (enables force output)")
    p.add_argument("--seconds", type=float, default=600.0)
    p.add_argument("--rate", type=float, default=110.0)
    p.add_argument("--status-every", type=int, default=10,
                   help="read the IK outputs every Nth cycle")
    p.add_argument("--log", default=None)
    p.add_argument("--angle-ratio", type=float, default=2.0)
    p.add_argument("--tool-ratio", type=float, default=3.0)
    args = p.parse_args()

    cfg = MapConfig.from_config_mh(CONFIG_MH,
                                   angle_ratio=args.angle_ratio,
                                   tool_ratio=args.tool_ratio)

    m = None
    if not args.dry_run:
        ip = discover_ip() if args.tcp == "auto" else args.tcp
        if not ip:
            print("\n  Nothing answered the Ethernet broadcast. Is X4 cabled?")
            return 1
        try:
            m = MacsTcp(ip).open()
        except MacsTcpError as e:
            print(f"\n  {e}")
            return 1
        print(f"\n  MACS: {m.info}")
        st, msg = m.read_param(USR_STATE), m.read_param(USR_MSG)
        print(f"  state=0x{st:X} msg={msg}")

    try:
        with Touch() as dev:
            info = dev.info
            print(f"  Touch: {info['model']} serial {info['serial']}")
            if not dev.is_live():
                print("\n  Touch is not returning a valid orientation - check its "
                      "power supply.")
                return 1
            if args.wall:
                dev.set_wall_gains(stiffness=wallmod.DEFAULT_STIFFNESS,
                                   damping=wallmod.DEFAULT_DAMPING,
                                   max_force=wallmod.DEFAULT_MAX_FORCE)
                dev.enable_force()
                dev.start_wall_loop()
                print("  force output ENABLED (wall on the insertion axis)")
            try:
                run(dev, m, cfg, args)
            finally:
                if args.wall:
                    dev.stop_wall_loop()
    finally:
        if m is not None:
            m.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
