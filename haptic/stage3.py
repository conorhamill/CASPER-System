"""
Stage 3 - teleoperation. THE ROBOT CAN MOVE.

Everything stage2 does, plus the three things that hand control over:

    heartbeat   slot 14, incremented every cycle. The rig drops out of
                following if it stops for C_TELE_STALE_MS.
    enable      slot 15, set while either clutch is held. Let go of both
                and the rig stops following where it is.
    command     C_CMD_TELEOP (0x40) to enter the Teleop state, again to
                leave.

    python stage3.py --dry-run     stage2 behaviour, nothing sent
    python stage3.py               arm it
    python stage3.py --wall --log teleop1.csv

Controls
    t          enter/leave the rig's Teleop state
    button 1   ROTATION clutch   - also holds the enable
    button 2   TRANSLATION clutch - also holds the enable
    s          STOP (leaves Teleop)
    z          zero the PC's commanded pose (only while not following)
    q          quit - sends STOP and clears the enable on the way out

>>> WHAT ACTUALLY STOPS THIS. <<<

Releasing both clutches stops the rig following, and quitting sends a STOP.
Neither is an emergency stop: they are a button and a keypress on the far
side of a Python program and an Ethernet link. The rig's own watchdog
covers this program dying. The hardware E-stop covers everything else, and
it should be within reach before you run this.
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import wall as wallmod
from macs_tcp import MacsTcp, MacsTcpError, discover_ip
from mapping import MapConfig, PoseMap
from stage0 import poll_key
from touch import BUTTON_1, BUTTON_2, Touch

CONFIG_MH = Path(__file__).resolve().parent.parent / "main" / "3R1T_Config.mh"

# Slots, from 3R1T_Config.mh. Parsed rather than trusted where it matters.
USR_COMMAND = 1
USR_STATE, USR_MSG = 2, 4
USR_TGT_PSI = 10
USR_TELE_HEARTBEAT, USR_TELE_ENABLE = 14, 15
USR_TELE_STATE, USR_TELE_HB_AGE = 74, 76
USR_TELE_MAX_AGE, USR_TELE_TRIPS, USR_TELE_CLAMPED = 77, 78, 79
USR_AX1_CDEG = 50

C_CMD_STOP, C_CMD_TELEOP = 0x02, 0x40
C_STA_TELEOP = 0x100
TELE_NAMES = {0: "OFF", 1: "WAIT_ENABLE", 2: "FOLLOW", 3: "DROPPED"}


def run(dev: Touch, m: MacsTcp | None, cfg: MapConfig, args):
    mapper = PoseMap(cfg)
    writer = handle = None
    if args.log:
        handle = open(args.log, "w", newline="")
        writer = csv.writer(handle)
        writer.writerow(["t_s", "rot", "tra", "enable", "hb",
                         "psi_deg", "phi_deg", "theta_n_deg", "tool_mm",
                         "clamped_pc", "send_ms",
                         "tele_state", "hb_age_ms", "clamped_rig", "trips"])

    period = 1.0 / args.rate
    t0 = time.perf_counter()
    prev_buttons = 0
    prev_enable = None
    hb = 0
    n = failed = 0
    status = [0, 0, 0, 0]
    send_ms = 0.0

    print("\n  t = enter/leave Teleop   s = STOP   z = zero   q = quit")
    print("  hold either clutch to hold the ENABLE\n")

    try:
        while time.perf_counter() - t0 < args.seconds:
            loop = time.perf_counter()
            s = dev.snapshot() if args.wall else dev.read()
            t = loop - t0

            key = poll_key()
            if key == "q":
                break
            if key == "t" and m is not None:
                m.write_param(USR_COMMAND, C_CMD_TELEOP)
                print(f"\n  [{t:6.2f}s] -> C_CMD_TELEOP")
            if key == "s" and m is not None:
                m.write_param(USR_COMMAND, C_CMD_STOP)
                print(f"\n  [{t:6.2f}s] -> C_CMD_STOP")
            if key == "z" and not (mapper.rot_engaged or mapper.trans_engaged):
                mapper.pose_deg[:] = 0.0
                mapper.tool_mm = 0.0
                print(f"\n  [{t:6.2f}s] PC pose zeroed")

            rot, tra = bool(s.buttons & BUTTON_1), bool(s.buttons & BUTTON_2)
            rot_was, tra_was = bool(prev_buttons & BUTTON_1), bool(prev_buttons & BUTTON_2)
            prev_buttons = s.buttons

            if rot and not rot_was:
                mapper.engage_rotation(s)
            elif rot_was and not rot:
                mapper.release_rotation()
            if tra and not tra_was:
                mapper.engage_translation(s)
                lo, hi = mapper.travel_limits_mm()
                if args.wall:
                    dev.set_wall(mapper.wall_anchor_device,
                                 mapper.wall_axis_device, lo, hi)
            elif tra_was and not tra:
                mapper.release_translation()
                if args.wall:
                    dev.clear_wall()

            cmd = mapper.update(s)
            n += 1

            # The enable is "a clutch is held". If the operator is not
            # holding one, they are not asking the rig to move, so there is
            # no separate button to forget about.
            enable = 1 if (rot or tra) else 0

            if m is not None:
                t_send = time.perf_counter()
                try:
                    m.write_params(USR_TGT_PSI, [
                        round(cmd.psi_deg * 100), round(cmd.phi_deg * 100),
                        round(cmd.theta_n_deg * 100), round(cmd.tool_mm * 100)])
                    # Every cycle, unconditionally. This is the only thing
                    # telling the rig this program is still alive.
                    hb = (hb + 1) & 0x7FFFFFFF
                    m.write_param(USR_TELE_HEARTBEAT, hb)
                    # Only on change - it costs a round trip and rarely moves.
                    if enable != prev_enable:
                        m.write_param(USR_TELE_ENABLE, enable)
                        prev_enable = enable
                except MacsTcpError as e:
                    failed += 1
                    if failed <= 3:
                        print(f"\n  [{t:6.2f}s] send failed: {e}")
                send_ms = (time.perf_counter() - t_send) * 1000

                if n % args.status_every == 0:
                    try:
                        status = [m.read_param(USR_TELE_STATE),
                                  m.read_param(USR_TELE_HB_AGE),
                                  m.read_param(USR_TELE_CLAMPED),
                                  m.read_param(USR_TELE_TRIPS)]
                    except MacsTcpError:
                        pass

            def mark(k):
                return "*" if k in cmd.clamped else " "

            rig = TELE_NAMES.get(status[0], str(status[0]))
            print(f"  psi {cmd.psi_deg:+6.2f}{mark('psi')}"
                  f"phi {cmd.phi_deg:+6.2f}{mark('phi')}"
                  f"thn {cmd.theta_n_deg:+7.2f}{mark('theta_n')}"
                  f"tool {cmd.tool_mm:+6.2f}{mark('tool')}| "
                  f"en {enable} | rig {rig:<11} age {status[1]:4d}ms "
                  f"clamp {status[2]:X} trips {status[3]} | "
                  f"{send_ms:5.1f}ms {n/max(t,1e-6):5.1f}Hz",
                  end="\r", flush=True)

            if writer:
                writer.writerow([f"{t:.4f}", int(rot), int(tra), enable, hb,
                                 f"{cmd.psi_deg:.4f}", f"{cmd.phi_deg:.4f}",
                                 f"{cmd.theta_n_deg:.4f}", f"{cmd.tool_mm:.4f}",
                                 "|".join(cmd.clamped), f"{send_ms:.3f}", *status])

            slack = period - (time.perf_counter() - loop)
            if slack > 0:
                time.sleep(slack)
    except KeyboardInterrupt:
        pass
    finally:
        if handle:
            handle.close()
        # Clear the enable and ask for a stop, in that order. Even if the
        # stop is refused or lost, a cleared enable stops the rig following.
        if m is not None:
            for fn in (lambda: m.write_param(USR_TELE_ENABLE, 0),
                       lambda: m.write_param(USR_COMMAND, C_CMD_STOP)):
                try:
                    fn()
                except MacsTcpError:
                    pass
            print("\n  enable cleared, STOP sent")

    el = time.perf_counter() - t0
    print(f"\n  {n} cycles in {el:.1f} s ({n/el:.1f} Hz), {failed} send failures")
    if args.log:
        print(f"  logged to {args.log}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--tcp", nargs="?", const="auto", default="auto", metavar="IP")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--wall", action="store_true")
    p.add_argument("--seconds", type=float, default=600.0)
    p.add_argument("--rate", type=float, default=100.0)
    p.add_argument("--status-every", type=int, default=15)
    p.add_argument("--log", default=None)
    p.add_argument("--angle-ratio", type=float, default=2.0)
    p.add_argument("--tool-ratio", type=float, default=3.0)
    p.add_argument("--yes", action="store_true", help="skip the confirmation")
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
        st = m.read_param(USR_STATE)
        print(f"\n  MACS {m.info}")
        print(f"  state=0x{st:X} msg={m.read_param(USR_MSG)} "
              f"tele={TELE_NAMES.get(m.read_param(USR_TELE_STATE), '?')}")
        print(f"  fences psi/phi +/-{cfg.psi_limit_deg:g} deg  "
              f"tool +/-{cfg.tool_limit_mm:g} mm  (from {CONFIG_MH.name})")

        print("\n  >>> THIS CAN MOVE THE ROBOT. <<<")
        print("      Hardware E-stop within reach. Nobody near the platform.")
        print("      Press t to enter Teleop, then hold a clutch to drive.")
        if not args.yes:
            try:
                input("\n      Enter to continue, Ctrl-C to abort: ")
            except (KeyboardInterrupt, EOFError):
                print("\n  aborted - nothing sent")
                m.close()
                return 0

    try:
        with Touch() as dev:
            print(f"  Touch: {dev.info['model']} serial {dev.info['serial']}")
            if not dev.is_live():
                print("\n  Touch is not returning a valid orientation.")
                return 1
            if args.wall:
                dev.set_wall_gains(stiffness=wallmod.DEFAULT_STIFFNESS,
                                   damping=wallmod.DEFAULT_DAMPING,
                                   max_force=wallmod.DEFAULT_MAX_FORCE)
                dev.enable_force()
                dev.start_wall_loop()
                print("  force output ENABLED")
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
