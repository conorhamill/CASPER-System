"""
Live view of the rig's USER_PARAM block, read over USB. READ ONLY.

    python macs_monitor.py
    python macs_monitor.py --all          every parameter, not just the
                                          interesting ones
    python macs_monitor.py --bench        time the link and exit

The parameter names come from main/3R1T_Config.mh at run time rather than
being copied here, so this cannot drift out of step with the .mc.

Reading the whole block costs about 250 ms because UserParamReadRaw is one
round trip per parameter - fine for a 1 Hz monitor, useless for a control
loop. See the note in macs.py.
"""
from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path

from macs import Macs, MacsError
from macs_tcp import MacsTcp, MacsTcpError, discover_ip

CONFIG = Path(__file__).resolve().parent.parent / "main" / "3R1T_Config.mh"

# Groups worth watching, by USR_ name. Anything not listed still shows under
# --all.
GROUPS = [
    ("command",  ["USR_COMMAND", "USR_STATE", "USR_LED", "USR_MSG"]),
    ("targets",  ["USR_TGT_PSI", "USR_TGT_PHI", "USR_TGT_THETA_N", "USR_TGT_TOOL"]),
    ("commanded axes", ["USR_AX1_CDEG", "USR_AX2_CDEG", "USR_AX3_CDEG", "USR_AXT_UU",
                        "USR_IK_SINGULAR", "USR_IK_LINK_SURPASS", "USR_ISR_DURATION"]),
    ("limb IMUs", ["USR_THETA1", "USR_THETA2", "USR_THETA3",
                   "USR_IMU_STATUS", "USR_THETA_STALE"]),
    ("platform",  ["USR_PLAT_X", "USR_PLAT_Y", "USR_PLAT_Z",
                   "USR_PLAT_STATUS", "USR_PLAT_STALE"]),
    ("stage",     ["USR_ENC_UM", "USR_ENC_VEL", "USR_ENC_STATUS", "USR_ENC_STALE"]),
    ("homing",    ["USR_HOME_STATE", "USR_HOME_PASS",
                   "USR_MEAS1", "USR_MEAS2", "USR_MEAS3"]),
    ("faults",    ["USR_ERROR_NO", "USR_FAULT_COUNT", "USR_FAULT_AXIS",
                   "USR_FAULT_NODE", "USR_FAULT_EPOS", "USR_FAULT_STATE"]),
]


def load_names(path: Path) -> dict[int, str]:
    """USR_* name for each parameter number, parsed from the .mc config."""
    if not path.exists():
        print(f"  ! {path} not found - falling back to bare numbers")
        return {}
    names = {}
    for m in re.finditer(r"^#define\s+(USR_\w+)\s+(\d+)", path.read_text(errors="replace"),
                         re.MULTILINE):
        names[int(m.group(2))] = m.group(1)
    return names


def bench(m: Macs):
    def timed(label, fn, n=20):
        for _ in range(3):
            fn()
        t0 = time.perf_counter()
        for _ in range(n):
            fn()
        dt = (time.perf_counter() - t0) / n
        print(f"  {label:<38} {dt * 1000:7.2f} ms  {1 / dt:7.1f} Hz")

    print()
    timed("read_param  x1", lambda: m.read_param(10))
    timed("read_params 4", lambda: m.read_params(10, 13))
    timed("read_params 28", lambda: m.read_params(10, 37), n=10)
    # ReadUserArray is ALL OR NOTHING: ask for fewer values than the array
    # holds and it returns -115 BADARRAYSIZE rather than a partial read. So
    # the whole array is the unit of transfer, which is the argument for
    # giving teleop its own small one rather than sharing a big one.
    for nr in range(6):
        size = m.array_size(nr)
        if not size:
            print(f"  array {nr}: does not exist")
            break
        print(f"  array {nr}: {size} values")
        timed(f"read_array({nr}) all {size} values",
              lambda nr=nr, size=size: m.read_array(nr, size), n=10)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--all", action="store_true", help="show every parameter")
    p.add_argument("--bench", action="store_true", help="time the link and exit")
    p.add_argument("--period", type=float, default=1.0, help="refresh [s]")
    p.add_argument("--tcp", nargs="?", const="auto", default=None,
                   metavar="IP",
                   help="use Ethernet instead of USB, leaving USB to the "
                        "ApossIDE. Bare --tcp discovers the address.")
    args = p.parse_args()

    names = load_names(CONFIG)
    by_name = {v: k for k, v in names.items()}

    try:
        if args.tcp:
            ip = discover_ip() if args.tcp == "auto" else args.tcp
            if not ip:
                print("\n  Nothing answered the Ethernet broadcast. Is X4 cabled?")
                return 1
            m = MacsTcp(ip)
        else:
            m = Macs()
        m.open()
    except (MacsError, MacsTcpError) as e:
        print(f"\n  {e}")
        if not args.tcp:
            print("\n  Is the MasterMACS powered and the USB cable in? The ApossIDE")
            print("  holds the USB port exclusively - disconnect it, or pass --tcp")
            print("  to go over Ethernet and leave USB to the IDE.")
        return 1

    try:
        print(f"\n  {m.version}")
        print(f"  {m.info}")
        print(f"  {len(names)} USR_ names from {CONFIG.name}")

        if args.bench:
            bench(m)
            return 0

        print("\n  Ctrl-C to stop.\n")
        try:
            while True:
                t0 = time.perf_counter()
                vals = m.read_params(0, m.info.user_params - 1)
                took = (time.perf_counter() - t0) * 1000

                lines = [f"  ---- {time.strftime('%H:%M:%S')}  "
                         f"(read {len(vals)} params in {took:.0f} ms) ----"]
                if args.all:
                    for n, v in enumerate(vals):
                        if v or names.get(n):
                            lines.append(f"    {n:3d} {names.get(n, ''):<22} {v}")
                else:
                    for title, keys in GROUPS:
                        cells = []
                        for k in keys:
                            n = by_name.get(k)
                            if n is None or n >= len(vals):
                                continue
                            cells.append(f"{k[4:].lower()}={vals[n]}")
                        if cells:
                            lines.append(f"    {title:<16} " + "  ".join(cells))
                print("\n".join(lines))
                time.sleep(max(0.0, args.period - (time.perf_counter() - t0)))
        except KeyboardInterrupt:
            print("\n  stopped")
    finally:
        m.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
