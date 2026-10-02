"""
Haptic teleoperation - the whole thing, with buttons.

    python gui.py

No arguments, no keystrokes. It finds the controller on Ethernet, opens the
Touch, and streams the pose. Press TELEOP to hand the rig over, then hold
either clutch on the stylus to drive.

>>> THE ENABLE IS STILL THE STYLUS, NOT A BUTTON HERE. <<<

A GUI button cannot be "held" while your hand is on the stylus, and an
enable you can walk away from is not an enable. So holding either clutch
holds the enable, exactly as before - the window only shows you whether
the rig agrees.

>>> AND NONE OF THESE BUTTONS IS AN EMERGENCY STOP. <<<

STOP asks the rig to leave Teleop over an Ethernet link, from a Python
program, through a window manager. The rig's own watchdog covers this
program dying. The hardware E-stop covers everything else.

HOW IT IS PUT TOGETHER

Tk wants the main thread and the pose loop wants a steady 100 Hz, so the
loop runs in a worker and the window polls a snapshot of it. Every byte of
MACS traffic happens on the worker - button presses are queued for it
rather than sent from the GUI thread - because MacsTcp holds one socket and
interleaving two threads on it would corrupt a frame eventually, which is
the sort of bug that shows up once a fortnight and never in testing.
"""
from __future__ import annotations

import queue
import re
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from pathlib import Path
from tkinter import ttk

import wall as wallmod
from macs_tcp import MacsTcp, MacsTcpError, discover_ip
from mapping import MapConfig, PoseMap, euler_xyz_deg
from touch import BUTTON_1, BUTTON_2, HDError, Touch

CONFIG_MH = Path(__file__).resolve().parent.parent / "main" / "3R1T_Config.mh"
RATE_HZ = 100.0
STATUS_EVERY = 15


# ---------------------------------------------------------------- the .mc
def load_defines(path: Path) -> dict[str, int]:
    """Every #define with a plain numeric value, so nothing is duplicated
    here that the .mc already states."""
    out = {}
    for m in re.finditer(r"^#define\s+(\w+)\s+(0x[0-9A-Fa-f]+|\d+)\s*(?://|$)",
                         path.read_text(errors="replace"), re.MULTILINE):
        out[m.group(1)] = int(m.group(2), 0)
    return out


D = load_defines(CONFIG_MH)


def slot(name: str) -> int:
    return D[name]


def bits(value: int, prefix: str) -> str:
    """Names of the set bits, for a C_STA_-style field."""
    on = [k[len(prefix):] for k, v in D.items()
          if k.startswith(prefix) and v and (value & v) == v]
    return " ".join(sorted(on)) or "-"


def enum_name(value: int, prefix: str) -> str:
    for k, v in D.items():
        if k.startswith(prefix) and v == value:
            return k[len(prefix):]
    return str(value)


MSG_TEXT = {v: k[4:].replace("_", " ").lower()
            for k, v in D.items() if k.startswith("MSG_")}


# ------------------------------------------------------------- shared state
@dataclass
class Shared:
    lock: threading.Lock = field(default_factory=threading.Lock)
    touch_ok: bool = False
    macs_ok: bool = False
    note: str = "starting"
    rate: float = 0.0
    send_ms: float = 0.0
    failures: int = 0
    rot: bool = False
    tra: bool = False
    enable: int = 0
    pose: tuple = (0.0, 0.0, 0.0, 0.0)
    clamped_pc: tuple = ()
    rig_state: int = 0
    rig_msg: int = 0
    tele_state: int = 0
    hb_age: int = 0
    max_age: int = 0
    trips: int = 0
    clamped_rig: int = 0
    tremor: int = 0
    # The RAW stylus, independent of any clutch. See the note where it
    # is displayed.
    stylus_pos: tuple = (0.0, 0.0, 0.0)
    stylus_rpy: tuple = (0.0, 0.0, 0.0)
    touch_id: str = ""
    ik: tuple = (0, 0, 0, 0)
    led: int = 0

    def snapshot(self) -> dict:
        with self.lock:
            return {k: v for k, v in self.__dict__.items() if k != "lock"}


class Worker(threading.Thread):
    """Owns the Touch and the socket. Nothing else touches either."""

    def __init__(self, shared: Shared, cfg: MapConfig):
        super().__init__(daemon=True)
        self.shared = shared
        self.cfg = cfg
        self.commands: queue.Queue = queue.Queue()
        self.stop_flag = threading.Event()
        self.mapper = PoseMap(cfg)
        self.wall_on = False
        self._dev: Touch | None = None
        self._macs: MacsTcp | None = None
        # Neither device is addressed by a COM port. The Touch is found by
        # OpenHaptics from its own driver configuration - a NAME, not a
        # port - and the controller is an IP found by broadcast. Both are
        # settable from the window so neither is baked in.
        self.touch_name = ""          # "" = the OpenHaptics default device
        self.macs_addr  = ""          # "" = discover by broadcast

    # ---- called from the GUI thread; only ever enqueues -----------------
    def send_command(self, name: str, value: int | None = None):
        self.commands.put((name, value))

    def _set(self, **kw):
        with self.shared.lock:
            for k, v in kw.items():
                setattr(self.shared, k, v)

    def run(self):
        # >>> THE TOUCH GOES FIRST, AND ON PURPOSE. <<<
        # Finding the controller means a broadcast that takes about five
        # seconds when nothing answers. Doing that first left the window
        # saying "Touch: not open" for five seconds at every start-up,
        # which reads as the haptic device not being detected. It opens in
        # well under a second, so it opens first.
        self._connect_touch()
        # >>> AND THE CONTROLLER SEARCH DOES NOT BLOCK THE POSE LOOP. <<<
        # The broadcast takes about five seconds when nothing answers.
        # Done on this thread it froze the whole window for that long -
        # including the raw stylus row, which is the one thing that proves
        # the haptic device is alive. discover_ip() touches no socket of
        # ours, so it is safe on a thread of its own; only the open() has
        # to come back here, which it does through the command queue.
        self._start_macs_search()
        try:
            self._loop()
        except Exception as e:                      # noqa: BLE001
            self._set(note=f"loop stopped: {e}")
        finally:
            self._shutdown()

    def _connect_touch(self):
        """Open the haptic device. A failure here is reportable, not fatal:
        the Reconnect button exists so a flat power supply or a device
        plugged in late does not mean restarting the program."""
        if self._dev is not None:
            return
        name = self.touch_name.strip()
        self._set(note=f"opening the Touch{f' as {name!r}' if name else ''} ...")
        try:
            dev = Touch(name or None).open()
        except Exception as e:                      # noqa: BLE001
            self._set(touch_ok=False, note=f"Touch will not open: {e}")
            return
        if not dev.is_live():
            dev.close()
            self._set(touch_ok=False,
                      note="Touch opened but returns no valid orientation - "
                           "check its power supply, it is separate from USB")
            return
        self._dev = dev
        info = dev.info
        self._set(touch_id=f"{info['model']} {info['serial']}")
        # The insertion wall is ON from the start. It is the only fence the
        # operator can actually feel, so leaving it opt-in was wrong.
        self._set_wall(True)
        self._set(touch_ok=True, note="Touch ready")

    def _start_macs_search(self):
        """Find the controller off-thread and queue the open."""
        def search():
            addr = self.macs_addr.strip()
            if not addr:
                self._set(note="looking for the controller on Ethernet ...")
                addr = discover_ip() or ""
            if addr:
                self.commands.put(("connect_macs", addr))
            else:
                self._set(macs_ok=False,
                          note="no controller answered the broadcast - type its "
                               "address and press Reconnect, or check X4")
        threading.Thread(target=search, daemon=True).start()

    def _connect_macs(self):
        """Open the controller. Discovers the address unless one was given."""
        if self._macs is not None:
            return
        addr = self.macs_addr.strip()
        if not addr:
            self._set(note="looking for the controller on Ethernet ...")
            addr = discover_ip() or ""
            if not addr:
                self._set(macs_ok=False,
                          note="no controller answered the broadcast - type its "
                               "address and press Reconnect, or check X4")
                return
            self.macs_addr = addr
        try:
            self._macs = MacsTcp(addr).open()
        except MacsTcpError as e:
            self._set(macs_ok=False, note=f"controller {addr}: {e}")
            return
        self._set(macs_ok=True, note=f"controller at {addr}")
        # A controller still running a pre-tremor build leaves slot 19 at
        # zero, which this code is entitled to read as "band off". Say so
        # rather than quietly not filtering.
        try:
            band = self._macs.read_param(slot("USR_TELE_TREMOR"))
            self.cfg.tremor_deg = max(0.0, band / 100.0)
            self._set(tremor=band)
            if band == 0:
                self._set(note=f"controller at {addr} - TREMOR BAND IS 0, no "
                               f"filtering. Download the current .mc, or set "
                               f"it here and Apply.")
        except MacsTcpError:
            pass

    def _disconnect(self):
        """Drop both so the next connect attempt starts clean."""
        if self._macs is not None:
            self._macs.close()
            self._macs = None
        if self._dev is not None:
            try:
                self._dev.stop_wall_loop()
            except Exception:                       # noqa: BLE001
                pass
            self._dev.close()
            self._dev = None
        self.wall_on = False
        self._set(touch_ok=False, macs_ok=False)

    def _shutdown(self):
        if self._macs is not None:
            # Enable first: even if the STOP is lost, a cleared enable
            # stops the rig following.
            for fn in (lambda: self._macs.write_param(slot("USR_TELE_ENABLE"), 0),
                       lambda: self._macs.write_param(slot("USR_COMMAND"),
                                                      D["C_CMD_STOP"])):
                try:
                    fn()
                except MacsTcpError:
                    pass
            self._macs.close()
        if self._dev is not None:
            try:
                self._dev.stop_wall_loop()
            except Exception:                       # noqa: BLE001
                pass
            self._dev.close()

    def _drain_commands(self):
        while True:
            try:
                name, value = self.commands.get_nowait()
            except queue.Empty:
                return
            try:
                # >>> A COMMAND WITH NO CONTROLLER MUST NOT VANISH. <<<
                # This used to be "if name == cmd and self._macs", so with
                # no connection a button press was silently discarded -
                # indistinguishable from a broken button.
                if name == "cmd":
                    code, label = value
                    if self._macs is None:
                        self._set(note=f"{label} NOT SENT - no controller "
                                       f"connected. Reconnect first.")
                    else:
                        self._macs.write_param(slot("USR_COMMAND"), code)
                        self._set(note=f"{label} sent")
                elif name == "zero":
                    if not (self.mapper.rot_engaged or self.mapper.trans_engaged):
                        self.mapper.pose_deg[:] = 0.0
                        self.mapper.tool_mm = 0.0
                        self._set(note="PC pose zeroed")
                    else:
                        self._set(note="release the clutches before zeroing")
                elif name == "speeds":
                    if self._macs is None:
                        self._set(note="speeds NOT SENT - no controller connected")
                        continue
                    vr, vt, acc, tremor = value
                    self._macs.write_param(slot("USR_TELE_VEL_ROT"), vr)
                    self._macs.write_param(slot("USR_TELE_VEL_TOOL"), vt)
                    self._macs.write_param(slot("USR_TELE_ACC_SCALE"), acc)
                    self._macs.write_param(slot("USR_TELE_TREMOR"), tremor)
                    self._set(note=f"follow {vr}, {vt}, {acc}%, "
                                   f"tremor {tremor/100:.2f} deg")
                elif name == "wall":
                    self._set_wall(bool(value))
                elif name == "connect_macs":
                    self.macs_addr = value
                    self._connect_macs()
                elif name == "reconnect":
                    self.touch_name, self.macs_addr = value
                    self._disconnect()
                    self._connect_touch()
                    self._start_macs_search()
            except MacsTcpError as e:
                self._set(note=f"command failed: {e}")

    def _set_wall(self, want: bool):
        if want == self.wall_on or self._dev is None:
            return
        if want:
            self._dev.set_wall_gains(stiffness=wallmod.DEFAULT_STIFFNESS,
                                     damping=wallmod.DEFAULT_DAMPING,
                                     max_force=wallmod.DEFAULT_MAX_FORCE)
            self._dev.enable_force()
            self._dev.start_wall_loop()
            self._set(note="force output ENABLED")
        else:
            self._dev.stop_wall_loop()
            self._set(note="force wound down")
        self.wall_on = want

    def _loop(self):
        period = 1.0 / RATE_HZ
        t0 = time.perf_counter()
        prev_buttons = 0
        prev_enable = None
        hb = n = failures = 0

        while not self.stop_flag.is_set():
            loop = time.perf_counter()
            self._drain_commands()

            # No device: idle slowly rather than spinning or dying, so the
            # window stays live and Reconnect still gets serviced.
            if self._dev is None:
                time.sleep(0.1)
                continue

            s = self._dev.snapshot() if self.wall_on else self._dev.read()

            rot, tra = bool(s.buttons & BUTTON_1), bool(s.buttons & BUTTON_2)
            rot_was = bool(prev_buttons & BUTTON_1)
            tra_was = bool(prev_buttons & BUTTON_2)
            prev_buttons = s.buttons

            if rot and not rot_was:
                self.mapper.engage_rotation(s)
            elif rot_was and not rot:
                self.mapper.release_rotation()
            if tra and not tra_was:
                self.mapper.engage_translation(s)
                if self.wall_on:
                    lo, hi = self.mapper.travel_limits_mm()
                    self._dev.set_wall(self.mapper.wall_anchor_device,
                                       self.mapper.wall_axis_device, lo, hi)
            elif tra_was and not tra:
                self.mapper.release_translation()
                if self.wall_on:
                    self._dev.clear_wall()

            cmd = self.mapper.update(s)
            n += 1
            enable = 1 if (rot or tra) else 0
            send_ms = 0.0

            if self._macs is not None:
                t_send = time.perf_counter()
                try:
                    self._macs.write_params(slot("USR_TGT_PSI"), [
                        round(cmd.psi_deg * 100), round(cmd.phi_deg * 100),
                        round(cmd.theta_n_deg * 100), round(cmd.tool_mm * 100)])
                    hb = (hb + 1) & 0x7FFFFFFF
                    self._macs.write_param(slot("USR_TELE_HEARTBEAT"), hb)
                    if enable != prev_enable:
                        self._macs.write_param(slot("USR_TELE_ENABLE"), enable)
                        prev_enable = enable
                except MacsTcpError as e:
                    failures += 1
                    if failures <= 3:
                        self._set(note=f"send failed: {e}")
                send_ms = (time.perf_counter() - t_send) * 1000

                if n % STATUS_EVERY == 0:
                    try:
                        # The band is applied HERE, on the PC, so the
                        # targets leave already smooth. The panel owns the
                        # number; this picks up whatever it was set to.
                        band = self._macs.read_param(slot("USR_TELE_TREMOR"))
                        self.cfg.tremor_deg = max(0.0, band / 100.0)
                        self._set(tremor=band)
                        self._set(
                            rig_state  = self._macs.read_param(slot("USR_STATE")),
                            rig_msg    = self._macs.read_param(slot("USR_MSG")),
                            led        = self._macs.read_param(slot("USR_LED")),
                            tele_state = self._macs.read_param(slot("USR_TELE_STATE")),
                            hb_age     = self._macs.read_param(slot("USR_TELE_HB_AGE")),
                            max_age    = self._macs.read_param(slot("USR_TELE_MAX_AGE")),
                            trips      = self._macs.read_param(slot("USR_TELE_TRIPS")),
                            clamped_rig= self._macs.read_param(slot("USR_TELE_CLAMPED")),
                            ik         = tuple(self._macs.read_params(
                                            slot("USR_AX1_CDEG"), slot("USR_AXT_UU"))),
                        )
                    except MacsTcpError:
                        pass

            (sr, sp, sy), _ = euler_xyz_deg(s.rotation)
            self._set(rot=rot, tra=tra, enable=enable, send_ms=send_ms,
                      failures=failures,
                      stylus_pos=(s.position[0], s.position[1], s.position[2]),
                      stylus_rpy=(sr, sp, sy),
                      pose=(cmd.psi_deg, cmd.phi_deg, cmd.theta_n_deg, cmd.tool_mm),
                      clamped_pc=cmd.clamped,
                      rate=n / max(time.perf_counter() - t0, 1e-6))

            slack = period - (time.perf_counter() - loop)
            if slack > 0:
                time.sleep(slack)


# --------------------------------------------------------------- the window
class App:
    def __init__(self, root: tk.Tk, worker: Worker, cfg: MapConfig):
        self.root = root
        self.worker = worker
        self.cfg = cfg
        root.title("3R1T haptic teleoperation")
        root.protocol("WM_DELETE_WINDOW", self.quit)
        root.minsize(620, 560)

        pad = dict(padx=8, pady=4)
        self.vars: dict[str, tk.StringVar] = {}

        # ---- connection -------------------------------------------------
        #
        # >>> NEITHER OF THESE IS A COM PORT. <<<
        #
        # The Touch does enumerate as a COM port in Windows, but nothing
        # here opens it that way - OpenHaptics finds it from its own driver
        # configuration, which is identified by a NAME. Blank means the
        # default device, which is the only one configured unless somebody
        # has run Touch_Setup and defined more.
        #
        # The controller is an IP address, not a port either. Blank means
        # find it by broadcast, which is the normal case - it self-assigns
        # a link-local 169.254.x that can change between boots, so a fixed
        # address here would be the thing that went stale.
        box = ttk.LabelFrame(root, text="Connection")
        box.pack(fill="x", **pad)

        ttk.Label(box, text="Touch device", width=22, anchor="e").grid(
            row=0, column=0, sticky="e", padx=6, pady=2)
        self.touch_name = tk.StringVar(value="")
        self.touch_pick = ttk.Combobox(box, textvariable=self.touch_name,
                                       width=18, values=("",))
        self.touch_pick.grid(row=0, column=1, sticky="w")
        ttk.Label(box, text="blank = default", foreground="grey45").grid(
            row=0, column=2, sticky="w", padx=6)

        ttk.Label(box, text="Controller address", width=22, anchor="e").grid(
            row=1, column=0, sticky="e", padx=6, pady=2)
        self.macs_addr = tk.StringVar(value="")
        ttk.Entry(box, textvariable=self.macs_addr, width=18).grid(
            row=1, column=1, sticky="w")
        ttk.Label(box, text="blank = find it", foreground="grey45").grid(
            row=1, column=2, sticky="w", padx=6)

        ttk.Button(box, text="Reconnect", command=self.reconnect).grid(
            row=0, column=3, rowspan=2, padx=10)

        self._row(box, 2, "Touch", "touch")
        self._row(box, 3, "Controller", "macs")
        self._row(box, 4, "Loop", "loop")

        # ---- rig --------------------------------------------------------
        box = ttk.LabelFrame(root, text="Rig")
        box.pack(fill="x", **pad)
        self.lamp = tk.Canvas(box, width=26, height=26, highlightthickness=0)
        self.lamp.grid(row=0, column=2, rowspan=3, padx=10)
        self.lamp_id = self.lamp.create_oval(3, 3, 23, 23, fill="grey30",
                                             outline="grey50")
        self._row(box, 0, "State", "rig_state")
        self._row(box, 1, "Message", "rig_msg")
        self._row(box, 2, "Teleop", "tele")

        # ---- raw stylus -------------------------------------------------
        #
        # >>> THIS ROW IS WHY A STILL WINDOW DOES NOT MEAN A DEAD DEVICE. <<<
        #
        # Everything below is a COMMANDED value, so with no clutch held it
        # all sits at zero and waving the stylus changes nothing on screen -
        # which looks exactly like the haptic device not being detected.
        # This row is the raw device, always live, clutch or no clutch.
        box = ttk.LabelFrame(root, text="Stylus  (raw - always live)")
        box.pack(fill="x", **pad)
        self._row(box, 0, "position  x y z [mm]", "sty_pos")
        self._row(box, 1, "roll pitch yaw [deg]", "sty_rpy")

        # ---- pose -------------------------------------------------------
        box = ttk.LabelFrame(root, text="Commanded pose   (* = clamped)")
        box.pack(fill="x", **pad)
        for i, name in enumerate(("psi [deg]", "phi [deg]",
                                  "theta_n [deg]", "tool [mm]")):
            self._row(box, i, name, f"pose{i}")
        self._row(box, 4, "Rig IK  [cdeg / UU]", "ik")

        # ---- link -------------------------------------------------------
        box = ttk.LabelFrame(root, text="Link health")
        box.pack(fill="x", **pad)
        self._row(box, 0, "Heartbeat age", "hb")
        self._row(box, 1, "Worst gap", "maxage")
        self._row(box, 2, "Watchdog drop-outs", "trips")
        self._row(box, 3, "Send failures", "fail")
        self._row(box, 4, "Clutches / enable", "clutch")

        # ---- follow speeds ----------------------------------------------
        box = ttk.LabelFrame(root, text="Follow speeds")
        box.pack(fill="x", **pad)
        self.sp = {}
        for i, (lbl, key, default) in enumerate((
                ("angles [cdeg/s]", "vr", D.get("C_TELE_VEL_ROT_DEF", 400)),
                ("tool [0.01 mm/s]", "vt", D.get("C_TELE_VEL_TOOL_DEF", 300)),
                ("accel [% of MOVE]", "acc", D.get("C_TELE_ACC_PCT_DEF", 50)),
                ("tremor band [cdeg]", "tremor", D.get("C_TELE_TREMOR_DEF", 100)))):
            ttk.Label(box, text=lbl, width=22, anchor="e").grid(
                row=i, column=0, sticky="e", padx=6, pady=2)
            v = tk.StringVar(value=str(default))
            ttk.Entry(box, textvariable=v, width=10).grid(row=i, column=1, sticky="w")
            self.sp[key] = v
        ttk.Button(box, text="Apply", command=self.apply_speeds).grid(
            row=0, column=2, rowspan=4, padx=10)

        self.wall_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(box, text="virtual wall on the insertion axis "
                                  "(enables force output)",
                        variable=self.wall_var,
                        command=self.toggle_wall).grid(
            row=4, column=0, columnspan=3, sticky="w", padx=6, pady=(6, 2))

        # ---- buttons ----------------------------------------------------
        bar = ttk.Frame(root)
        bar.pack(fill="x", **pad)
        self.rig_buttons = []
        for text, code in (("HOME", "C_CMD_HOME"), ("TELEOP", "C_CMD_TELEOP"),
                           ("Clear error", "C_CMD_ERROR_CLR")):
            b = ttk.Button(bar, text=text, width=12,
                           command=lambda c=code: self.cmd(c))
            b.pack(side="left", padx=4)
            self.rig_buttons.append(b)
        # Zeroing is PC-side and works with no controller at all.
        ttk.Button(bar, text="Zero PC pose", width=14,
                   command=lambda: worker.send_command("zero")).pack(side="left", padx=4)

        self.stop_btn = stop = tk.Button(
                         root, text="S T O P", command=lambda: self.cmd("C_CMD_STOP"),
                         bg="#b00020", fg="white", activebackground="#d32f2f",
                         activeforeground="white",
                         font=("Segoe UI", 16, "bold"), height=2)
        stop.pack(fill="x", padx=8, pady=(6, 2))

        self.note = ttk.Label(root, text="", anchor="w", foreground="grey20")
        self.note.pack(fill="x", padx=10, pady=(0, 8))

        ttk.Label(root, anchor="w", foreground="grey45", wraplength=600,
                  text="Hold either clutch on the stylus to hold the enable. "
                       "None of these buttons is an emergency stop.").pack(
            fill="x", padx=10, pady=(0, 8))

        self.refresh()

    def _row(self, parent, r, label, key):
        ttk.Label(parent, text=label, width=22, anchor="e").grid(
            row=r, column=0, sticky="e", padx=6, pady=2)
        v = tk.StringVar(value="-")
        self.vars[key] = v
        ttk.Label(parent, textvariable=v, anchor="w",
                  font=("Consolas", 10)).grid(row=r, column=1, sticky="w")

    # ---- actions --------------------------------------------------------
    def cmd(self, name: str):
        # The label travels with the code so a refusal can name the button
        # rather than reporting a bare number.
        self.worker.send_command("cmd", (D[name], name[6:].replace("_", " ")))

    def apply_speeds(self):
        try:
            vals = (int(self.sp["vr"].get()), int(self.sp["vt"].get()),
                    int(self.sp["acc"].get()), int(self.sp["tremor"].get()))
        except ValueError:
            self.note.config(text="follow speeds must be whole numbers")
            return
        self.worker.send_command("speeds", vals)

    def toggle_wall(self):
        self.worker.send_command("wall", int(self.wall_var.get()))

    def reconnect(self):
        """Drop both devices and open them again with whatever is typed.

        Does the work on the worker thread like every other command - the
        socket and the haptic device both belong to it."""
        self.worker.send_command("reconnect",
                                 (self.touch_name.get(), self.macs_addr.get()))

    def quit(self):
        self.note.config(text="stopping - clearing the enable and sending STOP")
        self.root.update_idletasks()
        self.worker.stop_flag.set()
        self.worker.join(timeout=3.0)
        self.root.destroy()

    # ---- the poll -------------------------------------------------------
    def refresh(self):
        s = self.worker.shared.snapshot()
        v = self.vars

        v["touch"].set(f"open   {s['touch_id']}" if s["touch_ok"] else "not open")
        v["sty_pos"].set("  ".join(f"{c:+8.2f}" for c in s["stylus_pos"]))
        v["sty_rpy"].set("  ".join(f"{c:+8.2f}" for c in s["stylus_rpy"]))
        v["macs"].set("open" if s["macs_ok"] else "not open")
        # Blank means "find it", so show what was found rather than leaving
        # the operator guessing which address is in use.
        if s["macs_ok"] and not self.macs_addr.get().strip():
            self.macs_addr.set(self.worker.macs_addr)
        v["loop"].set(f"{s['rate']:5.1f} Hz     send {s['send_ms']:4.1f} ms")

        v["rig_state"].set(f"0x{s['rig_state']:03X}  {bits(s['rig_state'], 'C_STA_')}")
        v["rig_msg"].set(MSG_TEXT.get(s["rig_msg"], f"code {s['rig_msg']}"))
        v["tele"].set(enum_name(s["tele_state"], "TELE_"))

        cl = s["clamped_pc"]
        for i, (name, val) in enumerate(zip(("psi", "phi", "theta_n", "tool"),
                                            s["pose"])):
            v[f"pose{i}"].set(f"{val:+8.2f} {'*' if name in cl else ' '}")
        v["ik"].set("  ".join(f"{x:+7d}" for x in s["ik"]))

        v["hb"].set(f"{s['hb_age']:4d} ms")
        v["maxage"].set(f"{s['max_age']:4d} ms")
        v["trips"].set(str(s["trips"]))
        v["fail"].set(str(s["failures"]))
        rigclamp = s["clamped_rig"]
        names = [n for b, n in ((1, "psi"), (2, "phi"), (4, "theta_n"), (8, "tool"))
                 if rigclamp & b]
        v["clutch"].set(
            f"{'ROT' if s['rot'] else '---'} {'TRA' if s['tra'] else '---'}"
            f"   enable {s['enable']}   tremor {s['tremor']/100:.2f} deg"
            + (f"   rig clamping {' '.join(names)}" if names else ""))

        # Nothing to send to means nothing to press. Clearer than letting a
        # press look like it worked.
        state = "normal" if s["macs_ok"] else "disabled"
        for b in self.rig_buttons:
            b.config(state=state)
        self.stop_btn.config(state=state)

        colour = {1: "#2e7d32", 2: "#f9a825", 4: "#b00020"}.get(s["led"], "grey30")
        if s["tele_state"] == D.get("TELE_FOLLOW", 2):
            colour = "#1565c0"
        self.lamp.itemconfig(self.lamp_id, fill=colour)
        self.note.config(text=s["note"])

        self.root.after(80, self.refresh)


def main() -> int:
    cfg = MapConfig.from_config_mh(CONFIG_MH)
    shared = Shared()
    worker = Worker(shared, cfg)
    worker.start()

    root = tk.Tk()
    try:
        App(root, worker, cfg)
        root.mainloop()
    finally:
        worker.stop_flag.set()
        worker.join(timeout=3.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
