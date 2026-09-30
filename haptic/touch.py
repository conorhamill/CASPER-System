"""
ctypes binding for the OpenHaptics HD API, for the 3D Systems Touch.

Force output is OFF unless enable_force() is called explicitly, so reading
is always safe and the arm stays limp. Once enabled, the only thing that
ever commands force is the wall in wall.py, driven from the servo thread.

Needs the OpenHaptics toolkit (hd.dll, which the installer puts in System32)
and the Touch Device Drivers.

    from touch import Touch
    with Touch() as dev:
        s = dev.read()
        print(s.position, s.buttons)
"""
from __future__ import annotations

import ctypes as C
import time
from dataclasses import dataclass

import numpy as np

import wall

# ---------------------------------------------------------------- constants
HD_SUCCESS             = 0x0000
CURRENT_BUTTONS        = 0x2000
# Unsupported on this Touch: every hdGetIntegerv on it returns zeros AND
# raises 0x0103, which then sits in the error queue waiting to be blamed on
# whatever is checked next. Not queried anywhere - kept only so nobody
# rediscovers it and adds it back.
CURRENT_ENCODER_VALUES = 0x2010
CURRENT_POSITION       = 0x2050
CURRENT_VELOCITY       = 0x2051
CURRENT_TRANSFORM      = 0x2052
CURRENT_JOINT_ANGLES   = 0x2100
CURRENT_GIMBAL_ANGLES  = 0x2150
DEVICE_MODEL_TYPE      = 0x2501
DEVICE_DRIVER_VERSION  = 0x2502
DEVICE_SERIAL_NUMBER   = 0x2504
USABLE_WORKSPACE       = 0x2551
UPDATE_RATE_INSTANT    = 0x2601
NOMINAL_MAX_STIFFNESS  = 0x2602
NOMINAL_MAX_FORCE      = 0x2603
NOMINAL_MAX_CONT_FORCE = 0x2604
NOMINAL_MAX_DAMPING    = 0x2609
CURRENT_FORCE          = 0x2700
# Enable/disable CAPABILITIES - not settable values. hdSetDoublev on one of
# these returns HD_INVALID_ENUM.
FORCE_OUTPUT           = 0x4000
MAX_FORCE_CLAMPING     = 0x4001
SOFTWARE_FORCE_LIMIT   = 0x4003

HD_CALLBACK_DONE     = 0
HD_CALLBACK_CONTINUE = 1
_PRIORITY_MAX        = 0xFFFF

BUTTON_1 = 1 << 0
BUTTON_2 = 1 << 1

CALIBRATION_NAMES = {0x5000: "OK", 0x5001: "NEEDS_UPDATE", 0x5002: "NEEDS_MANUAL_INPUT"}
STYLE_NAMES       = {1: "ENCODER_RESET", 2: "AUTO", 4: "INKWELL"}

_PRIORITY = 0xFFFF // 2                      # HD_DEFAULT_SCHEDULER_PRIORITY


class HDErrorInfo(C.Structure):
    """hdGetError returns this BY VALUE.

    It is 12 bytes, so on the x64 ABI it comes back through a hidden pointer.
    Declaring restype as c_uint instead corrupts the register layout and
    faults the *next* call into the DLL - which looks exactly like a dead
    device and is not at all obvious. Leave this alone.
    """
    _fields_ = [("errorCode", C.c_uint), ("internalErrorCode", C.c_int), ("hHD", C.c_uint)]


_CALLBACK = C.CFUNCTYPE(C.c_uint, C.c_void_p)    # HDCallbackCode (*)(void *)


class HDError(RuntimeError):
    pass


def _load():
    hd = C.CDLL("hd.dll")
    hd.hdInitDevice.restype,  hd.hdInitDevice.argtypes  = C.c_uint, [C.c_char_p]
    hd.hdGetError.restype        = HDErrorInfo
    hd.hdGetErrorString.restype  = C.c_char_p
    hd.hdGetErrorString.argtypes = [C.c_uint]
    hd.hdGetString.restype       = C.c_char_p
    hd.hdGetString.argtypes      = [C.c_uint]
    hd.hdGetCurrentDevice.restype   = C.c_uint
    hd.hdMakeCurrentDevice.argtypes = [C.c_uint]
    hd.hdBeginFrame.argtypes = [C.c_uint]
    hd.hdEndFrame.argtypes   = [C.c_uint]
    hd.hdCheckCalibration.restype      = C.c_uint
    hd.hdCheckCalibrationStyle.restype = C.c_uint
    hd.hdScheduleSynchronous.argtypes  = [_CALLBACK, C.c_void_p, C.c_ushort]
    hd.hdScheduleAsynchronous.restype  = C.c_ulong
    hd.hdScheduleAsynchronous.argtypes = [_CALLBACK, C.c_void_p, C.c_ushort]
    hd.hdUnschedule.argtypes    = [C.c_ulong]
    hd.hdGetDoublev.argtypes    = [C.c_uint, C.POINTER(C.c_double)]
    hd.hdSetDoublev.argtypes    = [C.c_uint, C.POINTER(C.c_double)]
    hd.hdGetIntegerv.argtypes   = [C.c_uint, C.POINTER(C.c_int)]
    hd.hdEnable.argtypes        = [C.c_uint]
    hd.hdDisable.argtypes       = [C.c_uint]
    hd.hdDisableDevice.argtypes = [C.c_uint]
    return hd


@dataclass
class Sample:
    """One consistent snapshot of the device, taken inside the servo loop."""
    position:    np.ndarray      # (3,)   mm, device world frame
    velocity:    np.ndarray      # (3,)   mm/s
    rotation:    np.ndarray      # (3, 3) stylus orientation in the world frame
    gimbal:      np.ndarray      # (3,)   rad, RAW wrist joints - not an orientation
    joints:      np.ndarray      # (3,)   rad
    buttons:     int
    rate_hz:     float
    calibration: int

    def button(self, mask: int) -> bool:
        return bool(self.buttons & mask)


class Touch:
    def __init__(self, name: str | None = None):
        self._hd      = _load()
        self._name    = name.encode() if name else None
        self._hhd     = None
        self._running = False
        # Preallocated so the servo callback allocates nothing.
        self._d16 = (C.c_double * 16)()
        self._pos = (C.c_double * 3)()
        self._vel = (C.c_double * 3)()
        self._gim = (C.c_double * 3)()
        self._jnt = (C.c_double * 3)()
        self._rate = (C.c_double * 1)()
        self._btn = C.c_int()
        self._cal = C.c_uint()
        self._force = (C.c_double * 3)()
        self._cb  = _CALLBACK(self._sample_in_servo_thread)   # keep a reference alive
        self._wall_cb = _CALLBACK(self._wall_in_servo_thread)
        self._wall_handle = None
        self._force_enabled = False
        # Wall state. Plain floats rather than arrays so the servo thread
        # never sees a half-written vector, and so the callback allocates
        # nothing at 1 kHz.
        self._wall_on = False
        self._w_p0x = self._w_p0y = self._w_p0z = 0.0
        self._w_ux = self._w_uy = self._w_uz = 0.0
        self._w_lo = self._w_hi = 0.0
        self._w_k = wall.DEFAULT_STIFFNESS
        self._w_b = wall.DEFAULT_DAMPING
        self._w_fmax = wall.DEFAULT_MAX_FORCE
        self._w_step = 0.05          # [N] per tick, ~60 ms to full scale
        self._w_force = 0.0          # last applied, along the axis
        self._w_s = 0.0              # last travel, for display

    # -------------------------------------------------------------- lifetime
    def _drain(self) -> list:
        """Empty the error queue and return what was in it.

        HD queues errors rather than keeping only the last one. Popping a
        single entry - which is what a naive check does - can report an
        error raised by some earlier, unrelated call and pin it on the one
        you just made. That cost an hour here: an unsupported encoder read
        in the servo callback was being blamed on hdEnable.
        """
        out = []
        for _ in range(16):
            e = self._hd.hdGetError()
            if e.errorCode == HD_SUCCESS:
                break
            msg = self._hd.hdGetErrorString(e.errorCode)
            out.append(f"0x{e.errorCode:04X} "
                       f"{msg.decode(errors='replace') if msg else ''} "
                       f"(internal {e.internalErrorCode})")
        return out

    def _check(self, where: str):
        errs = self._drain()
        if errs:
            raise HDError(f"{where}: " + "; ".join(errs))

    def open(self) -> "Touch":
        self._hhd = self._hd.hdInitDevice(self._name)
        self._check("hdInitDevice")
        # Harmless with one device, required with more than one. The vendor
        # sample and pyOpenHaptics both do it, so do it.
        self._hd.hdMakeCurrentDevice(self._hhd)
        self._hd.hdStartScheduler()
        self._check("hdStartScheduler")
        self._running = True
        return self

    def close(self):
        # Order matters: wind the force down and unschedule the loop BEFORE
        # stopping the scheduler, so the arm is never left mid-push.
        self.stop_wall_loop()
        if self._force_enabled:
            try:
                self._hd.hdDisable(FORCE_OUTPUT)
            except Exception:
                pass
            self._force_enabled = False
        if self._running:
            self._hd.hdStopScheduler()
            self._running = False
        if self._hhd is not None:
            self._hd.hdDisableDevice(self._hhd)
            self._hhd = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()
        return False

    # ------------------------------------------------------------------ info
    def _string(self, code: int) -> str:
        v = self._hd.hdGetString(code)
        return v.decode(errors="replace") if v else "<none>"

    def _scalar(self, code: int) -> float:
        d = (C.c_double * 6)()
        self._hd.hdGetDoublev(code, d)
        return d[0]

    @property
    def info(self) -> dict:
        # The device knows its own limits. Ask it rather than hard-coding
        # numbers from a datasheet that may not describe this unit.
        out = {
            "model":             self._string(DEVICE_MODEL_TYPE),
            "driver":            self._string(DEVICE_DRIVER_VERSION),
            "serial":            self._string(DEVICE_SERIAL_NUMBER),
            "max_force_N":       self._scalar(NOMINAL_MAX_FORCE),
            "max_cont_force_N":  self._scalar(NOMINAL_MAX_CONT_FORCE),
            "max_stiffness":     self._scalar(NOMINAL_MAX_STIFFNESS),   # [N/mm]
            "max_damping":       self._scalar(NOMINAL_MAX_DAMPING),     # [N.s/mm]
        }
        d = (C.c_double * 6)()
        self._hd.hdGetDoublev(USABLE_WORKSPACE, d)
        out["workspace"] = {"x": (d[0], d[3]), "y": (d[1], d[4]), "z": (d[2], d[5])}
        style = self._hd.hdCheckCalibrationStyle()
        out["cal_styles"] = [n for b, n in STYLE_NAMES.items() if style & b]
        return out

    # ------------------------------------------------------------- servo read
    def _sample_in_servo_thread(self, _):
        """Runs in the 1 kHz servo thread. No allocation, no I/O, no exceptions."""
        hd  = self._hd
        hhd = hd.hdGetCurrentDevice()
        hd.hdBeginFrame(hhd)
        hd.hdGetDoublev(CURRENT_TRANSFORM,     self._d16)
        hd.hdGetDoublev(CURRENT_POSITION,      self._pos)
        hd.hdGetDoublev(CURRENT_VELOCITY,      self._vel)
        hd.hdGetDoublev(CURRENT_GIMBAL_ANGLES, self._gim)
        hd.hdGetDoublev(CURRENT_JOINT_ANGLES,  self._jnt)
        hd.hdGetDoublev(UPDATE_RATE_INSTANT,   self._rate)
        hd.hdGetIntegerv(CURRENT_BUTTONS, C.byref(self._btn))
        self._cal.value = hd.hdCheckCalibration()
        hd.hdEndFrame(hhd)
        return 0                                   # HD_CALLBACK_DONE

    def read(self) -> Sample:
        self._hd.hdScheduleSynchronous(self._cb, None, _PRIORITY)
        t = self._d16
        # HD_CURRENT_TRANSFORM is a COLUMN-major 4x4: element (i, j) is t[4*j + i].
        # Getting this transposed is the classic way to end up with a mapping
        # that looks fine near home and is wrong everywhere else.
        rot = np.array([[t[0], t[4], t[8]],
                        [t[1], t[5], t[9]],
                        [t[2], t[6], t[10]]], dtype=float)
        return Sample(
            position    = np.array(self._pos[:3], dtype=float),
            velocity    = np.array(self._vel[:3], dtype=float),
            rotation    = rot,
            gimbal      = np.array(self._gim[:3], dtype=float),
            joints      = np.array(self._jnt[:3], dtype=float),
            buttons     = self._btn.value,
            rate_hz     = self._rate[0],
            calibration = self._cal.value,
        )

    def snapshot(self) -> Sample:
        """Build a Sample from the buffers WITHOUT touching HD.

        Use this instead of read() while the wall loop is running: that loop
        is already refreshing the buffers at 1 kHz, and adding a synchronous
        callback on top only adds contention. A torn read here costs one
        stale component on a display line, which does not matter.
        """
        t = self._d16
        rot = np.array([[t[0], t[4], t[8]],
                        [t[1], t[5], t[9]],
                        [t[2], t[6], t[10]]], dtype=float)
        return Sample(
            position    = np.array(self._pos[:3], dtype=float),
            velocity    = np.array(self._vel[:3], dtype=float),
            rotation    = rot,
            gimbal      = np.array(self._gim[:3], dtype=float),
            joints      = np.array(self._jnt[:3], dtype=float),
            buttons     = self._btn.value,
            rate_hz     = self._rate[0],
            calibration = self._cal.value,
        )

    # ----------------------------------------------------------------- force
    def enable_force(self, limit_n: float = wall.DEFAULT_MAX_FORCE):
        """Arm the motors. The arm stops being limp after this.

        The driver's own guards go on FIRST, so they are already in place
        before anything can command a force. All three are capabilities -
        things you hdEnable - not values you set.
        """
        self._drain()                             # unrelated stale errors
        self._hd.hdEnable(MAX_FORCE_CLAMPING)     # clip at the device maximum
        self._hd.hdEnable(SOFTWARE_FORCE_LIMIT)   # and at the software one
        self._hd.hdEnable(FORCE_OUTPUT)
        self._check("hdEnable(HD_FORCE_OUTPUT)")
        self._w_fmax = min(self._w_fmax, limit_n)
        self._force_enabled = True

    def set_wall_gains(self, stiffness: float = None, damping: float = None,
                       max_force: float = None):
        if stiffness is not None:
            self._w_k = stiffness
        if damping is not None:
            self._w_b = damping
        if max_force is not None:
            self._w_fmax = max_force

    def set_wall(self, anchor, axis, lo: float, hi: float):
        """Place the wall. Called from the main thread on a clutch press.

        anchor/axis are in the DEVICE frame - the servo thread works in raw
        device coordinates and knows nothing about the robot's base frame.
        """
        self._w_p0x, self._w_p0y, self._w_p0z = (float(v) for v in anchor)
        self._w_ux, self._w_uy, self._w_uz    = (float(v) for v in axis)
        self._w_lo, self._w_hi = float(lo), float(hi)
        self._wall_on = True

    def clear_wall(self):
        """Stop commanding wall force. The rate limiter decays it to zero
        rather than dropping it, so releasing the clutch is not a step."""
        self._wall_on = False

    @property
    def wall_force_n(self) -> float:
        return self._w_force

    @property
    def wall_s_mm(self) -> float:
        return self._w_s

    def _wall_in_servo_thread(self, _):
        """The 1 kHz loop. Reads state, renders the wall, applies the force.

        No allocation, no numpy, no I/O, no exceptions - this runs 1000
        times a second inside the driver.
        """
        hd  = self._hd
        hhd = hd.hdGetCurrentDevice()
        hd.hdBeginFrame(hhd)
        hd.hdGetDoublev(CURRENT_TRANSFORM,     self._d16)
        hd.hdGetDoublev(CURRENT_POSITION,      self._pos)
        hd.hdGetDoublev(CURRENT_VELOCITY,      self._vel)
        hd.hdGetDoublev(CURRENT_GIMBAL_ANGLES, self._gim)
        hd.hdGetDoublev(CURRENT_JOINT_ANGLES,  self._jnt)
        hd.hdGetDoublev(UPDATE_RATE_INSTANT,   self._rate)
        hd.hdGetIntegerv(CURRENT_BUTTONS, C.byref(self._btn))

        if self._wall_on:
            ux, uy, uz = self._w_ux, self._w_uy, self._w_uz
            s = ((self._pos[0] - self._w_p0x) * ux
                 + (self._pos[1] - self._w_p0y) * uy
                 + (self._pos[2] - self._w_p0z) * uz)
            s_dot = self._vel[0] * ux + self._vel[1] * uy + self._vel[2] * uz
            self._w_s = s
            target = wall.wall_force(s, self._w_lo, self._w_hi,
                                     self._w_k, self._w_b, s_dot, self._w_fmax)
        else:
            target = 0.0

        f = wall.rate_limit(target, self._w_force, self._w_step)
        self._w_force = f
        self._force[0] = f * self._w_ux
        self._force[1] = f * self._w_uy
        self._force[2] = f * self._w_uz
        hd.hdSetDoublev(CURRENT_FORCE, self._force)
        hd.hdEndFrame(hhd)
        return HD_CALLBACK_CONTINUE

    def start_wall_loop(self):
        self._wall_handle = self._hd.hdScheduleAsynchronous(
            self._wall_cb, None, _PRIORITY_MAX)
        self._check("hdScheduleAsynchronous")

    def stop_wall_loop(self):
        """Wind the force down before unscheduling, so the arm is not
        released mid-push."""
        if self._wall_handle is None:
            return
        self._wall_on = False
        deadline = time.perf_counter() + 0.5
        while abs(self._w_force) > 1e-3 and time.perf_counter() < deadline:
            time.sleep(0.005)
        self._hd.hdUnschedule(self._wall_handle)
        self._wall_handle = None

    def is_live(self) -> bool:
        """True if the device is producing real pose data.

        Tested on the ORIENTATION. HD_CURRENT_ENCODER_VALUES is not
        supported on this device at all - it returns zeros and raises - so
        keying liveness on it rejects a perfectly good Touch.

        A dead device returns zeros for everything, which makes the rotation
        block all-zero and its determinant 0. A live one returns a proper
        rotation - orthonormal, determinant +1 - and that is true whether or
        not anybody is moving the stylus, so this needs no motion and no
        waiting.
        """
        R = self.read().rotation
        if not np.isfinite(R).all():
            return False
        return (abs(np.linalg.det(R) - 1.0) < 1e-3
                and np.abs(R.T @ R - np.eye(3)).max() < 1e-3)
