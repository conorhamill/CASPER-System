"""
Talking to the maxon MasterMACS from Python, over the USB cable that is
already plugged in, through the vendor's own ZbMoc.dll.

    from macs import Macs
    with Macs() as m:
        print(m.info)
        print(m.read_params(10, 13))        # USR_TGT_PSI..USR_TGT_TOOL

HOW FAST THIS LINK ACTUALLY IS, measured on this rig

    single SDO read                    2.43 ms
    UserParamReadRaw,  1 param         2.65 ms
    UserParamReadRaw,  4 params        9.84 ms
    UserParamReadRaw, 28 params       70.04 ms
    UserParamReadRaw, 90 params      226.95 ms
    ReadUserArray, 20 longs, 1 call    9.07 ms

>>> USERPARAMREADRAW IS NOT A BLOCK TRANSFER. <<<

It is a loop that costs one round trip PER PARAMETER, about 2.5 ms each,
and the name invites you to assume otherwise. Reading the 28 parameters a
teleop loop wants takes 70 ms - about 14 Hz, before writing anything.

A DIM array IS a block transfer: 20 longs in a single 9 ms call, 5.4x
faster than the same 20 as parameters, and the cost is per CALL rather than
per value. So anything on the hot path belongs in an array, and the
per-parameter calls are for setup and diagnostics only.

Ethernet would be better still and would free USB for the ApossIDE, but
ZbMocOpenTcp finds nothing on this rig today (-219), so it is not an option
until the second Ethernet port is wired and addressed.
"""
from __future__ import annotations

import ctypes as C
import os
from dataclasses import dataclass

from _zbmoc_errors import ZBMOC_ERRORS

APOSS_DLL_DIR = r"C:\Program Files (x86)\Aposs\DLL_x64"

# USER_PARAM(n) is SDO 0x2201 sub n - see Aposs\Include\SdoDictionary.mh.
SDOINDEX_USER_PARAM = 0x2201
USER_PARAM_MAX      = 101


class MacsError(RuntimeError):
    def __init__(self, where: str, code: int):
        name, desc = ZBMOC_ERRORS.get(code, ("?", ""))
        self.code = code
        super().__init__(f"{where}: {code} {name}" + (f" - {desc}" if desc else ""))


class _OpenUsbParam(C.Structure):
    _fields_ = [("baud", C.c_uint32), ("retry", C.c_uint16), ("timeout", C.c_uint32),
                ("flags", C.c_uint32), ("latency", C.c_uint32)]


class _MoconInfo(C.Structure):
    _fields_ = [
        ("state", C.c_uint16), ("controller", C.c_uint16), ("interf", C.c_uint16),
        ("channelno", C.c_uint16), ("full_version", C.c_uint32),
        ("major_version", C.c_uint16), ("minor_version", C.c_uint16),
        ("micro_version", C.c_uint16), ("release_type", C.c_uint16),
        ("cpu_type", C.c_uint8), ("axis_controller_type", C.c_uint16),
        ("number_axis", C.c_uint8), ("option_code", C.c_uint8),
        ("board_revision", C.c_uint8), ("bus_id", C.c_uint16), ("can_id", C.c_uint16),
        ("unit_name", C.c_char * 25), ("regspeed", C.c_uint32),
        ("sdo_compatible", C.c_uint8), ("globalcnt", C.c_uint16),
        ("usercnt", C.c_uint16), ("axiscnt", C.c_uint16), ("arraymax", C.c_uint32),
        ("timebase", C.c_double), ("devclass", C.c_uint16), ("pid", C.c_uint16),
        ("serial", C.c_char * 50), ("description", C.c_char * 100),
        ("banner", C.c_char * 100), ("number_amp", C.c_uint32),
        ("ipaddr", C.c_uint32), ("features", C.c_uint32)]


@dataclass
class MacsInfo:
    name: str
    bus_id: int
    firmware: str
    axes: int
    user_params: int
    global_params: int
    timebase_ms: float

    def __str__(self):
        return (f"{self.name!r} id={self.bus_id} fw={self.firmware} "
                f"axes={self.axes} user_params={self.user_params} "
                f"timebase={self.timebase_ms:g} ms")


def _load():
    os.add_dll_directory(APOSS_DLL_DIR)
    z = C.CDLL(os.path.join(APOSS_DLL_DIR, "ZbMoc.dll"))
    i16, u16, u32, i32p = C.c_int16, C.c_uint16, C.c_uint32, C.POINTER(C.c_int32)
    z.ZbMocOpenUsb.restype,  z.ZbMocOpenUsb.argtypes  = i16, [C.POINTER(_OpenUsbParam)]
    z.ZbMocMoconInfoIdx.restype  = i16
    z.ZbMocMoconInfoIdx.argtypes = [u16, u16, C.POINTER(_MoconInfo)]
    z.ZbMocMoconInfo.restype     = i16
    z.ZbMocMoconInfo.argtypes    = [u16, u16, C.POINTER(_MoconInfo)]
    z.ZbMocConnect.restype,  z.ZbMocConnect.argtypes  = i16, [u16, u16]
    z.ZbMocUserParamReadRaw.restype  = i16
    z.ZbMocUserParamReadRaw.argtypes = [u16, u16, i32p, u16, u16]
    z.ZbMocUserParamWriteRaw.restype  = i16
    z.ZbMocUserParamWriteRaw.argtypes = [u16, u16, i32p, u16, u16]
    z.ZbMocReadUserArray.restype  = i16
    z.ZbMocReadUserArray.argtypes = [u16, u16, u16, u32, i32p, C.POINTER(u32)]
    z.ZbMocWriteUserArray.restype  = i16
    z.ZbMocWriteUserArray.argtypes = [u16, u16, C.POINTER(u16), C.POINTER(u32),
                                      C.POINTER(u32), i32p]
    z.ZbMocCanOpenReadSDO.restype  = i16
    z.ZbMocCanOpenReadSDO.argtypes = [u16, u16, u16, u16, i32p]
    z.ZbMocCanOpenWriteSDO.restype  = i16
    z.ZbMocCanOpenWriteSDO.argtypes = [u16, u16, u16, u16, C.c_int32]
    z.ZbMocClose.restype, z.ZbMocClose.argtypes = i16, [u16]
    z.ZbMocCloseAll.restype = i16
    z.ZbMocGetZbMocVersion.argtypes = [C.c_char_p]
    return z


class Macs:
    def __init__(self, baud: int = 921600, retry: int = 3,
                 timeout_ms: int = 200, latency_ms: int = 1,
                 array_capacity: int = 1024):
        self._z = _load()
        self._open_param = _OpenUsbParam(baud=baud, retry=retry, timeout=timeout_ms,
                                         flags=0, latency=latency_ms)
        self._h = None
        self._id = None
        self.info: MacsInfo | None = None
        self._buf = (C.c_int32 * USER_PARAM_MAX)()
        self._arr = (C.c_int32 * array_capacity)()
        self._arr_cap = array_capacity
        self._arrsize = C.c_uint32()

    # -------------------------------------------------------------- lifetime
    @property
    def version(self) -> str:
        v = C.create_string_buffer(256)
        self._z.ZbMocGetZbMocVersion(v)
        return v.value.decode(errors="replace")

    def open(self) -> "Macs":
        h = self._z.ZbMocOpenUsb(C.byref(self._open_param))
        if h < 0:
            raise MacsError("ZbMocOpenUsb", h)
        self._h = h

        info = _MoconInfo()
        rc = self._z.ZbMocMoconInfoIdx(h, 0, C.byref(info))
        if rc < 0:
            raise MacsError("ZbMocMoconInfoIdx", rc)
        self._id = info.bus_id

        rc = self._z.ZbMocConnect(h, self._id)
        if rc < 0:
            raise MacsError("ZbMocConnect", rc)

        # >>> RE-READ AFTER CONNECTING. <<<
        # Before ZbMocConnect the struct comes back almost entirely zero -
        # empty name, 0 axes, 0 parameters - which looks exactly like a
        # mis-declared ctypes layout and sends you hunting for the wrong bug.
        self._z.ZbMocMoconInfo(h, self._id, C.byref(info))
        self.info = MacsInfo(
            name          = info.unit_name.decode(errors="replace"),
            bus_id        = info.bus_id,
            firmware      = f"{info.full_version // 10000}."
                            f"{info.full_version // 100 % 100}."
                            f"{info.full_version % 100}",
            axes          = info.number_axis,
            user_params   = info.usercnt,
            global_params = info.globalcnt,
            timebase_ms   = info.timebase,
        )
        return self

    def close(self):
        if self._h is not None:
            self._z.ZbMocClose(self._h)
            self._h = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()
        return False

    def _require_open(self):
        if self._h is None:
            raise MacsError("not connected", -102)

    # ------------------------------------------------------------ parameters
    # Setup and diagnostics only. One round trip PER PARAMETER - see the
    # module docstring before putting these anywhere near a control loop.
    def read_params(self, first: int, last: int) -> list[int]:
        self._require_open()
        # The controller reports how many it has, and asking for one past
        # the end fails the WHOLE call with -372 SubIndexDoesNotExist rather
        # than returning what it could.
        if self.info and last >= self.info.user_params:
            raise ValueError(f"param {last} is past the controller's "
                             f"{self.info.user_params} user parameters "
                             f"(valid 0..{self.info.user_params - 1})")
        rc = self._z.ZbMocUserParamReadRaw(self._h, self._id, self._buf, first, last)
        if rc < 0:
            raise MacsError(f"read_params({first},{last})", rc)
        return list(self._buf[: last - first + 1])

    def read_param(self, n: int) -> int:
        return self.read_params(n, n)[0]

    def write_params(self, first: int, values) -> None:
        self._require_open()
        values = list(values)
        for i, v in enumerate(values):
            self._buf[i] = int(v)
        rc = self._z.ZbMocUserParamWriteRaw(self._h, self._id, self._buf,
                                            first, first + len(values) - 1)
        if rc < 0:
            raise MacsError(f"write_params({first},+{len(values)})", rc)

    def write_param(self, n: int, value: int) -> None:
        self.write_params(n, [value])

    # ---------------------------------------------------------------- arrays
    # The fast path. One round trip for the whole array.
    def read_array(self, number: int, count: int | None = None) -> list[int]:
        """Read DIM array `number`. Arrays are numbered in DECLARATION order
        in the .mc, and exposed as SDO 0x2100, 0x2101, ..."""
        self._require_open()
        want = self._arr_cap if count is None else count
        if want > self._arr_cap:
            raise ValueError(f"count {want} exceeds capacity {self._arr_cap}")
        rc = self._z.ZbMocReadUserArray(self._h, self._id, number, want,
                                        self._arr, C.byref(self._arrsize))
        if rc < 0:
            raise MacsError(f"read_array({number}, {want})", rc)
        return list(self._arr[: min(want, self._arrsize.value)])

    def array_size(self, number: int) -> int:
        """How many values the controller says array `number` holds.
        Zero means it does not exist - that is how you find the last one."""
        self._require_open()
        self._z.ZbMocReadUserArray(self._h, self._id, number, 1,
                                   self._arr, C.byref(self._arrsize))
        return self._arrsize.value

    def write_array(self, number: int, values) -> int:
        """Write a whole DIM array in one transaction. Returns how many the
        controller accepted."""
        self._require_open()
        values = list(values)
        for i, v in enumerate(values):
            self._arr[i] = int(v)
        nr    = C.c_uint16(number)
        size  = C.c_uint32(len(values))
        rsize = C.c_uint32(0)
        rc = self._z.ZbMocWriteUserArray(self._h, self._id, C.byref(nr),
                                         C.byref(size), C.byref(rsize), self._arr)
        if rc < 0:
            raise MacsError(f"write_array({number}, {len(values)})", rc)
        return rsize.value

    # ------------------------------------------------------------------ SDO
    def read_sdo(self, index: int, sub: int) -> int:
        self._require_open()
        v = C.c_int32()
        rc = self._z.ZbMocCanOpenReadSDO(self._h, self._id, index, sub, C.byref(v))
        if rc < 0:
            raise MacsError(f"read_sdo(0x{index:04X},{sub})", rc)
        return v.value

    def write_sdo(self, index: int, sub: int, value: int) -> None:
        self._require_open()
        rc = self._z.ZbMocCanOpenWriteSDO(self._h, self._id, index, sub, int(value))
        if rc < 0:
            raise MacsError(f"write_sdo(0x{index:04X},{sub})", rc)
