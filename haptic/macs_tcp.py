"""
Talking to the MasterMACS over Ethernet with plain sockets - no ZbMoc.dll.

This is the vendor's own TCP protocol, documented and implemented in C++ in
  C:\\Program Files (x86)\\Aposs\\Development\\TCP_SSP\\ZbTcp.cpp
ported here. Port 23. One TCP connection per controller at a time.

WHY THIS AND NOT ZbMoc OVER ETHERNET

ZbMocOpenTcp finds the controller fine, but ZbMocConnect then fails with
-261 USB_CONNECT_FAILED whether or not the USB cable is plugged in. Dead
end. This protocol works instead, and it has the property we actually
wanted: USB is left entirely free, so the ApossIDE keeps its connection
(and its scope) while Python drives the rig over Ethernet.

TWO PROTOCOL VARIANTS

  "short"  STX-framed, fixed 11 bytes, expedited SDO only.
  "long"   ENQ-framed, length-prefixed, and the only one that can do
           SEGMENTED SDO - which is how whole DIM arrays move in a single
           round trip. Needs firmware >= 7.2.29; this rig is 9.5.0.

ADDRESSING

  USER_PARAM(n)   SDO 0x2201 sub n
  DIM array k     SDO 0x2100 + k, sub 0, read/written segmented
                  (arrays are numbered in DECLARATION order in the .mc)
"""
from __future__ import annotations

import socket
import struct
from dataclasses import dataclass

# Frame markers
_STX = 0x02          # short-protocol frame start
_ENQ = 0x05          # long-protocol frame start
_ETX = 0x03          # frame end

# CAN SDO command bytes
_CMD_UPLOAD_INIT     = 0x40   # read; reply 0x43 expedited, 0x41 segmented
_CMD_DOWNLOAD_EXPED  = 0x22   # write one value; reply 0x60
_CMD_DOWNLOAD_SEGINI = 0x21   # start a segmented write; reply 0x60
_CMD_UPLOAD_SEGMENT  = 0x60   # ask for the next read segment (| 0x10 toggle)

_RSP_UPLOAD_EXPED    = 0x43
_RSP_UPLOAD_SEGMENT  = 0x41
_RSP_DOWNLOAD_OK     = 0x60

# SDO channel selector bytes, indexed by SDO number. SDO1 is the default.
_SDO_BYTES = (0x17, 0x18, 0x16, 0x30, 0x31, 0x32, 0x33, 0x34)

SDOINDEX_USER_PARAM  = 0x2201
SDOINDEX_SYS_INFO    = 0x2209

# >>> DIM ARRAYS ARE NOT AT 0x2100 OVER THIS PROTOCOL. <<<
# 0x2100+k with the element as subindex is the Sysvar addressing the .mc
# uses internally, and reading it over TCP quietly returns ELEMENT ZERO as
# a scalar - a wrong answer that looks like a right one.
# The TCP protocol has a selector window instead:
#   sub 1  write the array number to select it
#   sub 2  read its length in longs
#   sub 3  segmented read/write of the contents
#   sub 6  how many DIM arrays the program declares
SDOINDEX_ARRAY_ACCESS = 0x21FF
_ARR_SELECT, _ARR_SIZE, _ARR_DATA, _ARR_COUNT = 1, 2, 3, 6

DEFAULT_PORT = 23


class MacsTcpError(RuntimeError):
    pass


@dataclass
class TcpInfo:
    address: str
    firmware: str
    firmware_raw: int
    long_protocol: bool
    max_telegram: int
    arrays: int
    user_params: int = 100  # APOSS USER_PARAM(0..99)

    def __str__(self):
        return (f"{self.address} fw={self.firmware} "
                f"{'long' if self.long_protocol else 'SHORT ONLY'} "
                f"maxtg={self.max_telegram} dim_arrays={self.arrays}")


def discover_ip(timeout_ms: int = 1500) -> str | None:
    """Find the controller's Ethernet address by broadcast.

    Uses ZbMoc purely for its discovery, then throws the handle away - the
    open and info calls work over Ethernet even though ZbMocConnect does
    not. Worth having because the controller self-assigns a link-local
    169.254.x address, which can change between boots.

    Returns None if nothing answers or ZbMoc is not installed.
    """
    try:
        import ctypes as C
        import os

        import macs as _m
        dll_dir = _m.APOSS_DLL_DIR
        os.add_dll_directory(dll_dir)
        z = C.CDLL(os.path.join(dll_dir, "ZbMoc.dll"))

        class _P(C.Structure):
            _fields_ = [("local", C.c_int32), ("address", C.c_uint32),
                        ("port", C.c_uint32), ("retry", C.c_uint16),
                        ("timeout", C.c_uint32), ("flags", C.c_uint32)]

        z.ZbMocOpenTcp.restype, z.ZbMocOpenTcp.argtypes = C.c_int16, [C.POINTER(_P)]
        z.ZbMocMoconInfoIdx.restype = C.c_int16
        z.ZbMocMoconInfoIdx.argtypes = [C.c_uint16, C.c_uint16, C.POINTER(_m._MoconInfo)]
        z.ZbMocCloseAll.restype = C.c_int16

        p = _P(local=0, address=0, port=0, retry=2, timeout=timeout_ms,
               flags=0x0001 | 0x0008)                       # TCP | SEARCH
        h = z.ZbMocOpenTcp(C.byref(p))
        if h < 0:
            return None
        info = _m._MoconInfo()
        z.ZbMocMoconInfoIdx(h, 0, C.byref(info))
        raw = info.ipaddr
        z.ZbMocCloseAll()
        if not raw:
            return None
        # The ipaddr field is little-endian: 169.254.231.1 reads as 31981225.
        return ".".join(str((raw >> s) & 0xFF) for s in (0, 8, 16, 24))
    except Exception:
        return None


class MacsTcp:
    """Same surface as macs.Macs, so callers can swap transport."""

    # >>> THE ADDRESS IS REQUIRED. THERE IS NO DEFAULT. <<<
    #
    # It used to default to 169.254.231.1, which is what the controller
    # happened to self-assign one afternoon. That is a LINK-LOCAL address:
    # it is handed out by the controller to itself when no DHCP server
    # answers, and it can change between boots. A default like that works
    # until the day it doesn't, and then it fails as a connection timeout
    # that looks like a cabling fault. Use discover_ip(), or pass the
    # address you actually want.
    def __init__(self, address: str, port: int = DEFAULT_PORT,
                 sdo_number: int = 0, timeout_s: float = 2.0):
        self.address = address
        self.port = port
        self._sdo = _SDO_BYTES[sdo_number if 0 <= sdo_number < len(_SDO_BYTES) else 0]
        self._timeout = timeout_s
        self._sock: socket.socket | None = None
        self._long = False
        self._max_tg = 256
        self.info: TcpInfo | None = None

    # -------------------------------------------------------------- lifetime
    def open(self) -> "MacsTcp":
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self._timeout)
        # >>> NAGLE MUST BE OFF. <<<
        # Every exchange here is a small request followed by a small reply,
        # which is precisely the pattern Nagle delays. Left on, round trips
        # jump to tens of milliseconds and the whole point of moving to
        # Ethernet is lost.
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            s.connect((self.address, self.port))
        except OSError as e:
            s.close()
            raise MacsTcpError(f"connect {self.address}:{self.port} failed: {e}") from e
        self._sock = s

        # Firmware decides whether segmented transfer is available. Read it
        # with the SHORT protocol, which every TCP-capable controller speaks.
        self._long = False
        fw = self._read_sdo_short(SDOINDEX_SYS_INFO, 2)
        if fw >= 70229:
            self._long = True
        if fw >= 70323:
            try:
                self._max_tg = self.read_sdo(SDOINDEX_SYS_INFO, 63)
            except MacsTcpError:
                self._max_tg = 256

        self.info = TcpInfo(
            address       = self.address,
            firmware      = f"{fw // 10000}.{fw // 100 % 100}.{fw % 100}",
            firmware_raw  = fw,
            long_protocol = self._long,
            max_telegram  = self._max_tg,
            arrays        = self.array_count(),
        )
        return self

    def close(self):
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
            self._sock = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()
        return False

    # ----------------------------------------------------------------- wire
    def _send(self, frame: bytes):
        if self._sock is None:
            raise MacsTcpError("not connected")
        self._sock.sendall(frame)

    def _recv_exact(self, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise MacsTcpError("connection closed by controller")
            buf += chunk
        return bytes(buf)

    def _recv_frame(self) -> bytes:
        """One whole frame. Length is discovered from the first three bytes.

        A long frame's length field counts itself but excludes ENQ and ETX,
        so the total on the wire is length + 2. PDO frames (long, byte 3 ==
        0x19) are unsolicited and skipped here.
        """
        while True:
            head = self._recv_exact(3)
            if head[0] == _STX:
                total = 11
            elif head[0] == _ENQ:
                total = ((head[2] << 8) | head[1]) + 2
            else:
                raise MacsTcpError(f"bad frame start 0x{head[0]:02X}")
            if total < 9:
                raise MacsTcpError(f"implausible frame length {total}")
            frame = head + self._recv_exact(total - 3)
            if frame[-1] != _ETX:
                raise MacsTcpError(f"frame not ETX-terminated (0x{frame[-1]:02X})")
            if frame[0] == _ENQ and frame[3] == 0x19:
                continue                          # an unsolicited PDO; ignore
            return frame

    # ------------------------------------------------------------ short SDO
    def _read_sdo_short(self, index: int, sub: int) -> int:
        self._send(bytes([_STX, self._sdo, _CMD_UPLOAD_INIT,
                          index & 0xFF, (index >> 8) & 0xFF, sub & 0xFF,
                          0, 0, 0, 0, _ETX]))
        r = self._recv_frame()
        if len(r) != 11:
            raise MacsTcpError(f"short read: expected 11 bytes, got {len(r)}")
        if r[2] != _RSP_UPLOAD_EXPED:
            raise MacsTcpError(f"short read 0x{index:04X}.{sub}: response "
                               f"byte 0x{r[2]:02X}")
        return struct.unpack_from("<i", r, 6)[0]

    # ------------------------------------------------------------- long SDO
    def _long_frame(self, cmd: int, index: int, sub: int, payload: bytes) -> bytes:
        body = bytes([self._sdo, cmd, index & 0xFF, (index >> 8) & 0xFF,
                      sub & 0xFF]) + payload
        length = len(body) + 2                    # the length field counts itself
        return bytes([_ENQ, length & 0xFF, (length >> 8) & 0xFF]) + body + bytes([_ETX])

    def read_sdo(self, index: int, sub: int) -> int:
        """One expedited SDO value."""
        if not self._long:
            return self._read_sdo_short(index, sub)
        self._send(self._long_frame(_CMD_UPLOAD_INIT, index, sub, b"\0\0\0\0"))
        r = self._recv_frame()
        if r[4] == _RSP_UPLOAD_SEGMENT:
            raise MacsTcpError(f"0x{index:04X}.{sub} is a block, not a scalar "
                               f"- use read_block()")
        if r[4] != _RSP_UPLOAD_EXPED:
            raise MacsTcpError(f"read 0x{index:04X}.{sub}: response byte 0x{r[4]:02X}")
        return struct.unpack_from("<i", r, 8)[0]

    def write_sdo(self, index: int, sub: int, value: int) -> None:
        payload = struct.pack("<i", int(value))
        if not self._long:
            self._send(bytes([_STX, self._sdo, _CMD_DOWNLOAD_EXPED,
                              index & 0xFF, (index >> 8) & 0xFF, sub & 0xFF])
                       + payload + bytes([_ETX]))
            r = self._recv_frame()
            ok = r[2] == _RSP_DOWNLOAD_OK
        else:
            self._send(self._long_frame(_CMD_DOWNLOAD_EXPED, index, sub, payload))
            r = self._recv_frame()
            ok = r[4] == _RSP_DOWNLOAD_OK
        if not ok:
            raise MacsTcpError(f"write 0x{index:04X}.{sub}: refused "
                               f"(response 0x{r[4 if self._long else 2]:02X})")

    # -------------------------------------------------------- segmented SDO
    def read_block(self, index: int, sub: int = 0, max_values: int | None = None
                   ) -> list[int]:
        """Read a whole block (a DIM array) in as few round trips as possible.

        The initiate command is the same as an ordinary read; the controller
        decides. Response 0x43 means it was a scalar after all, 0x41 means a
        segmented transfer follows.
        """
        if not self._long:
            raise MacsTcpError("segmented transfer needs the long protocol "
                               "(firmware >= 7.2.29)")
        self._send(self._long_frame(_CMD_UPLOAD_INIT, index, sub, b"\0\0\0\0"))
        r = self._recv_frame()
        if r[4] == _RSP_UPLOAD_EXPED:
            return [struct.unpack_from("<i", r, 8)[0]]
        if r[4] != _RSP_UPLOAD_SEGMENT:
            raise MacsTcpError(f"read_block 0x{index:04X}.{sub}: response byte "
                               f"0x{r[4]:02X} - does the array exist?")

        total_bytes = struct.unpack_from("<I", r, 8)[0]
        want = total_bytes // 4
        if max_values is not None:
            want = min(want, max_values)

        out: list[int] = []
        # Data in the initiate reply starts at 12 and runs to the ETX.
        out += list(struct.unpack_from(f"<{(len(r) - 13) // 4}i", r, 12))

        toggle = False
        while len(out) < want:
            cmd = _CMD_UPLOAD_SEGMENT | (0x10 if toggle else 0x00)
            toggle = not toggle
            self._send(self._long_frame(cmd, index, sub, b"\0\0\0\0"))
            r = self._recv_frame()
            # A continuation reply has no index/subindex: status at 4, data at 5.
            n = (len(r) - 6) // 4
            if n > 0:
                out += list(struct.unpack_from(f"<{n}i", r, 5))
            if r[4] & 0x01:                       # controller says that was the last
                break
        return out[:want]

    def write_block(self, index: int, values, sub: int = 0) -> None:
        """Write a whole block in one telegram where it fits."""
        if not self._long:
            raise MacsTcpError("segmented transfer needs the long protocol")
        values = [int(v) for v in values]
        payload = struct.pack("<I", len(values) * 4) + struct.pack(
            f"<{len(values)}i", *values)
        frame = self._long_frame(_CMD_DOWNLOAD_SEGINI, index, sub, payload)
        if len(frame) > self._max_tg:
            raise MacsTcpError(f"{len(values)} values need {len(frame)} bytes, "
                               f"over the controller's {self._max_tg}-byte telegram "
                               f"limit - split the array")
        self._send(frame)
        r = self._recv_frame()
        if r[4] != _RSP_DOWNLOAD_OK:
            raise MacsTcpError(f"write_block 0x{index:04X}.{sub}: response byte "
                               f"0x{r[4]:02X}")

    # --------------------------------------------- the macs.Macs-shaped API
    def read_param(self, n: int) -> int:
        return self.read_sdo(SDOINDEX_USER_PARAM, n)

    def read_params(self, first: int, last: int) -> list[int]:
        """One round trip PER PARAMETER - there is no block form for these.
        Use an array for anything on a control loop."""
        return [self.read_sdo(SDOINDEX_USER_PARAM, n) for n in range(first, last + 1)]

    def write_param(self, n: int, value: int) -> None:
        self.write_sdo(SDOINDEX_USER_PARAM, n, value)

    def write_params(self, first: int, values) -> None:
        for i, v in enumerate(values):
            self.write_sdo(SDOINDEX_USER_PARAM, first + i, v)

    def array_count(self) -> int:
        """How many DIM arrays the loaded program declares."""
        try:
            return self.read_sdo(SDOINDEX_ARRAY_ACCESS, _ARR_COUNT)
        except MacsTcpError:
            return 0

    def _select_array(self, number: int) -> int:
        """Point the selector window at one array and return its length."""
        self.write_sdo(SDOINDEX_ARRAY_ACCESS, _ARR_SELECT, number)
        return self.read_sdo(SDOINDEX_ARRAY_ACCESS, _ARR_SIZE)

    def array_size(self, number: int) -> int:
        """Values in DIM array `number`, or 0 if it does not exist."""
        if number >= self.array_count():
            return 0
        try:
            return self._select_array(number)
        except MacsTcpError:
            return 0

    def read_array(self, number: int, count: int | None = None) -> list[int]:
        size = self._select_array(number)
        if size <= 0:
            raise MacsTcpError(f"array {number} reports length {size}")
        want = size if count is None else min(size, count)
        return self.read_block(SDOINDEX_ARRAY_ACCESS, _ARR_DATA, want)

    def write_array(self, number: int, values) -> int:
        values = [int(v) for v in values]
        size = self._select_array(number)
        if len(values) != size:
            raise MacsTcpError(f"array {number} holds {size} values, given "
                               f"{len(values)} - a partial write is refused")
        self.write_block(SDOINDEX_ARRAY_ACCESS, values, _ARR_DATA)
        return len(values)

    @property
    def version(self) -> str:
        return f"direct TCP to {self.address}:{self.port} (no ZbMoc)"
