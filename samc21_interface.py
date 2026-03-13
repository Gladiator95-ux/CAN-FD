"""
samc21_interface.py
Custom python-can interface for SAMC21 CAN FD — v3 internals
=============================================================

Drop-in replacement for the previous samc21_interface.py.
Public API is unchanged so initialize_pcan.py requires no edits:

    from .samc21_interface import SAMC21Bus
    SAMC21Bus(channel=None,  baudrate=460800, auto_detect=True)
    SAMC21Bus(channel='COM4', baudrate=460800, auto_detect=False)

What changed internally (v2 → v3):
  - Echo filter upgraded from an ID-only set to a counter-based,
    full-frame-key (id + data + flags) filter with 100 ms TTL.
    The old set caused corrupt frames to bleed through whenever
    the same ID was sent with different data, or when the echo
    arrived after a period boundary.
  - send() no longer calls time.sleep(0.002). That 2 ms busy-wait
    blocked the calling thread on every TX and caused jitter on
    cyclic tasks. The SLCAN write is fire-and-forget; the firmware
    DMA TX handles pacing.
  - _rx_worker reads 4096-byte chunks instead of 1024, reducing
    the number of serial.read() syscalls under heavy CAN FD load
    (64-byte frames at 10 ms cycles produce ~140-char SLCAN lines).
  - SAMC21Gateway and main() removed — not used by the application.
"""

import can
import time
import serial
import serial.tools.list_ports
import threading
import queue
import collections
from can import Message, BusABC


# ---------------------------------------------------------------------------
# Auto-detection
# ---------------------------------------------------------------------------

def detect_samc21_port() -> str | None:
    """
    Scan all available COM ports and return the first one that responds
    to the SLCAN 'V\\r' version query with a recognisable SAMC21 reply.

    Returns:
        str  — COM port name (e.g. 'COM4') if found
        None — if no SAMC21 device is detected
    """
    print("[SAMC21 AUTO-DETECT] Scanning for SAMC21 device...")

    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[SAMC21 AUTO-DETECT] No COM ports found!")
        return None

    print(f"[SAMC21 AUTO-DETECT] Found {len(ports)} COM port(s):")
    for p in ports:
        print(f"  - {p.device}: {p.description}")

    for port_info in ports:
        port_name = port_info.device

        # Skip Bluetooth / virtual ports that will never be SAMC21
        if any(tag in port_info.description for tag in ('Bluetooth', 'BT')):
            print(f"[SAMC21 AUTO-DETECT] Skipping Bluetooth port: {port_name}")
            continue

        print(f"[SAMC21 AUTO-DETECT] Testing {port_name}...")

        try:
            ser = serial.Serial(port=port_name, baudrate=460800, timeout=0.5)
            time.sleep(0.3)

            ser.write(b'C\r')   # close any open session first
            time.sleep(0.1)
            ser.reset_input_buffer()

            ser.write(b'V\r')
            time.sleep(0.2)
            response = ser.read(100).decode('ascii', errors='ignore').strip()
            ser.close()

            if response and (
                'SAMC21' in response
                or 'CAN'  in response
                or response.startswith('V')
            ):
                print(f"[SAMC21 AUTO-DETECT] ✓ Found SAMC21 at {port_name}")
                print(f"[SAMC21 AUTO-DETECT]   Version: {response}")
                return port_name

            print(f"[SAMC21 AUTO-DETECT]   Not SAMC21 "
                  f"(response: '{response[:30]}')")

        except (serial.SerialException, OSError) as e:
            print(f"[SAMC21 AUTO-DETECT]   Cannot open {port_name}: {e}")
        except Exception as e:
            print(f"[SAMC21 AUTO-DETECT]   Error on {port_name}: {e}")

    print("[SAMC21 AUTO-DETECT] ❌ SAMC21 not found on any port")
    return None


# ---------------------------------------------------------------------------
# Echo filter
# ---------------------------------------------------------------------------

_ECHO_TTL_S  = 0.100   # 100 ms — covers worst-case bus arbitration delay

class _EchoFilter:
    """
    Counter-based, full-frame echo filter with O(1) amortised expiry.

    Key = (arbitration_id, bytes(data), is_extended_id, is_fd, bitrate_switch)

    Design:
      _counts  dict  key → pending echo count
      _expiry  deque (key, expiry_time) in insertion order

    add():     increment counter, append to deque tail.
    is_echo(): drain expired entries from deque head (O(1) amortised),
               then check counter. Under high load (500 fr/s) the previous
               O(N) full-scan purge added measurable latency to the RX
               worker thread; the deque head-drain is effectively free.

    Counter (not flag) handles N in-flight copies of the same frame
    correctly — important when send_periodic fires faster than the echo
    round-trip under bus congestion.
    """

    def __init__(self):
        self._lock   = threading.Lock()
        self._counts: dict[tuple, int]                          = {}
        self._expiry: collections.deque[tuple[tuple, float]]    = collections.deque()

    def add(self, msg: Message) -> None:
        key    = self._key(msg)
        expiry = time.monotonic() + _ECHO_TTL_S
        with self._lock:
            self._counts[key] = self._counts.get(key, 0) + 1
            self._expiry.append((key, expiry))

    def is_echo(self, msg: Message) -> bool:
        key = self._key(msg)
        now = time.monotonic()
        with self._lock:
            # Drain expired entries from the front of the deque — O(1) each
            while self._expiry and self._expiry[0][1] < now:
                expired_key, _ = self._expiry.popleft()
                if expired_key in self._counts:
                    self._counts[expired_key] -= 1
                    if self._counts[expired_key] <= 0:
                        del self._counts[expired_key]
            # Check for match
            if self._counts.get(key, 0) > 0:
                self._counts[key] -= 1
                if self._counts[key] <= 0:
                    del self._counts[key]
                return True
        return False

    @staticmethod
    def _key(msg: Message) -> tuple:
        return (
            msg.arbitration_id,
            bytes(msg.data),
            bool(msg.is_extended_id),
            bool(msg.is_fd),
            bool(msg.bitrate_switch),
        )


# ---------------------------------------------------------------------------
# SAMC21Bus — python-can BusABC implementation
# ---------------------------------------------------------------------------

class SAMC21Bus(BusABC):
    """
    python-can BusABC implementation for the SAMC21 CAN FD gateway.

    Compatible with python-can's Notifier, send_periodic, and all other
    higher-level APIs. Registered in the application via app_globals.PCAN_HANDLE.

    Args:
        channel (str | None): COM port, e.g. 'COM4'.
                              If None or 'auto', auto-detection is attempted
                              when auto_detect=True.
        baudrate (int):       Serial baud rate. Must match firmware (460800).
        auto_detect (bool):   Scan available ports when channel is not given.
    """

    def __init__(self,
                 channel:     str | None = None,
                 baudrate:    int        = 460800,
                 auto_detect: bool       = True,
                 **kwargs):

        # Resolve COM port
        if channel is None or channel == 'auto':
            if auto_detect:
                channel = detect_samc21_port()
                if channel is None:
                    raise can.CanError(
                        "SAMC21 auto-detection failed. "
                        "Please specify the COM port manually."
                    )
            else:
                raise can.CanError(
                    "No COM port specified and auto_detect is disabled."
                )

        super().__init__(channel=channel, **kwargs)

        self.channel_info = f"SAMC21 on {channel}"
        self.port         = channel
        self.baudrate     = baudrate
        self.serial       = None
        self.running      = False
        self.rx_thread    = None
        self.rx_queue     = queue.Queue(maxsize=2000)
        self._echo        = _EchoFilter()
        # Serialise concurrent serial.write() calls.
        # python-can send_periodic spawns one thread per cyclic task;
        # without this lock, two threads writing simultaneously produce
        # interleaved SLCAN bytes → firmware receives garbled commands
        # → corrupt CAN frames appear on the bus.
        self._tx_lock     = threading.Lock()

        # Public statistics (read by application diagnostics if needed)
        self.tx_count          = 0
        self.rx_count          = 0
        self.rx_echo_filtered  = 0
        self.rx_corrupt        = 0

        self._connect()

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def _connect(self) -> None:
        print(f"[SAMC21] Connecting to {self.port} at {self.baudrate} baud...")

        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=0.1,
        )
        time.sleep(0.3)

        # Close any previously open CAN session in the firmware
        self.serial.write(b'C\r')
        time.sleep(0.1)
        self.serial.reset_input_buffer()

        # Version handshake
        self.serial.write(b'V\r')
        time.sleep(0.2)
        resp = self.serial.read(100).decode('ascii', errors='ignore').strip()
        if not resp:
            raise can.CanError(
                f"No version response from firmware on {self.port}. "
                "Check cable and firmware."
            )
        print(f"[SAMC21] ✓ Firmware: {resp}")

        # Open CAN channel
        self.serial.write(b'O\r')
        time.sleep(0.2)
        self.serial.reset_input_buffer()

        # Start background RX thread
        self.running   = True
        self.rx_thread = threading.Thread(
            target=self._rx_worker, daemon=True, name="samc21-rx"
        )
        self.rx_thread.start()
        print(f"[SAMC21] ✓ Channel open")

    # ------------------------------------------------------------------
    # RX worker
    # ------------------------------------------------------------------

    def _rx_worker(self) -> None:
        """
        Background thread: reads raw bytes from serial, splits on '\\r',
        parses SLCAN frames, applies the echo filter, and enqueues valid
        externally-originated frames for recv().
        """
        buf = b""

        while self.running:
            try:
                chunk = self.serial.read(4096)
                if not chunk:
                    continue

                buf += chunk

                # Safety valve — prevent unbounded accumulation
                if len(buf) > 65536:
                    buf = buf[-4096:]

                while b'\r' in buf:
                    raw, buf = buf.split(b'\r', 1)
                    if not raw:
                        continue

                    msg = self._parse_frame(raw)
                    if msg is None:
                        self.rx_corrupt += 1
                        continue

                    if self._echo.is_echo(msg):
                        self.rx_echo_filtered += 1
                        continue

                    self.rx_count += 1
                    try:
                        self.rx_queue.put_nowait(msg)
                    except queue.Full:
                        pass   # Drop oldest implicitly; queue has 2000-frame headroom

            except Exception as e:
                if self.running:
                    print(f"[SAMC21] RX error: {e}")

    # ------------------------------------------------------------------
    # SLCAN frame parser
    # ------------------------------------------------------------------

    def _parse_frame(self, raw: bytes) -> Message | None:
        """
        Parse one SLCAN frame (without the trailing '\\r') into a
        python-can Message. Returns None on any parse error.

        SLCAN format produced by SAMC21 firmware:
            CAN FD extended BRS : B<8-hex-id><2-hex-len><data>
            CAN FD extended     : D<8-hex-id><2-hex-len><data>
            CAN FD standard BRS : b<3-hex-id><2-hex-len><data>
            CAN FD standard     : d<3-hex-id><2-hex-len><data>
            Classic extended    : T<8-hex-id><1-hex-dlc><data>
            Classic standard    : t<3-hex-id><1-hex-dlc><data>
        """
        try:
            line = raw.decode('ascii', errors='strict').strip()
        except UnicodeDecodeError:
            return None

        if not line or line[0] not in 'bBdDtTrR':
            return None

        try:
            t      = line[0]
            is_fd  = t in 'bBdD'
            is_ext = t in 'BDTR'
            brs    = t in 'bB'
            id_dig = 8 if is_ext else 3
            pos    = 1

            if len(line) < pos + id_dig + (2 if is_fd else 1):
                return None

            arb_id  = int(line[pos : pos + id_dig], 16)
            pos    += id_dig

            if is_fd:
                num_bytes = int(line[pos : pos + 2], 16)
                pos += 2
            else:
                num_bytes = int(line[pos], 16)
                pos += 1

            if is_fd  and num_bytes > 64:
                return None
            if not is_fd and num_bytes > 8:
                return None

            hex_d = line[pos : pos + num_bytes * 2]
            if len(hex_d) != num_bytes * 2:
                return None
            if not all(c in '0123456789ABCDEFabcdef' for c in hex_d):
                return None

            return Message(
                arbitration_id = arb_id,
                data           = bytes.fromhex(hex_d) if hex_d else b'',
                is_extended_id = is_ext,
                is_fd          = is_fd,
                bitrate_switch = brs,
                timestamp      = time.time(),
            )

        except (ValueError, IndexError):
            return None

    # ------------------------------------------------------------------
    # Send
    # ------------------------------------------------------------------

    def send(self, msg: Message, timeout=None) -> None:
        """
        Format msg as an SLCAN frame and write it to the firmware.

        _tx_lock serialises concurrent callers: python-can send_periodic
        spawns one thread per cyclic task, all calling send() on the same
        bus object. pyserial write() is not thread-safe — concurrent writes
        interleave bytes and produce garbled SLCAN commands at the firmware.

        The echo filter entry is registered BEFORE the serial write so
        the filter is armed before the physical echo can arrive back
        (~0.3 ms round-trip at 500 kbit/s + 460800 baud UART).

        No sleep — the firmware DMA TX handles pacing autonomously.
        """
        frame = self._format_frame(msg).encode('ascii')
        self._echo.add(msg)
        with self._tx_lock:
            self.serial.write(frame)
        self.tx_count += 1



    # python-can stores msg.dlc as the BYTE COUNT, not the ISO DLC integer.
    #   can.Message(data=bytes(64))         → msg.dlc = 64
    #   can.Message(data=bytes(8))          → msg.dlc = 8
    #   can.Message(data=bytes(64), dlc=8)  → msg.dlc = 8  (kwarg honoured)
    #
    # _format_frame therefore uses msg.dlc directly as the number of bytes
    # to encode. min(msg.dlc, len(msg.data)) guards against dlc > actual data.
    #
    # This correctly handles every real-world case:
    #   Simulation  data=bytes([0x3F]*8+[0x00]*56), dlc=8  → encodes 8 bytes  ✓
    #   Wakeup      data=[0x00,0x00,0x02,...], no dlc       → encodes 8 bytes  ✓
    #   All-zero    data=bytes(8), no dlc                   → encodes 8 bytes  ✓
    #   FC frame    data=[0x30,0,0,0,0,0,0,0], no dlc       → encodes 8 bytes  ✓
    #   Genuine64   data=bytes(64), no dlc                  → encodes 64 bytes ✓

    def _format_frame(self, msg: Message) -> str:
        """
        Encode a python-can Message into an SLCAN frame string (\r-terminated).

        Uses msg.dlc — which python-can stores as the byte count — to determine
        how many bytes of msg.data to put on the wire.  This is the only correct
        approach: it respects an explicit dlc= kwarg from the application layer
        (e.g. simulation code that pads data to 64 bytes but passes dlc=8) while
        also working correctly for all-zero and variably-sized payloads.
        """
        if msg.is_fd:
            t = ('B' if msg.bitrate_switch else 'D') if msg.is_extended_id                 else ('b' if msg.bitrate_switch else 'd')
            n     = min(msg.dlc, len(msg.data))          # bytes to encode
            data  = bytes(msg.data)[:n]
            len_s = f"{n:02X}"                           # 2-digit hex byte count
        else:
            t     = 'T' if msg.is_extended_id else 't'
            n     = min(msg.dlc, len(msg.data), 8)
            data  = bytes(msg.data)[:n]
            len_s = f"{n:X}"                             # 1-digit hex DLC

        id_s = (f"{msg.arbitration_id:08X}" if msg.is_extended_id
                else f"{msg.arbitration_id:03X}")

        return f"{t}{id_s}{len_s}{data.hex().upper()}\r"

    # ------------------------------------------------------------------
    # Recv
    # ------------------------------------------------------------------

    def recv(self, timeout: float | None = None) -> Message | None:
        try:
            return self.rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def shutdown(self) -> None:
        self.running = False

        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=2.0)

        if self.serial and self.serial.is_open:
            try:
                self.serial.write(b'C\r')
                time.sleep(0.1)
            except Exception:
                pass
            self.serial.close()

        print(f"[SAMC21] Connection closed  "
              f"(TX={self.tx_count}  RX={self.rx_count}  "
              f"echo_filtered={self.rx_echo_filtered}  "
              f"corrupt={self.rx_corrupt})")

    # ------------------------------------------------------------------
    # Misc
    # ------------------------------------------------------------------

    def fileno(self) -> int:
        return -1

    def __str__(self) -> str:
        return self.channel_info
