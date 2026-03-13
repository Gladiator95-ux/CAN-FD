"""
can_fd.py — SAMC21 CAN FD Gateway
===================================
Standalone — no external project imports required.

Sends cyclic CAN FD messages via the SAMC21 gateway onto the bus.
Prints and logs all frames received from the bus (any node).
The second node on the bus can be anything — PCAN, Vector, ECU, etc.

Usage:
    python can_fd.py
    python can_fd.py --port COM4
    python can_fd.py --port COM4 --duration 30

Requirements:
    pip install python-can pyserial
"""

import sys
import time
import threading
import argparse
import queue
import collections
from datetime import datetime
from typing import Optional

import serial
import serial.tools.list_ports
import can
from can import Message, BusABC


# ===========================================================================
# SAMC21 interface — embedded, identical to samc21_interface.py v3
# ===========================================================================

_ECHO_TTL_S = 0.100   # 100 ms TTL


class _EchoFilter:
    """
    Counter-based, full-frame echo filter with O(1) amortised expiry.

    Key = (arbitration_id, bytes(data), is_extended_id, is_fd, bitrate_switch)

    Counter (not flag) correctly handles N in-flight copies of the same frame —
    critical when send_periodic fires faster than the echo round-trip.
    Deque is insertion-ordered so oldest entry always at front — O(1) drain.
    """
    def __init__(self):
        self._lock   = threading.Lock()
        self._counts: dict = {}
        self._expiry: collections.deque = collections.deque()

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
            while self._expiry and self._expiry[0][1] < now:
                expired_key, _ = self._expiry.popleft()
                if expired_key in self._counts:
                    self._counts[expired_key] -= 1
                    if self._counts[expired_key] <= 0:
                        del self._counts[expired_key]
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


def detect_samc21_port() -> Optional[str]:
    """Scan COM ports and return the first one that responds to 'V\\r'."""
    print("[AUTO-DETECT] Scanning for SAMC21...")
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[AUTO-DETECT] No COM ports found.")
        return None

    print(f"[AUTO-DETECT] Found {len(ports)} port(s):")
    for p in ports:
        print(f"  {p.device}: {p.description}")

    for p in ports:
        if any(tag in p.description for tag in ('Bluetooth', 'BT')):
            continue
        print(f"[AUTO-DETECT] Testing {p.device}...")
        try:
            ser = serial.Serial(port=p.device, baudrate=460800, timeout=0.5)
            time.sleep(0.3)
            ser.write(b'C\r')
            time.sleep(0.1)
            ser.reset_input_buffer()
            ser.write(b'V\r')
            time.sleep(0.2)
            resp = ser.read(100).decode('ascii', errors='ignore').strip()
            ser.close()
            if resp and ('SAMC21' in resp or 'CAN' in resp or resp.startswith('V')):
                print(f"[AUTO-DETECT] ✓ Found SAMC21 at {p.device}  ({resp[:40]})")
                return p.device
            print(f"[AUTO-DETECT]   Not SAMC21 (response: '{resp[:30]}')")
        except (serial.SerialException, OSError) as e:
            print(f"[AUTO-DETECT]   Cannot open {p.device}: {e}")
        except Exception as e:
            print(f"[AUTO-DETECT]   Error on {p.device}: {e}")

    print("[AUTO-DETECT] ❌ SAMC21 not found on any port")
    return None


class SAMC21Bus(BusABC):
    """
    python-can BusABC for the SAMC21 CAN FD gateway.
    Baud rate must match firmware: 460800.
    """

    def __init__(self,
                 channel:     Optional[str] = None,
                 baudrate:    int           = 460800,
                 auto_detect: bool          = True,
                 **kwargs):

        if channel is None or channel == 'auto':
            if auto_detect:
                channel = detect_samc21_port()
                if channel is None:
                    raise can.CanError(
                        "SAMC21 auto-detection failed. "
                        "Specify the COM port with --port COM<N>."
                    )
            else:
                raise can.CanError("No COM port specified and auto_detect is disabled.")

        super().__init__(channel=channel, **kwargs)

        self.channel_info      = f"SAMC21 on {channel}"
        self.port              = channel
        self.baudrate          = baudrate
        self.serial            = None
        self.running           = False
        self.rx_thread         = None
        self.rx_queue          = queue.Queue(maxsize=2000)
        self._echo             = _EchoFilter()
        self._tx_lock          = threading.Lock()
        self.tx_count          = 0
        self.rx_count          = 0
        self.rx_echo_filtered  = 0
        self.rx_corrupt        = 0

        self._connect()

    def _connect(self) -> None:
        print(f"[SAMC21] Connecting to {self.port} at {self.baudrate} baud...")
        self.serial = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.1)
        time.sleep(0.3)

        self.serial.write(b'C\r')
        time.sleep(0.1)
        self.serial.reset_input_buffer()

        self.serial.write(b'V\r')
        time.sleep(0.2)
        resp = self.serial.read(100).decode('ascii', errors='ignore').strip()
        if not resp:
            raise can.CanError(
                f"No version response from firmware on {self.port}. "
                "Check cable and firmware."
            )
        print(f"[SAMC21] ✓ Firmware: {resp}")

        self.serial.write(b'O\r')
        time.sleep(0.2)
        self.serial.reset_input_buffer()

        self.running   = True
        self.rx_thread = threading.Thread(
            target=self._rx_worker, daemon=True, name="samc21-rx"
        )
        self.rx_thread.start()
        print(f"[SAMC21] ✓ Channel open")

    def _rx_worker(self) -> None:
        buf = b""
        while self.running:
            try:
                chunk = self.serial.read(4096)
                if not chunk:
                    continue
                buf += chunk
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
                        pass
            except Exception as e:
                if self.running:
                    print(f"[SAMC21] RX error: {e}")

    def _parse_frame(self, raw: bytes) -> Optional[Message]:
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
            arb_id  = int(line[pos:pos + id_dig], 16)
            pos    += id_dig
            if is_fd:
                num_bytes = int(line[pos:pos + 2], 16)
                pos += 2
            else:
                num_bytes = int(line[pos], 16)
                pos += 1
            if is_fd  and num_bytes > 64: return None
            if not is_fd and num_bytes > 8:  return None
            hex_d = line[pos:pos + num_bytes * 2]
            if len(hex_d) != num_bytes * 2:  return None
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

    def _format_frame(self, msg: Message) -> str:
        if msg.is_fd:
            t = ('B' if msg.bitrate_switch else 'D') if msg.is_extended_id \
                else ('b' if msg.bitrate_switch else 'd')
            n     = min(msg.dlc, len(msg.data))
            data  = bytes(msg.data)[:n]
            len_s = f"{n:02X}"
        else:
            t     = 'T' if msg.is_extended_id else 't'
            n     = min(msg.dlc, len(msg.data), 8)
            data  = bytes(msg.data)[:n]
            len_s = f"{n:X}"
        id_s = (f"{msg.arbitration_id:08X}" if msg.is_extended_id
                else f"{msg.arbitration_id:03X}")
        return f"{t}{id_s}{len_s}{data.hex().upper()}\r"

    def send(self, msg: Message, timeout=None) -> None:
        frame = self._format_frame(msg).encode('ascii')
        self._echo.add(msg)
        with self._tx_lock:
            self.serial.write(frame)
        self.tx_count += 1

    def recv(self, timeout: Optional[float] = None) -> Optional[Message]:
        try:
            return self.rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None

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

    def fileno(self) -> int:
        return -1


# ===========================================================================
# Gateway
# ===========================================================================

class SAMC21Gateway:
    """
    CAN FD gateway over SAMC21.
    Sends cyclic messages onto the bus and prints/logs everything received.
    The second node on the bus is irrelevant — any CAN node will do.
    """

    def __init__(self, port: Optional[str] = None):
        self.port          = port
        self.bus:          Optional[SAMC21Bus]    = None
        self.cyclic_tasks: list                   = []
        self.notifier:     Optional[can.Notifier] = None
        self.logger:       Optional[can.Logger]   = None
        self.running       = False

    # ------------------------------------------------------------------
    def connect(self) -> bool:
        try:
            print("\n" + "="*80)
            print("  Connecting to SAMC21...")
            print("="*80)
            self.bus = SAMC21Bus(
                channel     = self.port,
                baudrate    = 460800,
                auto_detect = self.port is None,
            )
            print("✓ Connected.\n")
            return True
        except Exception as e:
            print(f"\n❌ Connection failed: {e}")
            return False

    # ------------------------------------------------------------------
    def add_cyclic_message(self,
                           msg_id:   int,
                           data:     list,
                           period:   float,
                           extended: bool = True,
                           use_brs:  bool = True,
                           name:     str  = None) -> None:
        """Add a cyclic CAN FD message transmitted via SAMC21."""
        msg = Message(
            arbitration_id = msg_id,
            data           = bytes(data),
            is_extended_id = extended,
            is_fd          = True,
            bitrate_switch = use_brs,
        )
        task  = self.bus.send_periodic(msg, period=period)
        label = name or (f"0x{msg_id:08X}" if extended else f"0x{msg_id:03X}")
        self.cyclic_tasks.append({'task': task, 'name': label})
        print(f"  ✓ Cyclic: {label:<20s}  period={period*1000:.0f}ms  "
              f"size={len(data)}B  BRS={use_brs}")

    # ------------------------------------------------------------------
    def _on_message_received(self, msg: Message) -> None:
        """
        Notifier callback — fires for every frame received from the bus.
        Echo filter in SAMC21Bus already suppresses our own TX frames,
        so only frames from other nodes reach here.
        """
        ts     = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        id_str = (f"0x{msg.arbitration_id:08X}" if msg.is_extended_id
                  else f"0x{msg.arbitration_id:03X}")
        print(f"[{ts}] RX  ID={id_str}  "
              f"Len={len(msg.data):<3d}  "
              f"FD={int(msg.is_fd)}  BRS={int(bool(msg.bitrate_switch))}  "
              f"Data={msg.data.hex().upper()}")

    # ------------------------------------------------------------------
    def start_logging(self, filename: Optional[str] = None) -> Optional[str]:
        """Attach .asc logger and RX print callback to the SAMC21 bus."""
        if filename is None:
            filename = f"can_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.asc"
        try:
            self.logger   = can.Logger(filename)
            self.notifier = can.Notifier(
                self.bus,
                [self.logger, self._on_message_received],
            )
            print(f"  ✓ Logging to: {filename}")
            return filename
        except Exception as e:
            print(f"  ⚠️  Logger failed: {e}")
            return None

    # ------------------------------------------------------------------
    def run(self, duration: Optional[float] = None) -> None:
        """Run until Ctrl+C or duration seconds."""
        self.running = True
        print("\n" + "="*80)
        print(f"  Gateway Running — {len(self.cyclic_tasks)} cyclic task(s)")
        print(f"  Press Ctrl+C to stop")
        print("="*80 + "\n")

        start = time.time()
        try:
            if duration is not None:
                time.sleep(duration)
            else:
                while self.running:
                    time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n⚠️  Shutdown requested...")
        finally:
            self._shutdown(elapsed=time.time() - start)

    # ------------------------------------------------------------------
    def _shutdown(self, elapsed: float = 0.0) -> None:
        print("\n" + "="*80)
        print("  Shutting Down")
        print("="*80)

        print(f"  Stopping {len(self.cyclic_tasks)} cyclic task(s)...")
        for t in self.cyclic_tasks:
            try:
                t['task'].stop()
            except Exception:
                pass
        print("  ✓ Cyclic tasks stopped")

        if self.notifier:
            try:
                self.notifier.stop()
            except Exception:
                pass
            print("  ✓ Notifier stopped")

        if self.logger:
            try:
                self.logger.stop()
            except Exception:
                pass
            print("  ✓ Log file closed")

        if self.bus and elapsed > 0:
            b = self.bus
            print(f"\n  Statistics ({elapsed:.1f}s runtime):")
            print(f"    TX : {b.tx_count}  ({b.tx_count / elapsed:.1f} msg/s)")
            print(f"    RX : {b.rx_count}  ({b.rx_count / elapsed:.1f} msg/s)")
            print(f"    Echo filtered : {b.rx_echo_filtered}")
            print(f"    Corrupt       : {b.rx_corrupt}")

        if self.bus:
            self.bus.shutdown()

        print("\n" + "="*80)
        print("  ✓ Shutdown complete")
        print("="*80 + "\n")


# ===========================================================================
# Main
# ===========================================================================

def main():
    parser = argparse.ArgumentParser(
        description="SAMC21 CAN FD Gateway",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python can_fd.py                         # auto-detect SAMC21 port
  python can_fd.py --port COM4
  python can_fd.py --port COM4 --duration 30
        """
    )
    parser.add_argument('--port',     default=None,
                        help="SAMC21 COM port (e.g. COM4). Omit to auto-detect.")
    parser.add_argument('--duration', type=float, default=None,
                        help="Run for N seconds then exit (default: run until Ctrl+C)")
    args = parser.parse_args()

    print("="*80)
    print("  SAMC21 CAN FD Gateway")
    print("="*80)

    gw = SAMC21Gateway(port=args.port)

    if not gw.connect():
        sys.exit(1)

    # ------------------------------------------------------------------
    # Cyclic messages — add/edit here
    # ------------------------------------------------------------------
    print("Configuring cyclic messages...")

    gw.add_cyclic_message(
        msg_id   = 0x12DD54FE,
        data     = [0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00],
        period   = 0.100,
        extended = True,
        use_brs  = True,
        name     = "WAKEUP_SIM",
    )

    gw.add_cyclic_message(
        msg_id   = 0x1C44001D,
        data     = [0x03, 0x22, 0xF1, 0x8C, 0x00, 0x00, 0x00, 0x00],
        period   = 1.000,
        extended = True,
        use_brs  = True,
        name     = "DIAG_REQUEST",
    )

    # ------------------------------------------------------------------
    print("\nStarting logger...")
    gw.start_logging()

    gw.run(duration=args.duration)


if __name__ == '__main__':
    main()