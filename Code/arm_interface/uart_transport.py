"""Single UART writer, latest-command slot, bounded reads, 20 Hz hold heartbeat."""

from dataclasses import dataclass, replace
from queue import Empty, Full, Queue
import re
from threading import Event, Lock, Thread
from time import monotonic
from kinematics import JOINT_LIMITS


def validate_packet(packet: str) -> bytes:
    if not re.fullmatch(r"\d{1,3},\d{1,3},\d{1,3}\n", packet, flags=re.ASCII):
        raise ValueError("UART needs exactly three ASCII integer angles and a newline.")
    values = [int(value) for value in packet.strip().split(",")]
    if any(not lower <= value <= upper for value, (lower, upper) in zip(values, JOINT_LIMITS)):
        raise ValueError("UART command exceeds the physical joint limits.")
    return packet.encode("ascii")


@dataclass
class Status:
    state: str = "offline"
    sent: int = 0
    acknowledged: int = 0
    last_ack_time: float = 0.0
    streaming_since: float = 0.0
    last_ack: str = ""
    last_sent: str = ""
    completed: int = 0
    last_done: str = ""
    last_done_time: float = 0.0
    error: str = ""


class SerialStreamer:
    def __init__(self, factory=None, period=0.05, startup_delay=2.0):
        self._factory = factory or self._open_serial
        self.period = period
        self.startup_delay = startup_delay
        self._lock = Lock()
        self._packet = None
        self._status = Status()
        self._stop = Event()
        self._thread = None
        self.events = Queue(maxsize=100)

    @staticmethod
    def _open_serial(port, baud):
        import serial
        # Opening some USB-UART boards resets the ESP32; startup_delay handles it.
        return serial.Serial(port, baudrate=baud, timeout=0, write_timeout=0.2,
                             rtscts=False, dsrdtr=False)

    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive()

    def status(self):
        with self._lock:
            return replace(self._status)

    def publish(self, packet):
        payload = validate_packet(packet)
        with self._lock:
            self._packet = payload  # Replace, never queue stale slider positions.

    def start(self, port, baud=115200):
        if self.running:
            raise ValueError("Disconnect before opening another UART connection.")
        if not port or port in ("Select a port", "No ports found"):
            raise ValueError("Select the ESP32 COM port first.")
        if int(baud) != 115200:
            raise ValueError("This ESP32 firmware requires 115200 baud.")
        with self._lock:
            if self._packet is None:
                raise ValueError("No validated joint command is available.")
            self._status = Status(state="opening")
        self._stop.clear()
        self._thread = Thread(target=self._run, args=(port, int(baud)), daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)

    def _event(self, text):
        try:
            self.events.put_nowait(text)
        except Full:
            pass

    def _line(self, line):
        text = line.decode("ascii", errors="replace")
        if text.startswith(("OK,", "DONE,")):
            prefix = 3 if text.startswith("OK,") else 5
            try:
                validate_packet(text[prefix:]+"\n")
            except ValueError:
                return
            with self._lock:
                if prefix == 3:
                    self._status.acknowledged += 1
                    self._status.last_ack = text[prefix:]
                    self._status.last_ack_time = monotonic()
                else:
                    self._status.completed += 1
                    self._status.last_done = text[prefix:]
                    self._status.last_done_time = monotonic()
        elif text.startswith(("READY,", "ERR,", "HOLD,")):
            self._event(text)

    def _run(self, port, baud):
        connection = None
        failure = ""
        try:
            connection = self._factory(port, baud)
            with self._lock:
                self._status.state = "starting"
            self._event(f"Opened {port} at {baud}; waiting for ESP32 startup.")
            if self._stop.wait(self.startup_delay):
                return
            with self._lock:
                self._status.state = "streaming"
                self._status.streaming_since = monotonic()
            next_send = monotonic()
            buffer = bytearray()
            discard = False
            while not self._stop.is_set():
                now = monotonic()
                if now >= next_send:
                    with self._lock:
                        payload = self._packet
                    if connection.write(payload) != len(payload):
                        raise OSError("Incomplete UART write; reconnect to resynchronize frames.")
                    with self._lock:
                        self._status.sent += 1
                        self._status.last_sent = payload.decode("ascii")
                    next_send = monotonic()+self.period
                waiting = connection.in_waiting
                if waiting:
                    for byte in connection.read(min(waiting, 512)):
                        if byte == 10:
                            if not discard:
                                self._line(bytes(buffer).rstrip(b"\r"))
                            buffer.clear()
                            discard = False
                        elif not discard:
                            if len(buffer) >= 96:
                                buffer.clear()
                                discard = True
                            else:
                                buffer.append(byte)
                self._stop.wait(0.005)
        except Exception as error:
            failure = str(error)
            self._event(f"UART error: {failure}")
        finally:
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass
            with self._lock:
                self._status.state = "error" if failure else "offline"
                self._status.error = failure

    def drain_events(self):
        while True:
            try:
                yield self.events.get_nowait()
            except Empty:
                return
