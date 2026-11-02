from pathlib import Path
from queue import Queue
import sys
from threading import Event, Lock
from time import monotonic, sleep
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from uart_transport import SerialStreamer, validate_packet


class FakeSerial:
    def __init__(self):
        self.writes = Queue()
        self.rx = bytearray()
        self.lock = Lock()
        self.closed = False
        self.partial = False
        self.fail = False
        self.auto_done = True  # Simulated controller reaches each setpoint instantly.

    def write(self, data):
        if self.fail:
            raise OSError("device removed")
        self.writes.put(data)
        if self.partial:
            return len(data)-1
        with self.lock:
            self.rx += b"OK,"+data
            if self.auto_done:
                self.rx += b"DONE,"+data
        return len(data)

    @property
    def in_waiting(self):
        with self.lock:
            return len(self.rx)

    def read(self, size):
        with self.lock:
            data = bytes(self.rx[:size])
            del self.rx[:size]
            return data

    def close(self):
        self.closed = True


def wait_for(predicate, timeout=1):
    deadline = monotonic()+timeout
    while monotonic() < deadline:
        if predicate():
            return True
        sleep(.005)
    return False


class TransportTests(unittest.TestCase):
    def test_done_validates_integer_payload_and_never_counts_as_ack(self):
        stream = SerialStreamer()
        stream._line(b"DONE,90,45,180")
        status = stream.status()
        self.assertEqual(status.completed, 1)
        self.assertEqual(status.last_done, "90,45,180")
        self.assertGreater(status.last_done_time, 0)
        self.assertEqual(status.acknowledged, 0)
        for line in (b"DONE,90.0,45,180", b"DONE,90,45,181", b"DONE,90,45", b"DONE,90,45,180junk"):
            stream._line(line)
        self.assertEqual(stream.status().completed, 1)

    def test_packet_contract(self):
        for packet in ("0,0,0\n", "180,180,180\n", "90,14,60\n", "90,166,60\n"):
            self.assertEqual(validate_packet(packet), packet.encode("ascii"))
        for text in ("90.0,45,60\n", "-1,45,60\n", "90,-1,60\n", "90,181,60\n",
                     "181,45,60\n", "90,45,60", "90,45,60\nignored", "９０,45,60\n"):
            with self.assertRaises(ValueError):
                validate_packet(text)

    def test_invalid_publish_never_replaces_last_valid_frame(self):
        fake = FakeSerial()
        stream = SerialStreamer(lambda *_: fake, period=.01, startup_delay=0)
        stream.publish("90,45,60\n")
        with self.assertRaises(ValueError):
            stream.publish("181,45,60\n")
        try:
            stream.start("FAKE")
            self.assertEqual(fake.writes.get(timeout=1), b"90,45,60\n")
            self.assertEqual(fake.writes.get(timeout=1), b"90,45,60\n")
            self.assertTrue(wait_for(lambda: stream.status().acknowledged >= 2))
            self.assertGreaterEqual(stream.status().completed, 2)
            self.assertEqual(stream.status().last_done, "90,45,60")
            self.assertEqual(stream.status().last_sent, "90,45,60\n")
        finally:
            stream.stop()
        self.assertTrue(fake.closed)
        self.assertFalse(stream.running)

    def test_latest_slot_during_startup_and_hold_heartbeat(self):
        fake = FakeSerial()
        opened = Event()
        def factory(*_):
            opened.set()
            return fake
        stream = SerialStreamer(factory, period=.02, startup_delay=.1)
        stream.publish("90,90,90\n")
        try:
            stream.start("FAKE")
            self.assertTrue(opened.wait(1))
            for angle in range(100):
                stream.publish(f"{angle},45,60\n")
            self.assertEqual(fake.writes.get(timeout=1), b"99,45,60\n")
            self.assertEqual(fake.writes.get(timeout=1), b"99,45,60\n")
            self.assertEqual(stream.status().last_ack, "99,45,60")
        finally:
            stream.stop()

    def test_disconnect_before_startup_sends_nothing(self):
        fake = FakeSerial()
        stream = SerialStreamer(lambda *_: fake, startup_delay=2)
        stream.publish("90,90,90\n")
        stream.start("FAKE")
        stream.stop()
        self.assertTrue(fake.writes.empty())
        self.assertTrue(fake.closed)
        self.assertEqual(stream.status().last_sent, "")

    def test_partial_write_fails_closed(self):
        fake = FakeSerial()
        fake.partial = True
        stream = SerialStreamer(lambda *_: fake, startup_delay=0)
        stream.publish("90,90,90\n")
        stream.start("FAKE")
        self.assertTrue(wait_for(lambda: stream.status().state == "error"))
        stream.stop()
        self.assertEqual(stream.status().sent, 0)
        self.assertEqual(stream.status().last_sent, "")
        self.assertIn("Incomplete UART write", stream.status().error)
        self.assertEqual(fake.writes.qsize(), 1)

    def test_last_complete_write_survives_failure_and_resets_on_reconnect(self):
        fake = FakeSerial()
        stream = SerialStreamer(lambda *_: fake, period=.01, startup_delay=0)
        stream.publish("70,60,40\n")
        try:
            stream.start("FAKE")
            self.assertTrue(wait_for(lambda: stream.status().last_sent == "70,60,40\n"))
            fake.partial = True
            stream.publish("71,60,40\n")
            self.assertTrue(wait_for(lambda: stream.status().state == "error"))
            self.assertEqual(stream.status().last_sent, "70,60,40\n")
        finally:
            stream.stop()
        # During the next startup dwell there has been no write on that connection.
        stream.startup_delay = 2
        try:
            stream.start("FAKE")
            self.assertEqual(stream.status().last_sent, "")
        finally:
            stream.stop()

    def test_open_failure_and_read_disconnect_are_reported(self):
        def fail(*_):
            raise OSError("port busy")
        stream = SerialStreamer(fail, startup_delay=0)
        stream.publish("90,90,90\n")
        stream.start("FAKE")
        self.assertTrue(wait_for(lambda: stream.status().state == "error"))
        stream.stop()
        self.assertEqual(stream.status().error, "port busy")

    def test_start_requires_valid_port_and_matching_baud(self):
        stream = SerialStreamer()
        stream.publish("90,90,90\n")
        for port, baud in (("Select a port", 115200), ("FAKE", 9600)):
            with self.assertRaises(ValueError):
                stream.start(port, baud)
        self.assertFalse(stream.running)


if __name__ == "__main__":
    unittest.main()
