import os
import pty
import select

import pytest

from dum_e_control.serial_engine import SerialEngine, packet


class FakeSerial:
    def __init__(self, partial=False):
        self.rx = bytearray()
        self.writes = []
        self.closed = False
        self.partial = partial

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, count):
        value = bytes(self.rx[:count])
        del self.rx[:count]
        return value

    def write(self, data):
        self.writes.append(data)
        return len(data) - 1 if self.partial else len(data)

    def close(self):
        self.closed = True


def ready(fake=None, **kwargs):
    device = fake or FakeSerial()
    engine = SerialEngine(dry_run=False, port='/dev/fake', startup_delay=0,
                          factory=lambda _: device, **kwargs)
    engine.update([90, 45, 180], 10.0)
    engine.enable(10.0)
    return engine, device


def test_no_initial_motion_or_auto_open():
    engine = SerialEngine(dry_run=False, port='/dev/fake', factory=lambda _: pytest.fail('opened automatically'))
    assert engine.tick(10) == []
    with pytest.raises(ValueError, match='target first'):
        engine.enable(10)
    assert not engine.enabled


def test_frame_limits_and_atomic_update():
    engine, _ = ready()
    for invalid in ([181, 45, 180], [90.0, 45, 180], [90, -1, 90], [90, 45], [True, 45, 90]):
        with pytest.raises(ValueError):
            engine.update(invalid, 10.1)
        assert engine.command == b'90,45,180\n'
        assert engine.received_at == 10.0
    assert packet([0, 180, 1]) == b'0,180,1\n'


def test_latest_target_and_startup_delay():
    fake = FakeSerial()
    engine = SerialEngine(dry_run=False, port='fake', startup_delay=2.5, factory=lambda _: fake)
    engine.update([90, 45, 180], 10)
    engine.enable(10)
    engine.tick(10)
    assert fake.writes == []
    engine.update([70, 30, 40], 12.5)
    engine.tick(12.5)
    assert fake.writes == [b'70,30,40\n']


def test_stale_kinematics_latches_fault_and_does_not_reconnect():
    engine, fake = ready()
    engine.tick(10)
    engine.tick(10.51)
    assert not engine.enabled and fake.closed
    assert 'command timed out' in engine.error
    engine.update([70, 30, 40], 11)
    engine.tick(11)
    assert len(fake.writes) == 1
    assert engine.phase == 'fault'


def test_ack_timeout_despite_fresh_commands():
    engine, fake = ready(ack_timeout=0.2)
    engine.tick(10)
    engine.update([70, 30, 40], 10.3)
    engine.tick(10.3)
    assert fake.closed and not engine.enabled
    assert 'ACK timed out' in engine.error


def test_partial_write_disarms():
    engine, fake = ready(FakeSerial(partial=True))
    engine.tick(10)
    assert fake.closed and 'Incomplete UART write' in engine.error


def test_valid_ack_done_and_malformed_or_wrong_target_ignored():
    engine, fake = ready()
    engine.tick(10)
    fake.rx += b'OK,70,30,40\nOK,90.0,45,180\nOK,90,45,181\nDONE,90,45,180\n'
    engine.tick(10.05)
    assert engine.acknowledged == engine.completed == 0
    fake.rx += b'OK,90,45,180\r\nDONE,90,45,180\n'
    engine.tick(10.1)
    assert engine.acknowledged == engine.completed == 1
    assert engine.last_done == '90,45,180'


@pytest.mark.parametrize('line', [b'READY,ARM_UART0,115200\n', b'HOLD,LINK_TIMEOUT\n', b'HOLD,STARTUP_HOME,90,45,180\n', b'ERR,PWM_FAILURE_RESTART_REQUIRED\n'])
def test_firmware_reset_hold_and_error_latch_fault(line):
    engine, fake = ready()
    engine.tick(10)
    fake.rx += line
    engine.tick(10.05)
    assert engine.phase == 'fault' and fake.closed


def test_bounded_input_recovers_at_newline():
    engine, fake = ready()
    engine.tick(10)
    fake.rx += b'x' * 100 + b'OK,90,45,180\nOK,90,45,180\n'
    engine.tick(10.05)
    assert engine.acknowledged == 1
    assert len(engine.buffer) <= 96


def test_dry_run_never_opens_a_port_and_marks_synthetic_feedback():
    engine = SerialEngine(factory=lambda _: pytest.fail('dry run opened a port'))
    engine.update([70, 30, 40], 10)
    engine.enable(10)
    assert ('tx', '70,30,40\n') in engine.tick(10)
    assert engine.phase == 'dry_run'
    assert engine.status()['dry_run']
    assert engine.last_done == '70,30,40'


def test_real_pyserial_over_linux_pty():
    master, slave = pty.openpty()
    engine = SerialEngine(dry_run=False, port=os.ttyname(slave), startup_delay=0)
    try:
        engine.update([70, 30, 40], 10)
        engine.enable(10)
        engine.tick(10)
        assert select.select([master], [], [], 1)[0]
        assert os.read(master, 512) == b'70,30,40\n'
        os.write(master, b'OK,70,30,40\nDONE,70,30,40\n')
        # Kernel PTY delivery is asynchronous; wait for input without sending another target.
        assert select.select([slave], [], [], 1)[0]
        engine.tick(10.1)
        assert engine.acknowledged == engine.completed == 1
    finally:
        engine.disable()
        os.close(master)
        os.close(slave)
