"""One latest-command slot, one serial writer, bounded input, latched faults.

This module has no ROS dependency, allowing deterministic and PTY verification.
The caller supplies monotonic time and calls tick at the configured stream rate.
"""
from math import isfinite
import re


def packet(values):
    if len(values) != 3 or any(type(v) is not int or not 0 <= v <= 180 for v in values):
        raise ValueError('Exactly three integer servo degrees in [0, 180] are required.')
    return (','.join(map(str, values)) + '\n').encode('ascii')


class SerialEngine:
    def __init__(self, *, dry_run=True, port='', startup_delay=2.5,
                 command_timeout=0.5, ack_timeout=2.0, factory=None):
        for name, value in [('startup_delay', startup_delay), ('command_timeout', command_timeout),
                            ('ack_timeout', ack_timeout)]:
            if not isfinite(value) or value < 0 or (name != 'startup_delay' and value == 0):
                raise ValueError(f'{name} must be finite and positive (startup_delay may be zero).')
        self.dry_run, self.port = dry_run, port
        self.startup_delay, self.command_timeout, self.ack_timeout = startup_delay, command_timeout, ack_timeout
        self.factory = factory or self.open_serial
        self.connection = None
        self.enabled = False
        self.phase = 'disabled'
        self.error = ''
        self.command = None
        self.received_at = None
        self.sent = self.acknowledged = self.completed = 0
        self.last_sent = self.last_ack = self.last_done = ''
        self.last_ack_at = None
        self.stream_started = None
        self.buffer = bytearray()
        self.discard = False
        self.events = []

    @staticmethod
    def open_serial(port):
        import serial
        return serial.Serial(port, baudrate=115200, bytesize=8, parity='N', stopbits=1,
                             timeout=0, write_timeout=0.1, exclusive=True,
                             rtscts=False, dsrdtr=False)

    def update(self, values, now):
        command = packet(values)  # Validate before replacing the previous command.
        self.command = command
        self.received_at = now

    def fresh(self, now):
        return self.command is not None and self.received_at is not None and 0 <= now-self.received_at <= self.command_timeout

    def enable(self, now):
        if self.enabled:
            return
        if not self.fresh(now):
            raise ValueError('Select a valid target first; no fresh kinematics command is available.')
        if not self.dry_run and not self.port:
            raise ValueError('Set an explicit Linux serial port before enabling hardware.')
        connection = None
        try:
            if not self.dry_run:
                connection = self.factory(self.port)
        except Exception as error:
            self.phase, self.error = 'fault', str(error)
            raise ValueError(f'Cannot open serial port: {error}') from error
        self.connection = connection
        self.enabled = True
        self.phase = 'starting'
        self.error = ''
        self.ready_at = now + (0.0 if self.dry_run else self.startup_delay)
        self.stream_started = self.last_ack_at = None
        self.last_sent = self.last_ack = self.last_done = ''
        self.buffer.clear()
        self.discard = False

    def disable(self, error=''):
        if self.connection is not None:
            try:
                self.connection.close()
            except Exception:
                pass
        self.connection = None
        self.enabled = False
        self.phase = 'fault' if error else 'disabled'
        self.error = error

    def line(self, data, now):
        text = data.decode('ascii', errors='replace')
        self.events.append(('rx', text))
        if text.startswith(('OK,', 'DONE,')):
            prefix, _, body = text.partition(',')
            if not re.fullmatch(r'[0-9]{1,3},[0-9]{1,3},[0-9]{1,3}', body):
                return
            try:
                reply = packet([int(v) for v in body.split(',')])
            except ValueError:
                return
            if not self.last_sent or reply.decode('ascii').strip() != self.last_sent:
                return
            if prefix == 'OK':
                self.last_ack_at = now
                self.last_ack = body
                self.acknowledged += 1
            elif self.last_ack == body:
                self.last_done = body
                self.completed += 1
        elif text.startswith('ERR,'):
            self.disable(f'Firmware error: {text}')
        elif self.stream_started is not None and text.startswith(('HOLD,', 'READY,')):
            self.disable(f'Controller reset or hold during streaming: {text}')

    def read(self, now):
        waiting = min(self.connection.in_waiting, 512)
        data = self.connection.read(waiting) if waiting else b''
        for byte in data:
            if byte == 10:
                if not self.discard:
                    self.line(bytes(self.buffer).rstrip(b'\r'), now)
                self.buffer.clear()
                self.discard = False
                if not self.enabled:
                    return
            elif not self.discard:
                if len(self.buffer) >= 96:
                    self.buffer.clear()
                    self.discard = True
                else:
                    self.buffer.append(byte)

    def tick(self, now):
        self.events = []
        if not self.enabled:
            return self.events
        try:
            if not self.fresh(now):
                self.disable('Kinematics command timed out; explicitly enable again after recovery.')
                return self.events
            if not self.dry_run:
                self.read(now)
            if not self.enabled or now < self.ready_at:
                return self.events
            if self.stream_started is None:
                self.stream_started = now
                self.phase = 'dry_run' if self.dry_run else 'streaming'
            reference = self.last_ack_at if self.last_ack_at is not None else self.stream_started
            if now-reference > self.ack_timeout:
                self.disable('ESP32 ACK timed out; explicitly enable again after checking the link.')
                return self.events
            if not self.dry_run and self.connection.write(self.command) != len(self.command):
                raise OSError('Incomplete UART write; reconnect to resynchronize frames.')
            self.last_sent = self.command.decode('ascii').strip()
            self.sent += 1
            self.events.append(('tx', self.last_sent + '\n'))
            if self.dry_run:
                self.line(b'OK,' + self.command.rstrip(b'\n'), now)
                self.line(b'DONE,' + self.command.rstrip(b'\n'), now)
        except Exception as error:
            self.disable(str(error))
        return self.events

    def status(self):
        result = {name: getattr(self, name) for name in (
            'enabled', 'phase', 'dry_run', 'port', 'error', 'sent', 'acknowledged',
            'completed', 'last_sent', 'last_ack', 'last_done')}
        result['pending_command'] = self.command.decode('ascii').strip() if self.command else ''
        return result
