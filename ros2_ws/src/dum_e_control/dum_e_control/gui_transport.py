"""Original transport API implemented with ROS; this GUI never opens serial."""
from dataclasses import dataclass
import re
from time import monotonic

from .gui_session import get_session
from .serial_engine import packet

DRY_RUN_PORT = 'ROS 2 dry run (no hardware)'


def validate_packet(text):
    if not re.fullmatch(r'[0-9]{1,3},[0-9]{1,3},[0-9]{1,3}\n', text):
        raise ValueError('UART needs three integer degrees and a newline.')
    return packet([int(v) for v in text.strip().split(',')])


@dataclass
class Status:
    state: str = 'offline'
    sent: int = 0
    acknowledged: int = 0
    last_ack_time: float = 0.0
    streaming_since: float = 0.0
    last_ack: str = ''
    last_sent: str = ''
    completed: int = 0
    last_done: str = ''
    last_done_time: float = 0.0
    error: str = ''


class SerialStreamer:
    def __init__(self):
        self.session = get_session()
        self._packet = None
        self.failure = ''

    @property
    def dry_run(self):
        return bool(self.session.serial and self.session.serial['dry_run'])

    @property
    def selected_port(self):
        return DRY_RUN_PORT if self.dry_run else self.session.serial.get('port', '')

    @property
    def running(self):
        return bool(self.session.serial and self.session.serial['enabled'])

    def publish(self, text):
        payload = validate_packet(text)
        changed = payload != self._packet
        self._packet = payload
        if self.running and changed:
            try:
                self.session.select_target([int(v) for v in text.strip().split(',')])
            except ValueError as error:
                self.failure = str(error)
                self.session.events.append('UART error: ' + str(error))

    def start(self, port, baud=115200):
        if int(baud) != 115200:
            raise ValueError('The ESP32 firmware requires 115200 baud.')
        if self._packet is None:
            raise ValueError('No validated command is available.')
        if not port or port in ('Select a port', 'No ports found'):
            raise ValueError('Select the ESP32 port or ROS 2 dry run first.')
        dry_run = port == DRY_RUN_PORT
        self.session.configure('' if dry_run else port, dry_run)
        self.session.select_target([int(v) for v in self._packet.decode('ascii').strip().split(',')])
        self.session.enable_serial()
        self.failure = ''
        self.session.pump()
        self.session.events.append('ROS 2 bridge enabled. Dry run has no physical output.' if dry_run else
                                   f'ROS 2 bridge opened {port}; waiting for ESP32 startup.')

    def stop(self):
        self.session.disable_serial()
        self.failure = ''

    def status(self):
        self.session.pump()
        data = self.session.serial or {}
        phase = data.get('phase', 'disabled')
        state = {'disabled': 'offline', 'starting': 'starting', 'streaming': 'streaming',
                 'dry_run': 'streaming', 'fault': 'error'}.get(phase, 'offline')
        error = self.failure or data.get('error', '')
        if data.get('enabled') and monotonic() - self.session.last_serial_seen > 1.0:
            state, error = 'error', 'ROS serial bridge status lost; check the ROS nodes.'
        elif self.failure:
            state = 'error'
        return Status(state=state, error=error, sent=data.get('sent', 0),
                      acknowledged=data.get('acknowledged', 0), completed=data.get('completed', 0),
                      last_ack_time=self.session.last_ack_time, last_done_time=self.session.last_done_time,
                      streaming_since=self.session.streaming_since, last_ack=data.get('last_ack', ''),
                      last_done=data.get('last_done', ''),
                      last_sent=data.get('last_sent', '') + ('\n' if data.get('last_sent') else ''))

    def drain_events(self):
        while self.session.events:
            yield self.session.events.popleft()
