"""ROS client shared by the original console's model and transport adapters."""
from collections import deque
import json
from time import monotonic

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from dum_e_interfaces.srv import ConfigureSerial, SolveTarget

from .command import Client

_session = None


class RemoteError(ValueError):
    def __init__(self, message, code='ROS_ERROR'):
        super().__init__(message)
        self.code = code


def install_session(session):
    global _session
    _session = session


def get_session():
    if _session is None:
        raise ValueError('Start this GUI with gui.sh so the ROS services are available.')
    return _session


class GuiSession(Client):
    def __init__(self, namespace='/dum_e'):
        super().__init__(namespace)
        self.configurer = self.create_client(ConfigureSerial, 'serial_bridge/configure')
        self.events = deque(maxlen=100)
        self.last_serial_seen = 0.0
        self.last_ack_time = self.last_done_time = self.streaming_since = 0.0
        self.previous_serial = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'serial_bridge/status', self.observe_status, qos)
        self.create_subscription(String, 'serial_bridge/rx', self.observe_rx, 100)
        try:
            self.wait(lambda: self.kinematics is not None and self.serial is not None, timeout=10)
        except RuntimeError as error:
            raise RemoteError(str(error)) from error

    def observe_status(self, msg):
        data = json.loads(msg.data)
        now = monotonic()
        if data['acknowledged'] != self.previous_serial.get('acknowledged', 0):
            self.last_ack_time = now
        if data['completed'] != self.previous_serial.get('completed', 0):
            self.last_done_time = now
        if data['phase'] in ('streaming', 'dry_run') and self.previous_serial.get('phase') not in ('streaming', 'dry_run'):
            self.streaming_since = now
        if data['error'] and data['error'] != self.previous_serial.get('error'):
            self.events.append('UART error: ' + data['error'])
        self.serial = data
        self.previous_serial = data
        self.last_serial_seen = now

    def observe_rx(self, msg):
        if msg.data.startswith(('READY,', 'ERR,', 'HOLD,')):
            self.events.append(msg.data)

    def pump(self):
        for _ in range(15):
            if rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0)

    def preview(self, mode, values, reference):
        try:
            if not self.solver.wait_for_service(timeout_sec=0.5):
                raise RemoteError('ROS kinematics node is unavailable; last command held.')
            request = SolveTarget.Request(mode=mode, values=[float(v) for v in values],
                                          use_reference=True, reference_degrees=[int(v) for v in reference])
            future = self.solver.call_async(request)
            self.wait(future.done, timeout=2)
            result = future.result()
            if not result.success:
                raise RemoteError(result.message, result.error_code)
            return result
        except (RuntimeError, TypeError, OverflowError) as error:
            raise RemoteError(str(error)) from error

    def select_target(self, values):
        try:
            self.send(values)
        except RuntimeError as error:
            raise RemoteError(str(error)) from error

    def configure(self, port, dry_run):
        try:
            self.call(self.configurer, ConfigureSerial.Request(port=port, dry_run=dry_run))
        except RuntimeError as error:
            raise RemoteError(str(error)) from error

    def enable_serial(self):
        try:
            self.call(self.enabler, SetBool.Request(data=True))
        except RuntimeError as error:
            raise RemoteError(str(error)) from error

    def disable_serial(self):
        if not rclpy.ok():
            return
        if self.serial is None or not self.serial['enabled']:
            return
        try:
            if not self.enabler.wait_for_service(timeout_sec=0.5):
                raise RemoteError('Serial bridge unavailable; disconnect could not be confirmed.')
            future = self.enabler.call_async(SetBool.Request(data=False))
            self.wait(future.done, timeout=1.0)
            response = future.result()
            if not response.success:
                raise RemoteError(response.message)
            self.wait(lambda: not self.serial['enabled'], timeout=1.0)
        except (RuntimeError, RemoteError) as error:
            self.events.append('UART error: ' + str(error))
