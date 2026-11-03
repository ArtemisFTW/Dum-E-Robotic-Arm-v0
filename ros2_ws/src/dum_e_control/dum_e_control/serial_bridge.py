"""The only ROS node that can open or write the ESP32 serial port."""
import json
from math import isfinite
from time import monotonic

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from dum_e_interfaces.msg import ServoCommand
from dum_e_interfaces.srv import ConfigureSerial

from .serial_engine import SerialEngine


class SerialBridge(Node):
    def __init__(self, **kwargs):
        super().__init__('serial_bridge', **kwargs)
        params = {name: self.declare_parameter(name, default).value for name, default in (
            ('dry_run', True), ('port', ''), ('startup_delay', 2.5),
            ('command_timeout', 0.5), ('ack_timeout', 2.0))}
        rate = float(self.declare_parameter('stream_rate', 20.0).value)
        if not isfinite(rate) or not 5.0 <= rate <= 50.0:
            raise ValueError('stream_rate must be between 5 and 50 Hz')
        self.engine = SerialEngine(**params)
        self.create_subscription(ServoCommand, 'servo_commands', self.on_command, 1)
        self.create_service(SetBool, 'serial_bridge/enable', self.on_enable)
        self.create_service(ConfigureSerial, 'serial_bridge/configure', self.on_configure)
        self.tx = self.create_publisher(String, 'serial_bridge/tx', 10)
        self.rx = self.create_publisher(String, 'serial_bridge/rx', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, 'serial_bridge/status', qos)
        self.create_timer(1.0 / rate, self.tick)
        self.last_status = None
        self.publish_status()
        self.get_logger().info(f'Serial disabled. dry_run={self.engine.dry_run}; enable service required.')

    def on_command(self, msg):
        stamp = msg.stamp.sec + msg.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if not -0.1 <= age <= self.engine.command_timeout:
            self.get_logger().warning('Rejected stale or future servo command.', throttle_duration_sec=2.0)
            return
        try:
            self.engine.update([int(v) for v in msg.degrees], monotonic())
            self.publish_status()
        except ValueError as error:
            self.get_logger().warning(f'Rejected servo command: {error}', throttle_duration_sec=2.0)

    def on_enable(self, request, response):
        try:
            if request.data:
                self.engine.enable(monotonic())
            else:
                self.engine.disable()
            response.success = True
            response.message = 'Enabled' if request.data else 'Disabled; firmware link timeout freezes PWM within about 1 second.'
        except ValueError as error:
            response.success = False
            response.message = str(error)
        self.publish_status()
        return response

    def on_configure(self, request, response):
        if self.engine.enabled:
            response.success = False
            response.message = 'Disconnect before changing the serial device or dry-run mode.'
        elif not request.dry_run and not request.port.startswith('/dev/'):
            response.success = False
            response.message = 'Select an explicit Linux /dev/ serial device.'
        else:
            self.engine.port = request.port
            self.engine.dry_run = request.dry_run
            self.engine.phase = 'disabled'
            self.engine.error = ''
            response.success = True
            response.message = 'Connection selection updated; device remains closed.'
        self.publish_status()
        return response

    def tick(self):
        previous = self.engine.phase
        for kind, text in self.engine.tick(monotonic()):
            (self.tx if kind == 'tx' else self.rx).publish(String(data=text))
        if self.engine.phase == 'fault' and previous != 'fault':
            self.get_logger().error(self.engine.error)
        self.publish_status()

    def publish_status(self):
        text = json.dumps(self.engine.status(), sort_keys=True)
        if text != self.last_status:
            self.status.publish(String(data=text))
            self.last_status = text

    def destroy_node(self):
        self.engine.disable()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
