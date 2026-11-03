"""Short-lived ROS command client and completion-driven pose player."""
import argparse
import json
from pathlib import Path
from time import monotonic
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool
from dum_e_interfaces.srv import SolveTarget
import yaml


class Client(Node):
    def __init__(self, namespace):
        super().__init__('arm_command_' + uuid.uuid4().hex[:8], namespace=namespace)
        self.kinematics = self.serial = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'kinematics/status', lambda m: setattr(self, 'kinematics', json.loads(m.data)), qos)
        self.create_subscription(String, 'serial_bridge/status', lambda m: setattr(self, 'serial', json.loads(m.data)), qos)
        self.publisher = self.create_publisher(Float64MultiArray, 'servo_target', 1)
        self.solver = self.create_client(SolveTarget, 'solve_target')
        self.enabler = self.create_client(SetBool, 'serial_bridge/enable')

    def wait(self, condition, timeout=5.0, message='Timed out waiting for ROS nodes.'):
        deadline = monotonic() + timeout
        while rclpy.ok() and not condition():
            if monotonic() >= deadline:
                raise RuntimeError(message)
            rclpy.spin_once(self, timeout_sec=0.05)
        if not rclpy.ok():
            raise RuntimeError('ROS shut down.')

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'Service unavailable: {client.srv_name}; start run_ros2.sh first.')
        future = client.call_async(request)
        self.wait(future.done)
        result = future.result()
        if not result.success:
            raise RuntimeError(result.message)
        return result

    def resolve(self, mode, values):
        if len(values) != 3:
            raise ValueError('Every pose needs exactly three values.')
        return self.call(self.solver, SolveTarget.Request(mode=mode, values=[float(v) for v in values]))

    def send(self, values):
        self.wait(lambda: self.kinematics is not None and self.publisher.get_subscription_count() > 0)
        before = self.kinematics['accepted']
        rejected = self.kinematics['rejected']
        self.publisher.publish(Float64MultiArray(data=[float(v) for v in values]))
        self.wait(lambda: self.kinematics['accepted'] > before or self.kinematics['rejected'] > rejected)
        if self.kinematics['rejected'] > rejected:
            raise RuntimeError(self.kinematics['error'])
        expected = ','.join(str(v) for v in self.kinematics['servo_degrees'])
        self.wait(lambda: self.serial is not None and self.serial.get('pending_command') == expected,
                  message='Kinematics accepted the target, but the serial bridge has not received it.')

    def check_link(self):
        if self.serial is None or not self.serial['enabled'] or self.serial['phase'] == 'fault':
            raise RuntimeError('Serial bridge disabled or faulted; select a target and enable first.')

    def play(self, poses, cycles, hold):
        # Resolve the entire list through the actual running model before moving.
        resolved = []
        for pose in poses:
            if not isinstance(pose, dict) or pose.get('mode') not in ('servo', 'dh', 'xyz'):
                raise ValueError('Each YAML pose needs mode: servo/dh/xyz and values: [a,b,c].')
            result = self.resolve(pose['mode'], pose.get('values', []))
            resolved.append(list(result.degrees))
        if not resolved:
            raise ValueError('The pose list is empty.')
        self.wait(lambda: self.serial is not None)
        self.check_link()
        cycle = 0
        while cycles == 0 or cycle < cycles:
            cycle += 1
            for index, values in enumerate(resolved):
                self.check_link()
                baseline = self.serial['completed']
                self.send(values)
                expected = ','.join(map(str, values))
                def arrived():
                    self.check_link()
                    return self.serial['completed'] > baseline and self.serial['last_done'] == expected and self.serial['last_sent'] == expected
                self.wait(arrived, timeout=20.0, message='No matching ESP32 DONE within 20 seconds.')
                print(f'Cycle {cycle}, pose {index + 1}: {expected}; hold {hold:g}s', flush=True)
                until = monotonic() + hold
                while monotonic() < until:
                    self.check_link()
                    rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    parser = argparse.ArgumentParser(description='Dum-E ROS 2 control client')
    parser.add_argument('--namespace', default='/dum_e')
    commands = parser.add_subparsers(dest='command', required=True)
    for name in ('servo', 'dh', 'xyz'):
        p = commands.add_parser(name, help={'servo': 'servo degrees', 'dh': 'DH radians', 'xyz': 'XYZ metres'}[name])
        p.add_argument('values', nargs=3, type=float)
    for name in ('home', 'enable', 'disable', 'status'):
        commands.add_parser(name)
    loop = commands.add_parser('loop')
    loop.add_argument('file', type=Path)
    loop.add_argument('--cycles', type=int, default=1, help='0 repeats until Ctrl+C')
    loop.add_argument('--hold', type=float, default=2.0)
    parsed = parser.parse_args(args)
    rclpy.init(args=[])
    node = Client(parsed.namespace)
    result_code = 0
    try:
        if parsed.command in ('servo', 'dh', 'xyz', 'home'):
            mode = 'servo' if parsed.command == 'home' else parsed.command
            values = [90, 45, 180] if parsed.command == 'home' else parsed.values
            result = node.resolve(mode, values)
            node.send(list(result.degrees))
            print(json.dumps({'servo_degrees': [int(v) for v in result.degrees],
                              'xyz_m': [float(v) for v in result.xyz_m]}))
        elif parsed.command in ('enable', 'disable'):
            print(node.call(node.enabler, SetBool.Request(data=parsed.command == 'enable')).message)
        elif parsed.command == 'status':
            node.wait(lambda: node.kinematics is not None and node.serial is not None)
            print(json.dumps({'kinematics': node.kinematics, 'serial': node.serial}, indent=2))
        else:
            from math import isfinite
            if parsed.cycles < 0 or not isfinite(parsed.hold) or parsed.hold < 0:
                raise ValueError('cycles must be >=0 and hold must be finite and >=0.')
            data = yaml.safe_load(parsed.file.read_text())
            if not isinstance(data, dict) or not isinstance(data.get('poses'), list):
                raise ValueError('YAML must contain poses: followed by a list.')
            node.play(data['poses'], parsed.cycles, parsed.hold)
    except KeyboardInterrupt:
        print('Playback stopped; bridge continues the current target. Use arm.sh disable to end streaming.')
    except (RuntimeError, ValueError, OSError, yaml.YAMLError) as error:
        print(f'Error: {error}')
        result_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result_code
