"""All FK/IK, physical limits, quantization and mounting calibration live here."""
import json
from math import degrees, radians, isfinite

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from dum_e_interfaces.msg import ServoCommand
from dum_e_interfaces.srv import SolveTarget

from .core.kinematics import CONFIG_PATH, Robot, conditioning
from .core.model import InterfaceState

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3']


class KinematicsNode(Node):
    def __init__(self, **kwargs):
        super().__init__('kinematics', **kwargs)
        path = self.declare_parameter('robot_config', '').value
        self.model_path = str(path or CONFIG_PATH)
        self.frame = self.declare_parameter('base_frame', 'base_link').value
        rate = float(self.declare_parameter('publish_rate', 20.0).value)
        if not isfinite(rate) or not 5.0 <= rate <= 100.0:
            raise ValueError('publish_rate must be between 5 and 100 Hz')
        self.state = InterfaceState(robot=Robot.load(path) if path else Robot.load())
        self.have_target = False
        self.accepted = 0
        self.rejected = 0
        self.last_error = ''
        self.commands = self.create_publisher(ServoCommand, 'servo_commands', 1)
        self.joints = self.create_publisher(JointState, 'joint_targets', 1)
        self.position = self.create_publisher(PointStamped, 'tool_target', 1)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, 'kinematics/status', status_qos)
        self.create_subscription(PointStamped, 'target_position', self.on_xyz, 1)
        self.create_subscription(JointState, 'joint_target', self.on_joints, 1)
        self.create_subscription(Float64MultiArray, 'servo_target', self.on_servo, 1)
        self.create_service(Trigger, 'home', self.on_home)
        self.create_service(SolveTarget, 'solve_target', self.on_solve)
        self.create_timer(1.0 / rate, self.publish_target)
        self.publish_status()
        self.get_logger().info('Ready; waiting for an explicit target. XYZ=m, DH joints=rad.')

    def accept(self, operation):
        # Evaluate on a separate state so malformed input cannot partially move joints.
        candidate = InterfaceState(robot=self.state.robot, joints=list(self.state.joints))
        try:
            operation(candidate)
        except (ValueError, TypeError, OverflowError) as error:
            self.rejected += 1
            self.last_error = str(error)
            self.get_logger().warning(f'Target rejected: {error}')
            self.publish_status()
            return False
        self.state = candidate
        self.have_target = True
        self.accepted += 1
        self.last_error = ''
        self.publish_target()
        self.publish_status()
        return True

    def on_servo(self, msg):
        self.accept(lambda state: state.accept_command(msg.data))

    def on_joints(self, msg):
        def apply(state):
            if len(msg.name) != 3 or set(msg.name) != set(JOINT_NAMES) or len(msg.position) != 3:
                raise ValueError('Supply joint_1, joint_2, joint_3 exactly once with three radians.')
            ordered = [msg.position[msg.name.index(name)] for name in JOINT_NAMES]
            state.accept_command(state.robot.dh_to_servo([degrees(value) for value in ordered]))
        self.accept(apply)

    def on_xyz(self, msg):
        def apply(state):
            if msg.header.frame_id not in ('', self.frame):
                raise ValueError(f'XYZ target must be in {self.frame}; no implicit TF transform is performed.')
            state.set_mode('IK')
            state.set_target_vector([1000.0 * value for value in (msg.point.x, msg.point.y, msg.point.z)])
        self.accept(apply)

    def on_home(self, request, response):
        response.success = self.accept(lambda state: state.accept_command([90, 45, 180]))
        response.message = 'Home target selected; serial enable is separate.'
        return response

    def on_solve(self, request, response):
        robot = self.state.robot
        response.model_json = json.dumps({
            'a': robot.a, 'd': robot.d, 'alpha': robot.alpha,
            'offsets': robot.offsets, 'directions': robot.directions,
            'xyz_signs': robot.xyz_signs, 'dh_limits': robot.dh_limits,
            'config_path': self.model_path,
        })
        try:
            reference = request.reference_degrees if request.use_reference else self.state.joints
            candidate = InterfaceState(robot=robot, joints=list(reference))
            if request.mode == 'servo':
                candidate.accept_command(request.values)
            elif request.mode == 'dh':
                candidate.accept_command(candidate.robot.dh_to_servo([degrees(v) for v in request.values]))
            elif request.mode == 'xyz':
                candidate.set_mode('IK')
                candidate.set_target_vector([v * 1000.0 for v in request.values])
            else:
                raise ValueError('mode must be servo, dh or xyz')
            response.success = True
            response.message = 'Validated; this service does not move the arm.'
            response.degrees = candidate.joints
            response.xyz_m = [v / 1000.0 for v in candidate.position]
            response.dh_radians = [radians(v) for v in candidate.dh_angles]
            response.sigma_min, response.singular, response.near_singular = conditioning(candidate.dh_angles, robot)
            response.position_error_m = candidate.position_error / 1000.0
            response.branch = candidate.branch
        except (ValueError, TypeError, OverflowError) as error:
            response.success = False
            response.message = str(error)
            response.error_code = getattr(error, 'code', 'INVALID_INPUT')
        return response

    def publish_target(self):
        if not self.have_target:
            return
        stamp = self.get_clock().now().to_msg()
        command = ServoCommand(stamp=stamp, degrees=self.state.joints)
        self.commands.publish(command)
        joints = JointState()
        joints.header.stamp = stamp
        joints.header.frame_id = self.frame
        joints.name = JOINT_NAMES
        joints.position = [radians(value) for value in self.state.dh_angles]
        self.joints.publish(joints)
        point = PointStamped()
        point.header = joints.header
        point.point.x, point.point.y, point.point.z = [float(v) / 1000.0 for v in self.state.position]
        self.position.publish(point)

    def publish_status(self):
        self.status.publish(String(data=json.dumps({
            'have_target': self.have_target, 'accepted': self.accepted,
            'rejected': self.rejected, 'error': self.last_error,
            'servo_degrees': self.state.joints,
            'dh_radians': [radians(v) for v in self.state.dh_angles],
            'xyz_m': [v / 1000.0 for v in self.state.position],
            'feedback': 'calculated target, not measured shaft position',
        })))


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
