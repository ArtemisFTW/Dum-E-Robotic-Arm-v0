#!/usr/bin/env python3
"""Exercise both real ROS nodes and pyserial on a PTY; never opens arm hardware."""
import json
import os
import pty
import select
from math import radians
from time import monotonic

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool
from dum_e_interfaces.msg import ServoCommand
from dum_e_interfaces.srv import SolveTarget
from dum_e_control.kinematics_node import KinematicsNode
from dum_e_control.serial_bridge import SerialBridge


def main():
    master, slave = pty.openpty()
    rclpy.init()
    ns = f'/dum_e_verification_{os.getpid()}'
    kin = KinematicsNode(namespace=ns)
    bridge = SerialBridge(namespace=ns, parameter_overrides=[
        Parameter('dry_run', value=False), Parameter('port', value=os.ttyname(slave)),
        Parameter('startup_delay', value=0.05), Parameter('command_timeout', value=0.25),
        Parameter('ack_timeout', value=0.3),
    ])
    tester = Node('integration_tester', namespace=ns)
    executor = SingleThreadedExecutor()
    for node in (kin, bridge, tester):
        executor.add_node(node)
    servo = tester.create_publisher(Float64MultiArray, 'servo_target', 1)
    xyz = tester.create_publisher(PointStamped, 'target_position', 1)
    joints = tester.create_publisher(JointState, 'joint_target', 1)
    raw = tester.create_publisher(ServoCommand, 'servo_commands', 1)
    enable = tester.create_client(SetBool, 'serial_bridge/enable')
    solve = tester.create_client(SolveTarget, 'solve_target')
    statuses = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [tester.create_subscription(String, f'{name}/status',
        lambda m, name=name: statuses.__setitem__(name, json.loads(m.data)), qos)
        for name in ('kinematics', 'serial_bridge')]
    frames, buffer = [], bytearray()
    reply = True

    def pump_once():
        executor.spin_once(timeout_sec=0.005)
        if select.select([master], [], [], 0)[0]:
            buffer.extend(os.read(master, 4096))
            while b'\n' in buffer:
                frame, _, rest = buffer.partition(b'\n')
                buffer[:] = rest
                frames.append(frame)
                if reply:
                    os.write(master, b'OK,' + frame + b'\nDONE,' + frame + b'\n')

    def wait(condition, timeout=4):
        deadline = monotonic() + timeout
        while not condition():
            assert monotonic() < deadline, f'ROS condition timed out. statuses={statuses}'
            pump_once()

    def service(client, request):
        wait(client.service_is_ready)
        future = client.call_async(request)
        wait(future.done)
        return future.result()

    def send(values):
        before = kin.accepted
        servo.publish(Float64MultiArray(data=[float(v) for v in values]))
        wait(lambda: kin.accepted > before)

    try:
        wait(lambda: len(statuses) == 2 and servo.get_subscription_count() == 1 and xyz.get_subscription_count() == 1)
        assert not frames and not kin.have_target and not bridge.engine.enabled
        assert not service(enable, SetBool.Request(data=True)).success
        result = service(solve, SolveTarget.Request(mode='xyz', values=[0.5, 0.5, 0.5]))
        assert not result.success and not kin.have_target
        result = service(solve, SolveTarget.Request(mode='servo', values=[70., 30., 40.]))
        assert result.success and list(result.degrees) == [70, 30, 40] and not kin.have_target
        print('PASS: no startup command/serial open; read-only solve and invalid target rejection')

        send([70, 30, 40])
        wait(lambda: bridge.engine.command == b'70,30,40\n')
        assert service(enable, SetBool.Request(data=True)).success
        wait(lambda: bridge.engine.completed >= 2)
        assert frames and all(frame == b'70,30,40' for frame in frames)
        print('PASS: target -> ROS message -> pyserial -> PTY -> OK/DONE -> ROS status')

        before = kin.accepted
        target = PointStamped()
        target.header.frame_id = 'base_link'
        target.point.x, target.point.y, target.point.z = [0.1291673, 0.0469705, 0.2119948]
        xyz.publish(target)
        wait(lambda: kin.accepted > before)
        assert kin.state.joints == [70, 30, 40]
        rejected = kin.rejected
        target.header.frame_id = 'unsupported_frame'
        xyz.publish(target)
        wait(lambda: kin.rejected > rejected)
        assert kin.state.joints == [70, 30, 40]
        rejected = kin.rejected
        servo.publish(Float64MultiArray(data=[181., 30., 40.]))
        wait(lambda: kin.rejected > rejected)
        assert kin.state.joints == [70, 30, 40]
        print('PASS: XYZ metres, calibrated IK, frame rejection and out-of-limit hold')

        before = kin.accepted
        msg = JointState(name=['joint_3', 'joint_1', 'joint_2'], position=[radians(-40), radians(10), radians(50)])
        joints.publish(msg)
        wait(lambda: kin.accepted > before)
        assert kin.state.joints == [80, 50, 50]
        wait(lambda: bridge.engine.last_done == '80,50,50')
        old_command = bridge.engine.command
        raw.publish(ServoCommand(degrees=[1, 2, 3]))  # zero timestamp is stale
        for _ in range(20):
            pump_once()
        assert bridge.engine.command == old_command
        print('PASS: named DH joints in radians, mounting conversion, stale message rejection')

        reply = False
        wait(lambda: bridge.engine.phase == 'fault')
        assert 'ACK timed out' in bridge.engine.error
        count = len(frames)
        for _ in range(30):
            pump_once()
        assert len(frames) == count and not bridge.engine.enabled
        print('PASS: missing ACK disables serial; fresh commands cannot auto-reconnect')

        reply = True
        assert service(enable, SetBool.Request(data=True)).success
        wait(lambda: bridge.engine.phase == 'streaming')
        os.write(master, b'READY,ARM_UART0,115200\n')
        wait(lambda: bridge.engine.phase == 'fault')
        assert 'reset or hold' in bridge.engine.error
        print('PASS: controller reset latches fault')

        assert service(enable, SetBool.Request(data=True)).success
        wait(lambda: bridge.engine.phase == 'streaming')
        executor.remove_node(kin)
        kin.destroy_node()
        kin = None
        wait(lambda: bridge.engine.phase == 'fault')
        assert 'command timed out' in bridge.engine.error
        print('PASS: kinematics-node death stops the serial stream')
        assert service(enable, SetBool.Request(data=False)).success
        print('ALL ROS 2 / PTY INTEGRATION CHECKS PASSED')
    finally:
        for node in (kin, bridge, tester):
            if node is not None:
                executor.remove_node(node)
                node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        os.close(master)
        os.close(slave)


if __name__ == '__main__':
    main()
