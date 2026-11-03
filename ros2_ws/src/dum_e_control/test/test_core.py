import json
from pathlib import Path
from math import radians
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from dum_e_control.core.kinematics import Robot, forward, inverse, conditioning
from dum_e_control.core.model import InterfaceState
from dum_e_control.description import robot_description

ROOT = Path(__file__).resolve().parents[4]


def test_ros_solver_matches_upstream_sources():
    upstream = ROOT / 'Code' / 'arm_interface'
    core = Path(__import__('dum_e_control.core.kinematics', fromlist=['x']).__file__).parent
    for name in ('kinematics.py', 'model.py', 'robot_config.json'):
        expected = (upstream / name).read_text().replace('from kinematics import ', 'from .kinematics import ')
        assert (core / name).read_text() == expected


def test_matlab_exported_references_against_ros_solver():
    fixture = json.loads((ROOT / 'Code/arm_interface/tests/matlab_reference.json').read_text())
    robot = Robot.load()
    for case in fixture['cases']:
        transform, points, jacobian = forward(case['q'], robot)
        np.testing.assert_allclose(transform, case['T'], atol=1e-10)
        np.testing.assert_allclose(points, case['points'], atol=1e-10)
        np.testing.assert_allclose(jacobian, case['J'], atol=1e-10)


def test_quantization_and_failed_ik_preserve_previous_command():
    state = InterfaceState()
    state.accept_command([70, 30, 40])
    state.set_mode('IK')
    state.set_target_vector([129.1673, 46.9705, 211.9948])
    assert state.packet == '70,30,40\n'
    for bad in ([500, 500, 500], [float('nan'), 0, 0], [241.05, -.04, 45.74]):
        with pytest.raises(ValueError):
            state.set_target_vector(bad)
        assert state.packet == '70,30,40\n'


def test_400_random_fk_ik_roundtrips():
    robot = Robot.load()
    for servo in np.random.default_rng(152).uniform(0, 180, (400, 3)):
        q = robot.servo_to_dh(servo)
        _, singular, near = conditioning(q, robot)
        if singular or near:
            continue
        target = forward(q, robot)[0][:3, 3]
        np.testing.assert_allclose(inverse(target, robot, servo).servo_angles, servo, atol=1e-8)


def test_urdf_chain_matches_independent_dh_transforms():
    # Reconstruct the actual URDF origin/axis transforms, not the generator's inputs.
    urdf = ET.fromstring(robot_description())
    by_parent = {j.find('parent').get('link'): j for j in urdf.findall('joint')}
    robot = Robot.load()
    for degrees in ([0, 45, 90], [20, 30, -50], [-80, 160, 80]):
        transform = np.eye(4)
        parent = 'base_link'
        while parent != 'tool0':
            joint = by_parent[parent]
            origin = joint.find('origin')
            if origin is not None:
                xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
                roll, pitch, yaw = [float(v) for v in origin.get('rpy', '0 0 0').split()]
                c, s = np.cos(roll), np.sin(roll)
                assert pitch == yaw == 0
                step = np.array([[1, 0, 0, xyz[0]], [0, c, -s, xyz[1]], [0, s, c, xyz[2]], [0, 0, 0, 1.]])
                transform = transform @ step
            if joint.get('type') == 'revolute':
                q = radians(degrees[int(joint.get('name')[-1]) - 1])
                c, s = np.cos(q), np.sin(q)
                transform = transform @ np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.]])
            parent = joint.find('child').get('link')
        expected = forward(degrees, robot)[0].copy()
        expected[:3, 3] /= 1000
        np.testing.assert_allclose(transform, expected, atol=1e-12)
