from dataclasses import replace
import json
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinematics import JOINT_LIMITS, KinematicsError, Robot, conditioning, forward, inverse
from model import InterfaceState


class KinematicsTests(unittest.TestCase):
    def setUp(self):
        self.robot = Robot.load()

    def test_supplied_dh_table_and_zero_pose(self):
        self.assertEqual(self.robot.a, (10.35, 125.7, 105))
        self.assertEqual(self.robot.d, (45.74, 6.24, 6.2))
        self.assertEqual(self.robot.alpha, (90, -180, 0))
        transform, points, _ = forward((0, 0, 0), self.robot)
        np.testing.assert_allclose(transform[:3, 3], [241.05, -.04, 45.74], atol=1e-11)
        np.testing.assert_allclose(points, [[0, 0, 0], [10.35, 0, 45.74],
                                           [136.05, -6.24, 45.74], [241.05, -.04, 45.74]], atol=1e-11)

    def test_offset_mapping_and_physical_limits(self):
        self.assertEqual(self.robot.dh_limits, ((-90, 90), (0, 180), (-90, 90)))
        self.assertEqual(self.robot.dh_to_servo((20, 30, -50)), (70, 30, 40))
        self.assertEqual(self.robot.servo_to_dh((70, 30, 40)), (20, 30, -50))
        for index, (lower, upper) in enumerate(JOINT_LIMITS):
            for boundary in (lower, upper):
                servo = [90, 90, 90]
                servo[index] = boundary
                np.testing.assert_allclose(self.robot.dh_to_servo(self.robot.servo_to_dh(servo)), servo)
            for invalid in (lower-.001, upper+.001):
                servo[index] = invalid
                with self.assertRaises(KinematicsError):
                    self.robot.dh_to_servo(self.robot.servo_to_dh(servo))

    def test_photographed_mounting_reference_and_each_joint_direction(self):
        # Independent physical references supplied with the arm photograph.
        # These expected numbers must not be generated through servo_to_dh.
        poses = [
            ((90, 0, 90), (241.05, -.04, 45.74)),   # Straight, maximum +X.
            ((90, 90, 90), (10.35, -.04, 276.44)),  # Shoulder upright.
            ((90, 180, 90), (-220.35, -.04, 45.74)),# Shoulder points left.
            ((90, 0, 0), (136.05, -.04, 150.74)),   # Distal link points up.
            ((90, 0, 180), (136.05, -.04, -59.26)), # Distal link points down.
            ((0, 0, 90), (.04, 241.05, 45.74)),   # Base towards camera is now +Y.
            ((180, 0, 90), (-.04, -241.05, 45.74)),
        ]
        for servo, expected in poses:
            with self.subTest(servo=servo):
                state = InterfaceState(joints=list(servo))
                np.testing.assert_allclose(state.position, expected, atol=1e-10)
                self.assertEqual(state.packet, ",".join(map(str, servo))+"\n")
                if servo[2] != 90:  # Regular elbow-up/down endpoints also solve in IK.
                    state.set_mode("IK")
                    state.set_target_vector(expected)
                    self.assertEqual(state.joints, list(servo))

    def test_horizontal_extension_is_manual_valid_but_singular_ik_holds(self):
        state = InterfaceState(joints=[90, 0, 90])
        self.assertEqual(state.dh_angles, (0, 0, 0))
        self.assertTrue(conditioning(state.dh_angles, self.robot)[1])
        state.set_mode("IK")
        with self.assertRaises(KinematicsError):
            state.set_target_vector((241.05, -.04, 45.74))
        self.assertEqual(state.packet, "90,0,90\n")

    def test_j1_reversal_endpoints_and_j2_j3_unchanged(self):
        self.assertEqual(self.robot.directions, (-1, 1, 1))
        for dh_angle, servo_angle in ((-90, 180), (0, 90), (90, 0)):
            self.assertEqual(self.robot.dh_to_servo((dh_angle, 0, 0)), (servo_angle, 0, 90))
            self.assertEqual(self.robot.servo_to_dh((servo_angle, 0, 90)), (dh_angle, 0, 0))
        self.assertEqual(self.robot.servo_to_dh((180, 180, 180)), (-90, 180, 90))
        self.assertEqual(self.robot.servo_to_dh((0, 0, 0)), (90, 0, -90))

    def test_cartesian_axis_signs_roundtrip_and_commands_unchanged(self):
        state = InterfaceState()
        packet = state.packet
        np.testing.assert_allclose(state.position,
                                   (10.35+230.7/2**.5, -.04, 45.74+20.7/2**.5), atol=1e-10)
        self.assertEqual(self.robot.position_to_user((10, 20, -30)), (10, 20, -30))
        self.assertEqual(self.robot.position_to_dh((10, 20, -30)), (10, 20, -30))
        self.assertEqual(state.packet, packet)
        state.set_joint(2, "40")  # Select the nearby negative-elbow branch.
        raw_target = forward((20, 30, -50), self.robot)[0][:3, 3]
        user_target = self.robot.position_to_user(raw_target)
        state.set_mode("IK")
        state.set_target_vector(user_target)
        self.assertEqual(state.packet, "70,30,40\n")
        np.testing.assert_allclose(state.position, user_target, atol=1e-10)

    def test_reversed_servo_direction(self):
        robot = replace(self.robot, directions=(-1, -1, 1))
        self.assertEqual(robot.dh_limits, ((-90, 90), (-180, 0), (-90, 90)))
        servo = robot.dh_to_servo((30, -50, -40))
        self.assertEqual(servo, (60, 50, 50))
        target = forward((30, -50, -40), robot)[0][:3, 3]
        np.testing.assert_allclose(inverse(target, robot, servo).servo_angles, servo, atol=1e-9)

    def test_all_joint_endpoints_are_reachable_after_conversion(self):
        for servo in ((0, 0, 0), (180, 180, 180)):
            state = InterfaceState(mode="IK", joints=list(servo))
            target = forward(self.robot.servo_to_dh(servo), self.robot)[0][:3, 3]
            state.set_target_vector(self.robot.position_to_user(target))
            self.assertEqual(state.joints, list(servo))
            self.assertEqual(state.packet, ",".join(map(str, servo))+"\n")

    def test_no_modulo_180_or_clipping(self):
        for q in ((180, 0, 0), (0, -1, 0), (0, 181, 0), (0, 0, 91)):
            with self.assertRaises(KinematicsError):
                self.robot.dh_to_servo(q)

    def test_random_fk_ik_roundtrips_and_branches(self):
        random = np.random.default_rng(152)
        checked = 0
        for _ in range(400):
            servo = random.uniform([0, 0, 0], [180, 180, 180])
            q = self.robot.servo_to_dh(servo)
            _, singular, near = conditioning(q, self.robot)
            if singular or near:
                continue
            target = forward(q, self.robot)[0][:3, 3]
            solution = inverse(target, self.robot, servo)
            np.testing.assert_allclose(solution.servo_angles, servo, atol=1e-8)
            self.assertLess(solution.position_error, 1e-8)
            checked += 1
        self.assertGreater(checked, 380)

    def test_independent_closed_form_and_finite_difference_jacobians(self):
        random = np.random.default_rng(22)
        for q in random.uniform(-160, 160, (100, 3)):
            t, _, jacobian = forward(q, self.robot)
            yaw, shoulder, elbow = np.radians(q)
            radial = self.robot.a[0]+self.robot.a[1]*np.cos(shoulder)+self.robot.a[2]*np.cos(shoulder-elbow)
            delta = self.robot.d[1]-self.robot.d[2]
            expected = [radial*np.cos(yaw)+delta*np.sin(yaw),
                        radial*np.sin(yaw)-delta*np.cos(yaw),
                        self.robot.d[0]+self.robot.a[1]*np.sin(shoulder)+self.robot.a[2]*np.sin(shoulder-elbow)]
            np.testing.assert_allclose(t[:3, 3], expected, atol=1e-10)
            numerical = np.zeros((3, 3))
            step = 1e-5
            for index in range(3):
                offset = np.zeros(3)
                offset[index] = np.degrees(step)
                numerical[:, index] = (forward(q+offset, self.robot)[0][:3, 3]-forward(q-offset, self.robot)[0][:3, 3])/(2*step)
            np.testing.assert_allclose(jacobian, numerical, rtol=1e-7, atol=1e-7)

    def test_failure_classes_hold_exact_previous_frame(self):
        state = InterfaceState()
        state.set_joint(2, "40")
        state.set_mode("IK")
        cases = [("UNREACHABLE", (500, 500, 500)),
                 ("UNREACHABLE_AXIAL", (0, 0, self.robot.d[0])),
                 ("SINGULAR", forward((0, 0, 0), self.robot)[0][:3, 3]),
                 ("NEAR_SINGULAR", forward((0, 0, .02), self.robot)[0][:3, 3]),
                 ("JOINT_LIMIT", forward((0, -135, 40), self.robot)[0][:3, 3])]
        for code, target in cases:
            with self.subTest(code=code):
                previous, revision = state.packet, state.revision
                with self.assertRaises(KinematicsError) as raised:
                    state.set_target_vector(self.robot.position_to_user(target))
                self.assertEqual(raised.exception.code, code)
                self.assertEqual(state.packet, previous)
                self.assertEqual(state.revision, revision)
        state.set_target_vector(self.robot.position_to_user(forward((20, 30, -50), self.robot)[0][:3, 3]))
        self.assertEqual(state.packet, "70,30,40\n")
        self.assertFalse(state.ik_error)

    def test_quantized_output_has_matching_fk_and_valid_range(self):
        state = InterfaceState(mode="IK")
        random = np.random.default_rng(26)
        for _ in range(100):
            servo = random.uniform([5, 20, 5], [175, 160, 175])
            if conditioning(self.robot.servo_to_dh(servo), self.robot)[2]:
                continue
            target = forward(self.robot.servo_to_dh(servo), self.robot)[0][:3, 3]
            state.set_target_vector(self.robot.position_to_user(target))
            self.assertRegex(state.packet, r"^\d+,\d+,\d+\n$")
            transmitted = [int(value) for value in state.packet.strip().split(",")]
            self.assertEqual(transmitted, state.joints)
            self.assertTrue(all(type(value) is int for value in state.joints))
            for value, (lower, upper) in zip(transmitted, JOINT_LIMITS):
                self.assertTrue(lower <= value <= upper)
            self.assertLess(state.position_error, 5.1)

    def test_integer_rounding_into_singularity_holds_previous_frame(self):
        state = InterfaceState(mode="IK")
        state.set_target_vector(self.robot.position_to_user(forward((20, 30, -50), self.robot)[0][:3, 3]))
        previous, revision = state.packet, state.revision
        # The exact elbow is nonsingular but rounds from 90.4° servo to 90°.
        target = forward((0, 45, .4), self.robot)[0][:3, 3]
        self.assertFalse(conditioning((0, 45, .4), self.robot)[1])
        self.assertFalse(conditioning((0, 45, .4), self.robot)[2])
        with self.assertRaises(KinematicsError) as raised:
            state.set_target_vector(self.robot.position_to_user(target))
        self.assertEqual(raised.exception.code, "SINGULAR")
        self.assertEqual(state.packet, previous)
        self.assertEqual(state.revision, revision)

    def test_invalid_configuration(self):
        for kwargs in ({"directions": (0, 1, 1)}, {"offsets": (90, float("nan"), 90)},
                       {"alpha": (-90, 0, 0)}, {"a": (10, 0, 105)}):
            with self.assertRaises(ValueError):
                Robot(**kwargs)


class MatlabReferenceTests(unittest.TestCase):
    def test_exported_matlab_transforms_jacobians_and_ik(self):
        fixture_path = Path(__file__).with_name("matlab_reference.json")
        if not fixture_path.exists():
            self.fail("Run export_matlab_reference.m in MATLAB to create the independent reference fixture.")
        fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
        robot = Robot.load()
        self.assertEqual(list(robot.a), fixture["a"])
        self.assertEqual(list(robot.offsets), fixture["servo_offsets"])
        self.assertEqual(list(robot.directions), fixture["servo_directions"])
        self.assertEqual(list(robot.xyz_signs), fixture["cartesian_axis_signs"])
        for index, case in enumerate(fixture["cases"]):
            with self.subTest(case=index):
                transform, points, jacobian = forward(case["q"], robot)
                np.testing.assert_allclose(transform, case["T"], atol=1e-10)
                np.testing.assert_allclose(points, case["points"], atol=1e-10)
                np.testing.assert_allclose(jacobian, case["J"], atol=1e-10)
                np.testing.assert_allclose(robot.position_to_user(case["target"]), case["app_target"], atol=1e-10)
                safe = np.asarray(case["safe_solutions"])
                reference_servo = tuple(offset+direction*value for offset, direction, value in
                                        zip(robot.offsets, robot.directions, case["q"]))
                np.testing.assert_allclose(reference_servo, case["servo"], atol=1e-10)
                if safe.size:
                    solution = inverse(case["target"], robot, reference_servo)
                    safe = safe.reshape(-1, 3)
                    self.assertLess(min(np.linalg.norm(row-solution.dh_angles) for row in safe), 1e-7)
                else:
                    with self.assertRaises(KinematicsError):
                        inverse(case["target"], robot, reference_servo)


if __name__ == "__main__":
    unittest.main()
