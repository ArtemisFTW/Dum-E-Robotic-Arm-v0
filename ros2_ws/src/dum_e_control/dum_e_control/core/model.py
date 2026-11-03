"""Arm interface state: calibrated servo commands and FK/IK validation."""

from dataclasses import dataclass, field
from math import ceil, floor, isfinite, radians

import numpy as np

from .kinematics import HOME_JOINTS, JOINT_LIMITS, KinematicsError, Robot, conditioning, forward, inverse


CARTESIAN_SLIDER_LIMITS = (-500.0, 500.0)  # UI travel, NOT the robot workspace.
MODES = ("Jog", "FK", "IK")


def number(text: str) -> float:
    try:
        value = float(text)
    except (ValueError, TypeError):
        raise ValueError("Enter a numeric value.") from None
    if not isfinite(value):
        raise ValueError("Enter a finite number; NaN and infinity are not allowed.")
    return value


def display(value: float) -> str:
    text = f"{value:.1f}".rstrip("0").rstrip(".")
    return "0" if text == "-0" else text


def whole_degrees(value: float) -> int:
    """Nearest whole degree; exact half degrees round away from zero."""
    value = number(value)
    return floor(value+0.5) if value >= 0 else ceil(value-0.5)


@dataclass
class InterfaceState:
    mode: str = "Jog"
    joints: list[int] = field(default_factory=lambda: list(HOME_JOINTS))
    target: list[float] = field(default_factory=list)
    paused: bool = False
    revision: int = 0
    robot: Robot = field(default_factory=Robot.load)
    ik_error: str = ""
    ik_error_code: str = ""
    branch: str = ""
    target_dirty: bool = False

    def __post_init__(self):
        self.robot.dh_to_servo(self.robot.servo_to_dh(self.joints))
        self.joints = [whole_degrees(value) for value in self.joints]
        if not self.target:
            self.target = list(self.position)

    @property
    def dh_angles(self):
        return self.robot.servo_to_dh(self.joints)

    @property
    def position(self):
        return self.robot.position_to_user(forward(self.dh_angles, self.robot)[0][:3, 3])

    def set_mode(self, mode: str) -> None:
        if mode not in MODES:
            raise ValueError("Unknown control mode.")
        if mode == "IK" and self.mode != "IK":
            # Enter at the current FK position without a jump or implicit solve.
            self.target = list(self.position)
            self.ik_error = self.ik_error_code = ""
            self.target_dirty = False
        self.mode = mode

    def set_joint(self, index: int, text: str) -> None:
        if self.paused or self.mode == "IK":
            raise ValueError("Joint input is currently locked.")
        raw = number(text)
        value = whole_degrees(raw)
        # Validate the raw value too: rounding must not hide a limit violation.
        lower, upper = JOINT_LIMITS[index]
        if not lower <= raw <= upper:
            raise ValueError(f"J{index + 1} must be between {lower:g}° and {upper:g}°.")
        if self.joints[index] != value:
            self.joints[index] = value
            self.revision += 1
        self.target = list(self.position)
        self.ik_error = self.ik_error_code = self.branch = ""
        self.target_dirty = False

    def set_target(self, index: int, text: str) -> None:
        """Stage an IK input. Only set_target_vector (Solve XYZ) may move joints."""
        if self.paused or self.mode != "IK":
            raise ValueError("Cartesian input is available in IK mode while inputs are running.")
        target = list(self.target)
        target[index] = number(text)
        lower, upper = CARTESIAN_SLIDER_LIMITS
        if not lower <= target[index] <= upper:
            raise ValueError("Cartesian controls span −500 to 500 mm; the solver checks the actual workspace.")
        self.target = target
        self.target_dirty = True
        self.ik_error = self.ik_error_code = self.branch = ""

    def set_target_vector(self, values) -> None:
        if self.paused or self.mode != "IK":
            raise ValueError("Cartesian input is available in IK mode while inputs are running.")
        if len(values) != 3:
            raise ValueError("Enter all three Cartesian coordinates.")
        target = [number(value) for value in values]
        lower, upper = CARTESIAN_SLIDER_LIMITS
        if any(not lower <= value <= upper for value in target):
            raise ValueError("Cartesian controls span −500 to 500 mm; the solver checks the actual workspace.")
        # Keep a failed request editable. Replace commands only after every check.
        self.target = target
        self.target_dirty = True
        try:
            solution = inverse(self.robot.position_to_dh(target), self.robot, self.joints)
            command = [whole_degrees(value) for value in solution.servo_angles]
            self.robot.dh_to_servo(self.robot.servo_to_dh(command))
            q = self.robot.servo_to_dh(command)
            _, singular, near = conditioning(q, self.robot)
            if singular or near:
                raise KinematicsError("SINGULAR", "Servo rounding places the solution too close to a singularity.")
            achieved = self.robot.position_to_user(forward(q, self.robot)[0][:3, 3])
            error = float(np.linalg.norm(np.asarray(achieved)-target))
            # Bound Cartesian displacement from whole-degree commands (±0.5°).
            rounding_bound = radians(0.5)*(2*self.robot.reach+abs(self.robot.a[0])+
                                           abs(self.robot.d[1]-self.robot.d[2])+self.robot.a[2])+1e-6
            if error > rounding_bound:
                raise KinematicsError("FK_MISMATCH", "Rounded servo commands failed the FK position check.")
        except KinematicsError as error:
            self.ik_error, self.ik_error_code = str(error), error.code
            raise
        if self.joints != command:
            self.joints = command
            self.revision += 1
        self.ik_error = self.ik_error_code = ""
        self.branch = solution.branch
        self.target_dirty = False

    def accept_command(self, values):
        """Accept an automation command atomically, through manual joint validation."""
        if len(values) != 3:
            raise ValueError("Enter three servo angles.")
        candidate = InterfaceState(robot=self.robot, joints=list(self.joints))
        for index, value in enumerate(values):
            candidate.set_joint(index, value)
        if self.joints != candidate.joints:
            self.revision += 1
        self.joints = list(candidate.joints)
        self.target = list(candidate.position)
        self.ik_error = self.ik_error_code = self.branch = ""
        self.target_dirty = False

    @property
    def position_error(self):
        return float(np.linalg.norm(np.asarray(self.position)-self.target))

    @property
    def packet(self) -> str:
        # Final output boundary validates calibrated servo-domain commands.
        self.robot.dh_to_servo(self.robot.servo_to_dh(self.joints))
        if any(type(value) is not int for value in self.joints):
            raise ValueError("Servo commands must be stored as whole-degree integers.")
        return ",".join(str(value) for value in self.joints) + "\n"
