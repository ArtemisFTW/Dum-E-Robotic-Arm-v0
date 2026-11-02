"""Current MATLAB RRR equations: standard DH, degrees, millimetres.

Servo = offset + direction * DH angle. Analytical IK needs [90, -180, 0] twists.
"""

from dataclasses import dataclass
import json
from math import atan2, ceil, cos, degrees, floor, hypot, isfinite, radians, sin, sqrt
from pathlib import Path

import numpy as np


JOINT_LIMITS = ((0.0, 180.0), (0.0, 180.0), (0.0, 180.0))
HOME_JOINTS = (90, 45, 180)
CONFIG_PATH = Path(__file__).with_name("robot_config.json")


class KinematicsError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def vector(values, name):
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        raise ValueError(f"{name} must contain three finite numbers.") from None
    if len(result) != 3 or not all(isfinite(value) for value in result):
        raise ValueError(f"{name} must contain three finite numbers.")
    return result


@dataclass(frozen=True)
class Robot:
    a: tuple = (10.35, 125.7, 105.0)
    d: tuple = (45.74, 6.24, 6.2)
    alpha: tuple = (90.0, -180.0, 0.0)
    offsets: tuple = (90.0, 0.0, 90.0)
    directions: tuple = (-1, 1, 1)
    xyz_signs: tuple = (1, 1, 1)
    near_threshold: float = 1e-3

    def __post_init__(self):
        for name in ("a", "d", "alpha", "offsets", "directions", "xyz_signs"):
            object.__setattr__(self, name, vector(getattr(self, name), name))
        if self.a[0] < 0 or min(self.a[1:]) <= 0:
            raise ValueError("Require a1 ≥ 0, a2 > 0, and a3 > 0.")
        if self.alpha != (90, -180, 0):
            raise ValueError("Analytical IK requires DH twists [90, −180, 0]°.")
        if any(direction not in (-1, 1) for direction in self.directions):
            raise ValueError("Each servo direction must be +1 or −1.")
        if any(sign not in (-1, 1) for sign in self.xyz_signs):
            raise ValueError("Each Cartesian axis sign must be +1 or −1.")
        if not isfinite(self.near_threshold) or self.near_threshold <= 1e-10:
            raise ValueError("Near-singular threshold must be finite and greater than 1e-10.")

    @classmethod
    def load(cls, path=CONFIG_PATH):
        with open(path, encoding="utf-8") as source:
            config = json.load(source)
        return cls(a=config["a_mm"], d=config["d_mm"], alpha=config["alpha_deg"],
                   offsets=config["servo_offsets_deg"], directions=config["servo_directions"],
                   xyz_signs=config["cartesian_axis_signs"],
                   near_threshold=config["near_singular_threshold"])

    @property
    def reach(self):
        return self.a[1] + self.a[2]

    @property
    def dh_limits(self):
        return tuple(tuple(sorted(((lower-offset)/direction, (upper-offset)/direction)))
                     for (lower, upper), offset, direction in zip(JOINT_LIMITS, self.offsets, self.directions))

    def servo_to_dh(self, servo):
        return tuple((angle-offset)/direction for angle, offset, direction in
                     zip(vector(servo, "Servo angles"), self.offsets, self.directions))

    def position_to_user(self, position):
        """DH/world coordinates -> app XYZ. This changes no actuator commands."""
        return tuple(sign*value for sign, value in zip(self.xyz_signs, vector(position, "XYZ")))

    def position_to_dh(self, position):
        # A diagonal +/-1 axis transformation is its own inverse.
        return self.position_to_user(position)

    def dh_to_servo(self, q):
        servo = tuple(offset+direction*angle for angle, offset, direction in
                      zip(vector(q, "DH angles"), self.offsets, self.directions))
        for index, (value, (lower, upper)) in enumerate(zip(servo, JOINT_LIMITS)):
            if not lower-1e-9 <= value <= upper+1e-9:
                raise KinematicsError("JOINT_LIMIT", f"J{index+1} would be {value:.3f}°; allowed {lower:g}–{upper:g}°.")
        # Only remove floating-point noise at a boundary, never clip a solution.
        return tuple(min(upper, max(lower, value)) for value, (lower, upper) in zip(servo, JOINT_LIMITS))


def dh(a, d, alpha, theta):
    c, s, ca, sa = cos(radians(theta)), sin(radians(theta)), cos(radians(alpha)), sin(radians(alpha))
    return np.array([[c, -s*ca, s*sa, a*c], [s, c*ca, -c*sa, a*s],
                     [0, sa, ca, d], [0, 0, 0, 1]], dtype=float)


def forward(q, robot):
    """Return T03, four frame origins, and the position Jacobian (rad/s)."""
    q = vector(q, "DH angles")
    chain = [np.eye(4)]
    for a, d, alpha, theta in zip(robot.a, robot.d, robot.alpha, q):
        chain.append(chain[-1] @ dh(a, d, alpha, theta))
    position = chain[-1][:3, 3]
    jacobian = np.column_stack([np.cross(t[:3, 2], position-t[:3, 3]) for t in chain[:3]])
    return chain[-1], np.array([t[:3, 3] for t in chain]), jacobian


def conditioning(q, robot):
    minimum = float(np.linalg.svd(forward(q, robot)[2]/robot.reach, compute_uv=False)[-1])
    return minimum, minimum <= 1e-10, 1e-10 < minimum <= robot.near_threshold


@dataclass(frozen=True)
class Solution:
    dh_angles: tuple
    servo_angles: tuple
    position_error: float
    sigma_min: float
    branch: str


def _equivalent(angle, limits, reference):
    angle = (angle+180) % 360-180
    minimum = ceil((limits[0]-angle-1e-9)/360)
    maximum = floor((limits[1]-angle+1e-9)/360)
    if minimum <= maximum:
        return angle+360*max(minimum, min(maximum, round((reference-angle)/360)))
    return angle


def inverse(target, robot, reference_servo=HOME_JOINTS):
    """Choose the nearest valid, nonsingular branch, independently checked by DH FK."""
    target = np.asarray(vector(target, "XYZ target"))
    reference = robot.servo_to_dh(reference_servo)
    a0, length_a, length_b = robot.a
    delta = robot.d[1]-robot.d[2]
    rho = hypot(target[0], target[1])
    height = target[2]-robot.d[0]
    tolerance = 64*np.spacing(max(robot.reach, *map(abs, target), *map(abs, robot.d)))
    if rho < abs(delta)-tolerance:
        raise KinematicsError("UNREACHABLE_AXIAL", "Target is inside the excluded central cylinder (XY radius < |d2 − d3|).")
    radius_abs = sqrt(max(0.0, (rho-abs(delta))*(rho+abs(delta))))
    radii = (0.0,) if radius_abs <= tolerance else (radius_abs, -radius_abs)
    geometric, within_any, limited, safe = False, False, [], []
    for base_index, radius in enumerate(radii):
        distance = hypot(radius-a0, height)
        if not abs(length_a-length_b)-tolerance <= distance <= robot.reach+tolerance:
            continue
        geometric = True
        if rho <= tolerance and delta == 0:
            yaw = min(robot.dh_limits[0][1], max(robot.dh_limits[0][0], reference[0]))
        else:
            yaw = degrees(atan2(target[1], target[0])+atan2(delta, radius))
        if distance <= tolerance and abs(length_a-length_b) <= tolerance:
            shoulder = min(robot.dh_limits[1][1], max(robot.dh_limits[1][0], reference[1]))
            candidates = [(yaw, shoulder, 180.0)]
        else:
            value = ((distance/robot.reach)**2-(length_a/robot.reach)**2-
                     (length_b/robot.reach)**2)/(2*(length_a/robot.reach)*(length_b/robot.reach))
            value = max(-1.0, min(1.0, value))
            elbow_abs = atan2(sqrt(max(0.0, (1-value)*(1+value))), value)
            candidates = []
            for sign in (1, -1):
                elbow = sign*elbow_abs
                shoulder = atan2(height, radius-a0)-atan2(length_b*sin(elbow), length_a+length_b*cos(elbow))
                candidates.append((yaw, degrees(shoulder), -degrees(elbow)))
        for candidate in candidates:
            q = tuple(_equivalent(angle, limit, ref) for angle, limit, ref in
                      zip(candidate, robot.dh_limits, reference))
            try:
                servo = robot.dh_to_servo(q)
            except KinematicsError:
                continue
            within_any = True
            error = float(np.linalg.norm(forward(q, robot)[0][:3, 3]-target))
            if error > 1e-8*robot.reach:
                continue
            sigma, singular, near = conditioning(q, robot)
            limited.append((singular, near))
            if not singular and not near:
                safe.append(Solution(q, servo, error, sigma, f"Base {base_index+1}"))
    if not geometric:
        raise KinematicsError("UNREACHABLE", "Target is outside the geometric workspace; neither base-heading branch reaches it.")
    if not safe:
        if limited:
            code = "SINGULAR" if all(row[0] for row in limited) else "NEAR_SINGULAR"
            raise KinematicsError(code, "All physically allowed IK branches are singular or too close to a singularity.")
        if within_any:
            raise KinematicsError("FK_MISMATCH", "IK candidates failed the independent standard-DH FK check.")
        raise KinematicsError("JOINT_LIMIT", "Geometric solutions exist, but none satisfy the servo joint limits after conversion.")
    return min(safe, key=lambda solution: sum((value-ref)**2 for value, ref in
                                             zip(solution.servo_angles, reference_servo)))
