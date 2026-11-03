"""Original UI model API, with all FK/IK and servo mapping done by ROS."""
from collections import OrderedDict
import json
from math import degrees, radians

from .core.model import CARTESIAN_SLIDER_LIMITS, display, number, whole_degrees
from .core.kinematics import HOME_JOINTS, JOINT_LIMITS
from .gui_session import get_session, RemoteError

CONFIG_PATH = 'Active calibration supplied by /dum_e/solve_target'


class ModelInfo:
    def __init__(self, session, model_json):
        self.session = session
        for name, value in json.loads(model_json).items():
            setattr(self, name, value)
        self.conditions = OrderedDict()

    def remember(self, angles, response):
        self.conditions[tuple(angles)] = (response.sigma_min, response.singular, response.near_singular)
        while len(self.conditions) > 128:
            self.conditions.popitem(last=False)


def conditioning(q, robot):
    if tuple(q) not in robot.conditions:
        response = robot.session.preview('dh', [radians(v) for v in q], HOME_JOINTS)
        robot.remember(q, response)
    return robot.conditions[tuple(q)]


class InterfaceState:
    def __init__(self, *, robot=None, joints=None, mode='Jog', **kwargs):
        self.session = robot.session if robot is not None else get_session()
        initial = list(joints) if joints is not None else list(self.session.kinematics['servo_degrees'])
        response = self.session.preview('servo', initial, initial)
        self.robot = robot or ModelInfo(self.session, response.model_json)
        self.mode = mode
        self.paused = False
        self.revision = 0
        self.ik_error = self.ik_error_code = self.branch = ''
        self.target_dirty = False
        self._assign(response)
        self.target = list(self.position)

    def _assign(self, response):
        self.joints = [int(v) for v in response.degrees]
        self._dh_angles = tuple(degrees(v) for v in response.dh_radians)
        self._position = tuple(float(v) * 1000.0 for v in response.xyz_m)
        self._position_error = response.position_error_m * 1000.0
        self.robot.remember(self._dh_angles, response)

    @property
    def position(self):
        return self._position

    @property
    def dh_angles(self):
        return self._dh_angles

    @property
    def position_error(self):
        return self._position_error

    @property
    def packet(self):
        return ','.join(map(str, self.joints)) + '\n'

    def set_mode(self, mode):
        if mode not in ('Jog', 'FK', 'IK'):
            raise ValueError('Unknown control mode.')
        if mode == 'IK' and self.mode != 'IK':
            self.target = list(self.position)
            self.ik_error = self.ik_error_code = ''
            self.target_dirty = False
        self.mode = mode

    def accept_command(self, values):
        if len(values) != 3:
            raise ValueError('Enter three servo angles.')
        response = self.session.preview('servo', [number(v) for v in values], self.joints)
        if self.joints != list(response.degrees):
            self.revision += 1
        self._assign(response)
        self.target = list(self.position)
        self.ik_error = self.ik_error_code = self.branch = ''
        self.target_dirty = False

    def set_joint(self, index, text):
        if self.paused or self.mode == 'IK':
            raise ValueError('Joint input is currently locked.')
        values = list(self.joints)
        values[index] = number(text)
        self.accept_command(values)

    def set_target(self, index, text):
        if self.paused or self.mode != 'IK':
            raise ValueError('Cartesian input is available in IK mode while inputs are running.')
        value = number(text)
        if not CARTESIAN_SLIDER_LIMITS[0] <= value <= CARTESIAN_SLIDER_LIMITS[1]:
            raise ValueError('Cartesian controls span −500 to 500 mm; IK checks actual reach.')
        self.target[index] = value
        self.target_dirty = True
        self.ik_error = self.ik_error_code = self.branch = ''

    def set_target_vector(self, values):
        if self.paused or self.mode != 'IK':
            raise ValueError('Cartesian input is available in IK mode while inputs are running.')
        if len(values) != 3:
            raise ValueError('Enter all three Cartesian coordinates.')
        values = [number(v) for v in values]
        if any(not -500 <= v <= 500 for v in values):
            raise ValueError('Cartesian controls span −500 to 500 mm; IK checks actual reach.')
        self.target, self.target_dirty = values, True
        try:
            response = self.session.preview('xyz', [v / 1000.0 for v in values], self.joints)
        except RemoteError as error:
            self.ik_error, self.ik_error_code = str(error), error.code
            raise
        if self.joints != list(response.degrees):
            self.revision += 1
        self._assign(response)
        self.branch = response.branch
        self.ik_error = self.ik_error_code = ''
        self.target_dirty = False
