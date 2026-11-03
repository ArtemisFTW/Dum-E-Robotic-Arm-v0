"""Validated pose lists and a nonblocking, completion-driven loop player."""

from dataclasses import dataclass
from .gui_model import InterfaceState, number
from .gui_transport import validate_packet

HOLD_SECONDS = 2.0
ARRIVAL_TIMEOUT = 20.0


@dataclass(frozen=True)
class Pose:
    source: str
    values: tuple
    joints: tuple[int, int, int]
    xyz: tuple

    @property
    def packet(self):
        return ",".join(map(str, self.joints))+"\n"


def make_pose(source, values, robot, reference):
    """Solve on an isolated state: editing a pose list never moves the arm."""
    if source not in ("Joints", "XYZ") or len(values) != 3:
        raise ValueError("Choose Joints or XYZ and enter three values.")
    state = InterfaceState(robot=robot, joints=list(reference))
    if source == "Joints":
        state.accept_command(values)
        inputs = tuple(state.joints)  # Joint inputs and packets stay integers.
    else:
        inputs = tuple(number(value) for value in values)
        state.set_mode("IK")
        state.set_target_vector(inputs)
    return Pose(source, inputs, tuple(state.joints), tuple(state.position))


class LoopPlayer:
    def __init__(self):
        self.running = False
        self.poses = ()
        self.index = 0
        self.cycle = 1
        self.phase = "stopped"
        self.hold_until = 0.0

    @staticmethod
    def _check_link(status, now):
        if status.state != "streaming":
            raise ValueError("Connect UART and wait for STREAMING before sending the loop.")
        reference = status.last_ack_time or status.streaming_since
        if now-reference > 2.0:
            raise ValueError("No recent ESP32 ACK. Loop stopped at its current target.")

    def start(self, poses, status, now):
        if not poses:
            raise ValueError("Add at least one pose before sending the loop.")
        self._check_link(status, now)
        for pose in poses:
            validate_packet(pose.packet)
        self.poses = tuple(poses)
        self.index, self.cycle = 0, 1
        self.running = True
        return self._begin_pose(status, now)

    def _begin_pose(self, status, now):
        self.phase = "moving"
        self.started = now
        self.baseline_completed = status.completed
        return self.poses[self.index]

    def stop(self):
        self.running = False
        self.phase = "stopped"

    def tick(self, status, now):
        if not self.running:
            return None
        try:
            self._check_link(status, now)
            if self.phase == "moving":
                expected = self.poses[self.index].packet.rstrip("\n")
                if status.completed > self.baseline_completed and status.last_done == expected:
                    self.phase = "holding"
                    self.hold_until = now+HOLD_SECONDS
                elif now-self.started > ARRIVAL_TIMEOUT:
                    raise ValueError("No matching DONE from ESP32 within 20 s. Upload the speed-limited firmware; loop stopped.")
            elif now >= self.hold_until:
                self.index = (self.index+1) % len(self.poses)
                if self.index == 0:
                    self.cycle += 1
                return self._begin_pose(status, now)
        except ValueError:
            self.stop()
            raise
        return None

    def description(self, now):
        if not self.running:
            return "Stopped · 2-second hold after commanded PWM arrival"
        timing = (f"holding {max(0, self.hold_until-now):.1f} s" if self.phase == "holding" else
                  "moving · waiting for ESP32 DONE")
        return f"Cycle {self.cycle} · pose {self.index+1}/{len(self.poses)} · {timing}"
