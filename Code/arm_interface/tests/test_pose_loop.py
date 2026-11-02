from dataclasses import replace
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from model import InterfaceState
from kinematics import Robot
from pose_loop import LoopPlayer, make_pose
from uart_transport import Status


def live(now, completed=0, done=""):
    return Status(state="streaming", last_ack_time=now, completed=completed, last_done=done)


class PoseLoopTests(unittest.TestCase):
    def setUp(self):
        self.robot = Robot.load()
        self.reference = (90,45,180)
        self.poses = [make_pose("Joints", values, self.robot, self.reference)
                      for values in ((90,45,180), (70,30,40), (110,60,100))]

    def test_mixed_joint_and_xyz_inputs_use_current_calibration_without_moving_state(self):
        state = InterfaceState()
        original = state.packet
        joint = make_pose("Joints", (70.5,30,40), self.robot, self.reference)
        self.assertEqual(joint.joints, (71,30,40))
        xyz = make_pose("XYZ", self.poses[1].xyz, self.robot, self.poses[1].joints)
        self.assertEqual(xyz.joints, (70,30,40))
        self.assertEqual(xyz.packet, "70,30,40\n")
        self.assertEqual(state.packet, original)

    def test_invalid_limits_unreachable_and_singular_xyz_are_not_added(self):
        for source, values in (("Joints", (181,45,90)), ("Joints", (90,-.1,90)),
                               ("Joints", (90,45,"nan")), ("XYZ", (500,500,500)),
                               ("XYZ", (241.05,-.04,45.74)), ("XYZ", (1,2)),
                               ("unknown", (1,2,3))):
            with self.subTest(source=source, values=values), self.assertRaises(ValueError):
                make_pose(source, values, self.robot, self.reference)

    def test_waits_for_matching_fresh_done_then_holds_full_two_seconds(self):
        player = LoopPlayer()
        self.assertEqual(player.start(self.poses, live(100, 7, "90,45,180"), 100), self.poses[0])
        # A stale matching DONE and a fresh DONE for the wrong pose are insufficient.
        self.assertIsNone(player.tick(live(101,7,"90,45,180"),101))
        self.assertEqual(player.phase,"moving")
        self.assertIsNone(player.tick(live(102,8,"70,30,40"),102))
        self.assertEqual(player.phase,"moving")
        self.assertIsNone(player.tick(live(106,9,"90,45,180"),106))
        self.assertEqual(player.phase,"holding")
        self.assertIsNone(player.tick(live(107.999,40,"90,45,180"),107.999))
        self.assertEqual(player.tick(live(108,41,"90,45,180"),108), self.poses[1])
        self.assertEqual(player.phase,"moving")
        self.assertEqual(player.baseline_completed,41)

    def test_more_than_ten_poses_cycle_in_order_and_single_pose_repeats(self):
        for poses in (self.poses[:1], self.poses*4):
            player = LoopPlayer()
            now, count = 100.0, 0
            self.assertEqual(player.start(poses,live(now),now),poses[0])
            for index in range(len(poses)*2):
                expected = poses[index%len(poses)]
                now += .1
                count += 1
                player.tick(live(now,count,expected.packet.strip()),now)
                now += 2
                next_pose = player.tick(live(now,count,expected.packet.strip()),now)
                self.assertEqual(next_pose,poses[(index+1)%len(poses)])
            self.assertEqual(player.cycle,3)

    def test_no_connection_empty_list_invalid_frames_and_timeout_stop(self):
        player = LoopPlayer()
        for poses, status in (([],live(100)), (self.poses,Status()),
                              ([replace(self.poses[0],joints=(181,0,0))],live(100))):
            with self.assertRaises(ValueError):
                player.start(poses,status,100)
            self.assertFalse(player.running)
        player.start(self.poses,live(100),100)
        with self.assertRaisesRegex(ValueError,"DONE"):
            player.tick(live(120.01),120.01)
        self.assertFalse(player.running)
        self.assertEqual(player.index,0)

    def test_link_error_stale_ack_and_stop_do_not_advance(self):
        for status in (Status(state="error",error="removed"),live(100)):
            player = LoopPlayer()
            player.start(self.poses,live(100),100)
            with self.assertRaises(ValueError):
                player.tick(status,103)
            self.assertFalse(player.running)
            self.assertEqual(player.index,0)
        player.start(self.poses,live(110),110)
        player.stop()
        self.assertIsNone(player.tick(live(200,5,"90,45,180"),200))
        self.assertEqual(player.index,0)


if __name__ == "__main__":
    unittest.main()
