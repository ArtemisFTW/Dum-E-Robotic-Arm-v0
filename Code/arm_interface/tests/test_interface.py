"""Checks calibrated FK/IK, physical limits and command preservation in the UI."""

from pathlib import Path
from dataclasses import replace
from time import monotonic
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from model import InterfaceState, JOINT_LIMITS
from app import ArmConsole
from kinematics import forward
from uart_transport import SerialStreamer
from test_uart_transport import FakeSerial, wait_for


class StateTests(unittest.TestCase):
    def test_home_and_reversed_j1_servo_to_dh_mapping(self):
        state = InterfaceState()
        self.assertEqual(state.joints, [90, 45, 180])
        self.assertEqual(state.dh_angles, (0, 45, 90))
        self.assertEqual(state.packet, "90,45,180\n")
        self.assertAlmostEqual(state.position[0], 10.35+230.7/2**.5)
        self.assertAlmostEqual(state.position[1], -.04)
        self.assertAlmostEqual(state.position[2], 45.74+20.7/2**.5)
        for servo, angle in ((0, -90), (90, 0), (180, 90)):
            self.assertEqual(state.robot.servo_to_dh((servo, servo, servo)), (-angle, servo, angle))
            self.assertEqual(state.robot.dh_to_servo((-angle, servo, angle)), (servo, servo, servo))

    def test_exact_frame_and_newline(self):
        state = InterfaceState()
        for index, value in enumerate((90, 45, 60)):
            state.set_joint(index, str(value))
        self.assertEqual(state.packet.encode("ascii"), b"90,45,60\n")
        state.set_joint(0, "23.5")
        self.assertEqual(state.packet, "24,45,60\n")
        state.set_joint(0, "24.5")
        self.assertEqual(state.packet, "25,45,60\n")
        self.assertTrue(all(type(value) is int for value in state.joints))

    def test_fractional_initial_commands_are_normalized_to_integers(self):
        state = InterfaceState(joints=[90.4, 44.5, 59.9])
        self.assertEqual(state.joints, [90, 45, 60])
        self.assertEqual(state.packet, "90,45,60\n")

    def test_packet_rejects_unexpected_fractional_state(self):
        state = InterfaceState()
        state.joints[0] = 90.5
        with self.assertRaises(ValueError):
            _ = state.packet

    def test_every_joint_boundary(self):
        state = InterfaceState()
        for index, limits in enumerate(JOINT_LIMITS):
            for limit in limits:
                state.set_joint(index, str(limit))
                self.assertEqual(state.joints[index], limit)
            for bad in (limits[0] - 0.001, limits[1] + 0.001):
                previous = state.packet
                with self.assertRaises(ValueError):
                    state.set_joint(index, str(bad))
                self.assertEqual(state.packet, previous)

    def test_invalid_joint_inputs_hold_previous(self):
        state = InterfaceState()
        state.set_joint(1, "45")
        for value in ("", "abc", "nan", "inf", "-inf", "1e9999"):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    state.set_joint(1, value)
                self.assertEqual(state.packet, "90,45,180\n")

    def test_ik_solves_valid_target_and_holds_on_invalid_target(self):
        state = InterfaceState()
        state.set_joint(2, "60")
        state.set_mode("IK")
        target = state.robot.position_to_user(forward((20, 30, -50), state.robot)[0][:3, 3])
        state.set_target_vector(target)
        self.assertEqual(state.packet, "70,30,40\n")
        with self.assertRaises(ValueError):
            state.set_target_vector((500, 500, 500))
        self.assertEqual(state.target, [500, 500, 500])
        self.assertEqual(state.packet, "70,30,40\n")
        with self.assertRaises(ValueError):
            state.set_joint(0, "20")

    def test_disabled_target_bank_and_pause(self):
        state = InterfaceState()
        for mode in ("Jog", "FK"):
            state.set_mode(mode)
            with self.assertRaises(ValueError):
                state.set_target(0, "20")
        state.paused = True
        with self.assertRaises(ValueError):
            state.set_joint(0, "20")
        state.set_mode("IK")
        with self.assertRaises(ValueError):
            state.set_target(0, "20")
        self.assertEqual(state.packet, "90,45,180\n")

    def test_invalid_cartesian_entry_preserves_target_and_packet(self):
        state = InterfaceState(mode="IK")
        original = list(state.target)
        for value in ("nan", "inf", "bad", "500.001", "-500.001"):
            with self.assertRaises(ValueError):
                state.set_target(0, value)
            self.assertEqual(state.target, original)
            self.assertEqual(state.packet, "90,45,180\n")


class WidgetTests(unittest.TestCase):
    def setUp(self):
        self.app = ArmConsole()
        self.app.withdraw()
        self.app.update_idletasks()

    def tearDown(self):
        if self.app.toast_job is not None:
            self.app.after_cancel(self.app.toast_job)
        self.app.destroy()

    def test_mode_switch_locks_correct_controls(self):
        for mode in ("Jog", "FK", "IK", "Jog"):
            self.app._set_mode(mode)
            self.assertTrue(all(row.enabled == (mode != "IK") for row in self.app.joint_rows))
            self.assertTrue(all(row.enabled == (mode == "IK") for row in self.app.cartesian_rows))

    def test_entry_slider_packet_sync_and_error_banner(self):
        row = self.app.joint_rows[1]
        row.text.set("45")
        row._commit()
        self.assertEqual(row.variable.get(), 45)
        self.assertEqual(self.app.packet_label.cget("text"), "90,45,180")
        row.text.set("181")
        row._commit()
        self.assertEqual(row.text.get(), "45")
        self.assertEqual(row.variable.get(), 45)
        self.assertIn("INPUT ERROR", self.app.banner_label.cget("text"))
        self.assertEqual(self.app.state_model.packet, "90,45,180\n")

    def test_joint_entry_and_slider_show_only_integer_commands(self):
        row = self.app.joint_rows[0]
        row.text.set("90.5")
        row._commit()
        self.assertEqual(row.text.get(), "91")
        self.assertEqual(row.variable.get(), 91)
        self.assertIs(type(row.variable.get()), int)
        self.assertEqual(self.app.packet_label.cget("text"), "91,45,180")
        row._slide("123.6")
        self.assertEqual(row.text.get(), "124")
        self.assertEqual(row.variable.get(), 124)
        self.assertEqual(self.app.state_model.packet, "124,45,180\n")
        self.assertEqual(self.app.dh_label.cget("text"), "DH θ1, θ2, θ3:  (-34, 45, 90)°")

    def test_cartesian_edit_does_not_change_command(self):
        self.app._joint_changed(2, "60")
        self.app._set_mode("IK")
        self.app.cartesian_rows[0].text.set("250")
        self.app.cartesian_rows[0]._commit()
        self.assertEqual(self.app.state_model.target[0], 250)
        self.assertEqual(self.app.packet_label.cget("text"), "90,45,60")
        self.assertIn("Holding the last", self.app.banner_label.cget("text"))

    def test_reachable_ik_slider_and_entry_stage_until_solve_button(self):
        self.app._joint_changed(2, "40")
        self.app._set_mode("IK")
        original, revision = self.app.state_model.packet, self.app.state_model.revision
        robot = self.app.state_model.robot
        target = robot.position_to_user(forward((20,30,-50),robot)[0][:3,3])
        self.app.cartesian_rows[0]._slide(str(round(target[0],1)))
        for row, value in zip(self.app.cartesian_rows,target):
            row.text.set(str(value))
            row._commit()  # Enter/FocusOut must also stage rather than solve.
        self.assertEqual(self.app.state_model.packet,original)
        self.assertEqual(self.app.state_model.revision,revision)
        self.assertTrue(self.app.state_model.target_dirty)
        self.assertIn("Press Solve XYZ",self.app.banner_label.cget("text"))
        self.app.apply_button.invoke()
        self.assertEqual(self.app.state_model.packet,"70,30,40\n")
        self.assertFalse(self.app.state_model.target_dirty)

    def test_loop_list_mixed_inputs_edits_and_invalid_pose_do_not_send(self):
        original = self.app.state_model.packet
        self.app._show_page("Loop")
        self.assertEqual(self.app.page,"Loop")
        for index in range(12):
            for field,value in zip(self.app.pose_fields,(70+index,30,40)):
                field.set(str(value))
            self.app._add_pose()
        self.assertEqual(len(self.app.poses),12)
        self.assertEqual(len(self.app.pose_tree.get_children()),12)
        self.app.pose_source.set("XYZ")
        for field,value in zip(self.app.pose_fields,self.app.poses[0].xyz):
            field.set(str(value))
        self.app._add_pose()
        self.assertEqual(self.app.poses[-1].joints,(70,30,40))
        self.app.pose_source.set("Joints")
        self.app.pose_fields[0].set("181")
        self.app._add_pose()
        self.assertEqual(len(self.app.poses),13)
        self.app.pose_source.set("XYZ")
        for field in self.app.pose_fields:
            field.set("500")
        self.app._update_pose()
        self.assertEqual(len(self.app.poses),13)
        self.assertEqual(self.app.poses[-1].joints,(70,30,40))
        self.app.pose_tree.selection_set("0")
        self.app.pose_source.set("Joints")
        for field,value in zip(self.app.pose_fields,(65,35,45)):
            field.set(str(value))
        self.app._update_pose()
        self.assertEqual(self.app.poses[0].joints,(65,35,45))
        self.app._move_pose(1)
        self.assertEqual(self.app.poses[1].joints,(65,35,45))
        self.app._remove_pose()
        self.assertEqual(len(self.app.poses),12)
        self.assertEqual(self.app.state_model.packet,original)
        self.assertFalse(self.app.streamer.running)

    def test_loop_send_wait_hold_cycle_stop_and_connection_failure(self):
        for values in ((70,30,40),(110,60,100)):
            for field,value in zip(self.app.pose_fields,values):
                field.set(str(value))
            self.app._add_pose()
        original = self.app.state_model.packet
        self.app._start_loop() # Offline must not replace the held target.
        self.assertFalse(self.app.loop_player.running)
        self.assertEqual(self.app.state_model.packet,original)
        fake = FakeSerial()
        self.app.streamer = SerialStreamer(lambda *_:fake,period=.01,startup_delay=0)
        self.app.port.set("FAKE")
        self.app._toggle_connection()

        self.assertTrue(wait_for(lambda:self.app.streamer.status().completed > 0))
        now = monotonic()
        with patch("app.monotonic",side_effect=lambda:now):
            self.app._start_loop()
            self.assertTrue(self.app.loop_player.running)
            self.assertTrue(all(not row.enabled for row in self.app.joint_rows+self.app.cartesian_rows))
            self.assertTrue(wait_for(lambda:self.app.streamer.status().last_done == "70,30,40"))
            self.app._poll_loop(replace(self.app.streamer.status(),last_ack_time=now))
            self.assertEqual(self.app.loop_player.phase,"holding")
            self.app._joint_changed(0,"0")
            self.assertEqual(self.app.state_model.packet,"70,30,40\n")
            now += 1.999
            self.app._poll_loop(replace(self.app.streamer.status(),last_ack_time=now))
            self.assertEqual(self.app.state_model.packet,"70,30,40\n")
            now += .01
            self.app._poll_loop(replace(self.app.streamer.status(),last_ack_time=now))
            self.assertTrue(wait_for(lambda:self.app.streamer.status().last_done == "110,60,100"))
            self.app._poll_loop(replace(self.app.streamer.status(),last_ack_time=now))
            now += 2.01
            self.app._poll_loop(replace(self.app.streamer.status(),last_ack_time=now))
            self.assertEqual(self.app.state_model.packet,"70,30,40\n")
            self.assertEqual(self.app.loop_player.cycle,2)
            self.app._stop_loop()
            self.assertFalse(self.app.loop_player.running)
            self.assertTrue(all(row.enabled for row in self.app.joint_rows))
            self.assertEqual(self.app.state_model.packet,"70,30,40\n")
        self.app._toggle_connection()

    def test_firmware_error_stops_loop_before_advancing_at_hold_boundary(self):
        from pose_loop import make_pose
        from uart_transport import Status
        self.app.poses = [make_pose("Joints", values, self.app.state_model.robot, self.app.state_model.joints)
                          for values in ((70,30,40),(110,60,100))]
        self.app._refresh_pose_list()
        now = monotonic()
        status = Status(state="streaming",last_ack_time=now)
        first = self.app.loop_player.start(self.app.poses,status,now)
        self.app._send_loop_pose(first)
        self.app.loop_player.phase = "holding"
        self.app.loop_player.hold_until = now-1
        self.app.streamer.events.put("ERR,PWM_FAILURE_RESTART_REQUIRED")
        self.app.after_cancel(self.app.transport_job)
        with patch.object(self.app.streamer,"status",return_value=status):
            self.app._poll_transport()
        self.assertFalse(self.app.loop_player.running)
        self.assertEqual(self.app.loop_player.index,0)
        self.assertEqual(self.app.state_model.packet,"70,30,40\n")
        self.assertIn("PWM_FAILURE",self.app.banner_label.cget("text"))

    def test_fk_readout_uses_inverse_servo_mapping(self):
        self.app._set_mode("FK")
        for index, value in enumerate((110, 60, 40)):
            self.app._joint_changed(index, str(value))
        robot = self.app.state_model.robot
        expected = robot.position_to_user(forward((-20, 60, -50), robot)[0][:3, 3])
        for row, value in zip(self.app.cartesian_rows, expected):
            self.assertAlmostEqual(row.variable.get(), value)
        self.assertEqual(self.app.state_model.dh_angles, (-20, 60, -50))

    def test_photo_pose_readout_and_servo_calculation_match_mounting(self):
        self.app._set_mode("FK")
        self.app._joint_changed(1, "0")
        self.app._joint_changed(2, "90")
        self.assertEqual(self.app.packet_label.cget("text"), "90,0,90")
        self.assertEqual(self.app.dh_label.cget("text"), "DH θ1, θ2, θ3:  (0, 0, 0)°")
        self.assertEqual([row.text.get() for row in self.app.cartesian_rows], ["241.1", "0", "45.7"])
        self.assertEqual(self.app.packet_note.cget("text"),
                         "Achieved app XYZ: (241.05, -0.04, 45.74) mm")
        self.assertEqual(self.app.calculation_labels[1].cget("text"), "J2 = round(0 + (0)) = 0°")
        self.assertFalse(self.app.streamer.running)

    def test_calculation_panel_tracks_integer_commands_and_reversed_j1(self):
        self.assertEqual([row.variable.get() for row in self.app.joint_rows], [90, 45, 180])
        self.assertEqual(self.app.packet_label.cget("text"), "90,45,180")
        self.assertEqual(self.app.dh_label.cget("text"), "DH θ1, θ2, θ3:  (0, 45, 90)°")
        self.assertEqual(self.app.calculation_labels[0].cget("text"),
                         "J1 = round(90 − (0)) = 90°")
        self.assertEqual(self.app.last_sent_label.cget("text"), "Last UART write: none")
        for index, value in enumerate((70, 60, 40)):
            self.app._joint_changed(index, str(value))
        self.assertEqual([widget.cget("text") for widget in self.app.calculation_labels], [
            "J1 = round(90 − (20)) = 70°",
            "J2 = round(0 + (60)) = 60°",
            "J3 = round(90 + (-50)) = 40°",
        ])
        self.app._joint_changed(0, "70.5")
        self.assertEqual(self.app.calculation_labels[0].cget("text"),
                         "J1 = round(90 − (19)) = 71°")
        self.assertEqual(self.app.packet_label.cget("text"), "71,60,40")
        # Changing the preview while offline must not claim a UART write.
        self.assertEqual(self.app.last_sent_label.cget("text"), "Last UART write: none")

    def test_j1_live_jog_endpoints_have_corrected_fk_direction(self):
        fake = FakeSerial()
        self.app.streamer = SerialStreamer(lambda *_: fake, period=.01, startup_delay=0)
        self.app.port.set("FAKE")
        self.app._toggle_connection()
        self.assertEqual(fake.writes.get(timeout=1), b"90,45,180\n")
        self.app._joint_changed(1, "0")  # Photograph's straight horizontal pose.
        self.app._joint_changed(2, "90")
        for command, expected_y, expected_dh in ((180, -241.05, -90), (0, 241.05, 90)):
            self.app._joint_changed(0, str(command))
            self.assertAlmostEqual(self.app.state_model.position[1], expected_y, places=8)
            self.assertEqual(self.app.state_model.dh_angles[0], expected_dh)
            self.assertTrue(wait_for(lambda: self.app.streamer.status().last_ack == f"{command},0,90"))
        self.app._toggle_connection()

    def test_unchanged_coordinate_editors_do_not_round_or_move_pose(self):
        self.app._joint_changed(2, "45")
        self.app._set_mode("IK")
        packet, revision = self.app.state_model.packet, self.app.state_model.revision
        target = list(self.app.state_model.target)
        for row in self.app.cartesian_rows:
            row._commit()
        self.assertEqual(self.app.state_model.packet, packet)
        self.assertEqual(self.app.state_model.target, target)
        self.assertEqual(self.app.state_model.revision, revision)

    def test_apply_xyz_updates_servo_sliders_and_error_persists(self):
        # Prefer the known negative-elbow branch from a nearby reference pose.
        self.app._joint_changed(2, "40")
        self.app._set_mode("IK")
        robot = self.app.state_model.robot
        target = robot.position_to_user(forward((20, 30, -50), robot)[0][:3, 3])
        for row, value in zip(self.app.cartesian_rows, target):
            row.text.set(str(value))
        self.app._apply_target()
        self.assertEqual(self.app.packet_label.cget("text"), "70,30,40")
        self.assertEqual([row.variable.get() for row in self.app.joint_rows], [70, 30, 40])
        calculations = [widget.cget("text") for widget in self.app.calculation_labels]
        for row in self.app.cartesian_rows:
            row.text.set("500")
        self.app._apply_target()
        self.app._baseline_message()
        self.assertEqual(self.app.packet_label.cget("text"), "70,30,40")
        self.assertEqual([widget.cget("text") for widget in self.app.calculation_labels], calculations)
        self.assertIn("UNREACHABLE", self.app.banner_label.cget("text"))
        self.assertIsNone(self.app.toast_job)

    def test_pause_preserves_setpoints_and_restore_mode(self):
        self.app._set_mode("IK")
        self.app._toggle_pause()
        self.assertTrue(all(not row.enabled for row in self.app.joint_rows + self.app.cartesian_rows))
        self.app._toggle_pause()
        self.assertTrue(all(row.enabled for row in self.app.cartesian_rows))
        self.assertTrue(all(not row.enabled for row in self.app.joint_rows))

    def test_no_automatic_connection_and_port_is_required(self):
        self.assertFalse(self.app.streamer.running)
        self.app._toggle_connection()
        self.assertIn("Select the ESP32 COM port", self.app.banner_label.cget("text"))
        self.assertEqual(self.app.revision_label.cget("text"), "0 edits · 0 sent")

    def test_live_jog_stream_and_ik_failure_hold(self):
        fake = FakeSerial()
        self.app.streamer = SerialStreamer(lambda *_: fake, period=.01, startup_delay=0)
        self.app.port.set("FAKE")
        self.app._toggle_connection()
        self.assertEqual(fake.writes.get(timeout=1), b"90,45,180\n")
        self.app._joint_changed(0, "120")
        self.assertTrue(wait_for(lambda: self.app.streamer.status().last_ack == "120,45,180"))
        self.app._joint_changed(2, "40")
        self.assertTrue(wait_for(lambda: self.app.streamer.status().last_ack == "120,45,40"))
        self.app._refresh_last_sent(self.app.streamer.status())
        self.assertEqual(self.app.last_sent_label.cget("text"), "Last UART write: 120,45,40\\n")
        self.app._toggle_pause()
        self.assertTrue(self.app.streamer.running)
        self.app._toggle_pause()
        self.app._set_mode("IK")
        self.app._target_changed(0, "500")
        self.assertEqual(self.app.state_model.packet, "120,45,40\n")
        self.assertTrue(wait_for(lambda: self.app.streamer.status().last_ack == "120,45,40"))
        self.app._refresh_last_sent(self.app.streamer.status())
        self.assertEqual(self.app.last_sent_label.cget("text"), "Last UART write: 120,45,40\\n")
        self.app._toggle_connection()
        self.assertFalse(self.app.streamer.running)


if __name__ == "__main__":
    unittest.main()
