#!/usr/bin/env python3
"""Drive the real Tk widgets and ROS nodes. Optional hardware test is home-only."""
import argparse
from pathlib import Path
import signal
import subprocess
from time import monotonic
from unittest.mock import patch

import rclpy
from PIL import ImageGrab

from dum_e_control.gui_session import GuiSession, install_session
from dum_e_control.gui_app import ArmConsole
from dum_e_control.gui_transport import DRY_RUN_PORT


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--hardware-port', help='If set, test only the existing home target on this real device')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    folder = root / 'test-results'
    folder.mkdir(exist_ok=True)
    log_path = folder / ('gui-hardware.log' if args.hardware_port else 'gui-dry-run.log')
    rclpy.init(args=[])
    app = session = launch = None
    with log_path.open('w') as log:
        try:
            launch = subprocess.Popen([str(root/'run_ros2.sh')], cwd=root,
                                      stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            session = GuiSession('/dum_e')
            install_session(session)
            # Calculation code is unavailable in the GUI process during this test.
            # Successful FK/IK therefore proves use of the actual service node.
            with patch('dum_e_control.core.kinematics.forward', side_effect=AssertionError('Local GUI FK called')), \
                 patch('dum_e_control.core.kinematics.inverse', side_effect=AssertionError('Local GUI IK called')), \
                 patch('dum_e_control.core.model.forward', side_effect=AssertionError('Local GUI model FK called')), \
                 patch('dum_e_control.core.model.inverse', side_effect=AssertionError('Local GUI model IK called')):
                app = ArmConsole()
                app.update()

                def wait(condition, timeout=8):
                    deadline = monotonic() + timeout
                    while not condition():
                        assert monotonic() < deadline, f'GUI condition timed out: {session.serial}'
                        app.update()
                        session.pump()

                assert app.state_model.joints == [90, 45, 180]
                assert not session.kinematics['have_target'] and not session.serial['enabled']
                assert len(app.joint_rows) == len(app.cartesian_rows) == 3

                if args.hardware_port:
                    app.port.set(args.hardware_port)
                    app._toggle_connection()
                    wait(lambda: session.serial['acknowledged'] >= 20)
                    assert not session.serial['dry_run']
                    assert session.serial['last_sent'] == session.serial['last_ack'] == session.serial['last_done'] == '90,45,180'
                    print(f"PASS: same GUI -> real ROS nodes -> ESP32 home; {session.serial['acknowledged']} ACKs / {session.serial['completed']} DONEs")
                else:
                    for index, value in enumerate([70, 30, 40]):
                        app._joint_changed(index, str(value))
                    assert app.state_model.joints == [70, 30, 40]
                    assert abs(app.state_model.position[0] - 129.1673) < 0.001
                    assert not session.kinematics['have_target']  # Offline preview remains read-only.
                    before = app.state_model.packet
                    app._joint_changed(0, '181')
                    assert app.state_model.packet == before
                    app._set_mode('IK')
                    for row, value in zip(app.cartesian_rows, [129.1673, 46.9705, 211.9948]):
                        row.text.set(str(value))
                    app._apply_target()
                    assert app.state_model.joints == [70, 30, 40] and not app.state_model.ik_error
                    for row in app.cartesian_rows:
                        row.text.set('500')
                    app._apply_target()
                    assert app.state_model.packet == before and app.state_model.ik_error
                    app._use_current_pose()
                    print('PASS: original Jog/FK/IK controls, remote calculations, offline preview and invalid-target hold')

                    app.port.set(DRY_RUN_PORT)
                    app._toggle_connection()
                    wait(lambda: session.serial['acknowledged'] >= 3)
                    assert session.serial['dry_run'] and app.streamer.running
                    try:
                        session.configure('/dev/should-not-open', False)
                    except ValueError:
                        pass
                    else:
                        raise AssertionError('Connection selection changed while enabled')
                    app._toggle_pause()
                    assert app.state_model.paused and str(app.joint_rows[0].entry.cget('state')) == 'disabled'
                    count = session.serial['sent']
                    wait(lambda: session.serial['sent'] >= count + 3)
                    app._toggle_pause()
                    print('PASS: Connect, clear dry-run indication, pause/resume and configure-while-enabled rejection')

                    accepted = session.kinematics['accepted']
                    for source, values in [('Joints', [90, 45, 180]), ('XYZ', [129.1673, 46.9705, 211.9948])]:
                        app.pose_source.set(source)
                        app._pose_input_changed()
                        for field, value in zip(app.pose_fields, values):
                            field.set(str(value))
                        app._add_pose()
                    assert len(app.poses) == 2 and session.kinematics['accepted'] == accepted
                    app._show_page('Loop')
                    app._start_loop()
                    wait(lambda: app.loop_player.phase == 'holding' and app.loop_player.index == 0)
                    wait(lambda: app.loop_player.index == 1, timeout=5)
                    wait(lambda: app.loop_player.phase == 'holding')
                    app._stop_loop()
                    assert not app.loop_player.running and app.streamer.running
                    app._show_page('Control')
                    app._set_mode('FK')
                    print('PASS: same mixed-pose Loop page, read-only list editing, matching DONE, two-second dwell and Stop loop')

                app._show_model()
                app.update()
                # Close the model dialog so the screenshot shows the actual console.
                for child in app.winfo_children():
                    if child.winfo_class() == 'Toplevel':
                        child.destroy()
                app.lift()
                app.update()
                settle = monotonic() + 0.4
                wait(lambda: monotonic() >= settle)  # Allow the window manager's dialog animation to finish.
                screenshot = folder / ('gui-hardware.png' if args.hardware_port else 'gui-dry-run.png')
                capture = ImageGrab.grab(bbox=(app.winfo_rootx(), app.winfo_rooty(),
                                             app.winfo_rootx()+app.winfo_width(), app.winfo_rooty()+app.winfo_height()))
                if capture.getbbox() is not None:
                    capture.save(screenshot)
                else:
                    if screenshot.exists():
                        screenshot.unlink()
                    print('Display capture unavailable (screen is blank); widget and ROS checks completed.')
                app._toggle_connection()
                wait(lambda: not session.serial['enabled'])
                app.destroy()
                app = None
                print(f'PASS: Model/mapping dialog and GUI Disconnect. Screenshot when available: {screenshot}')
        finally:
            if app is not None:
                app.destroy()
            if session is not None:
                session.disable_serial()
                session.destroy_node()
            install_session(None)
            if launch is not None and launch.poll() is None:
                launch.send_signal(signal.SIGINT)
                launch.wait(timeout=10)
            if rclpy.ok():
                rclpy.shutdown()
    print(f'GUI VERIFICATION PASSED. Log: {log_path}')


if __name__ == '__main__':
    main()
