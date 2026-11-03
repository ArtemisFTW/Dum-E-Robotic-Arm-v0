#!/usr/bin/env python3
"""Smoke-test the installed launch file and user-facing commands in dry run."""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
from time import monotonic, sleep


def main():
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryFile(mode='w+') as log:
        launch = subprocess.Popen([str(root / 'run_ros2.sh')], cwd=root,
                                  stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            def command(*args, expected=0):
                result = subprocess.run([str(root / 'arm.sh'), *args], cwd=root,
                                        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=20)
                assert result.returncode == expected, result.stdout
                return result.stdout
            deadline = monotonic() + 15
            while True:
                try:
                    state = json.loads(command('status'))
                    break
                except (AssertionError, json.JSONDecodeError):
                    assert monotonic() < deadline and launch.poll() is None, 'Launch did not become ready'
                    sleep(0.1)
            assert not state['kinematics']['have_target'] and not state['serial']['enabled']
            command('enable', expected=1)
            result = json.loads(command('servo', '70', '30', '40'))
            assert result['servo_degrees'] == [70, 30, 40]
            command('enable')
            state = json.loads(command('status'))
            assert state['serial']['phase'] == 'dry_run' and state['serial']['sent'] > 0
            result = json.loads(command('xyz', '0.1291673', '0.0469705', '0.2119948'))
            assert result['servo_degrees'] == [70, 30, 40]
            command('servo', '181', '30', '40', expected=1)
            command('dh', '0.1745329252', '0.8726646260', '-0.6981317008')
            state = json.loads(command('status'))
            assert state['kinematics']['servo_degrees'] == [80, 50, 50]
            output = command('loop', 'ros2_ws/src/dum_e_control/config/poses.yaml', '--cycles', '1', '--hold', '0.01')
            assert 'pose 3:' in output
            command('disable')
            assert not json.loads(command('status'))['serial']['enabled']
            print('PASS: installed launch defaults, enable guard, servo, XYZ, DH, invalid target, pose loop, disable and status')
        except Exception:
            log.seek(0)
            print(log.read())
            raise
        finally:
            if launch.poll() is None:
                # Let ros2 launch forward the signal once to each managed node.
                launch.send_signal(signal.SIGINT)
                try:
                    launch.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGTERM)
                    launch.wait(timeout=5)


if __name__ == '__main__':
    main()
