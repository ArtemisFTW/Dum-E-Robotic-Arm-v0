#!/usr/bin/env python3
"""Check the one-command GUI launcher and clean shutdown without serial I/O."""
import os
from pathlib import Path
import signal
import subprocess
from time import monotonic

import rclpy
from dum_e_control.command import Client


def main():
    root = Path(__file__).resolve().parents[1]
    folder = root/'test-results'
    folder.mkdir(exist_ok=True)
    path = folder/'gui-launch.log'
    rclpy.init(args=[])
    client = Client('/dum_e')
    with path.open('w+') as log:
        launch = subprocess.Popen([str(root/'gui.sh')], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            client.wait(lambda: client.kinematics is not None and client.serial is not None, timeout=10)
            client.wait(lambda: 'arm_console' in client.get_node_names(), timeout=10)
            settle = monotonic()+1.0
            client.wait(lambda: monotonic() >= settle)
            assert not client.serial['enabled'] and not client.kinematics['have_target']
            children = Path(f'/proc/{launch.pid}/task/{launch.pid}/children').read_text().split()
            gui_pids = [int(pid) for pid in children if b'/lib/dum_e_control/gui\x00' in Path(f'/proc/{pid}/cmdline').read_bytes()]
            assert len(gui_pids) == 1, f'Could not identify GUI process among {children}'
            os.kill(gui_pids[0], signal.SIGTERM)
            assert launch.wait(timeout=10) == 0
            log.flush()
            log.seek(0)
            output = log.read()
            assert 'Traceback' not in output and '[ERROR]' not in output and 'process has died' not in output, output
            print('PASS: gui.sh starts the original console and both ROS nodes without selecting a target/opening serial; GUI exit shuts down all three cleanly')
        finally:
            if launch.poll() is None:
                launch.send_signal(signal.SIGINT)
                launch.wait(timeout=10)
            client.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
