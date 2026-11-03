#!/usr/bin/env python3
"""Real ESP32 ROS test using only the firmware's existing home target.

Opening serial may reset the board and cause its startup home movement.
No firmware is flashed and no off-home motion commands are sent.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
from time import monotonic

import rclpy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from dum_e_control.command import Client


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--duration', type=float, default=3.0)
    args = parser.parse_args()
    if not 1 <= args.duration <= 15:
        parser.error('--duration must be between 1 and 15 seconds')
    if not os.access(args.port, os.R_OK | os.W_OK):
        parser.error(f'No read/write permission for {args.port}')
    root = Path(__file__).resolve().parents[1]
    folder = root / 'test-results'
    folder.mkdir(exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    report_path = folder / f'esp32-home-{timestamp}.json'
    launch_log = folder / f'esp32-home-{timestamp}.log'
    report = {'utc': timestamp, 'port': args.port, 'baud': 115200,
              'servo_target': [90, 45, 180], 'passed': False,
              'physical_position_measured': False, 'firmware_flashed': False,
              'rx': []}
    rclpy.init(args=[])
    node = Client('/dum_e')
    rx = node.create_subscription(String, 'serial_bridge/rx',
        lambda msg: report['rx'].append(msg.data), 100)
    launch = None
    try:
        with launch_log.open('w') as log:
            launch = subprocess.Popen([str(root / 'run_ros2.sh'), 'dry_run:=false', f'port:={args.port}'],
                                      cwd=root, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            node.wait(lambda: node.kinematics is not None and node.serial is not None, timeout=10)
            if node.serial['dry_run'] or node.serial['enabled']:
                raise RuntimeError('Expected a disabled hardware bridge on an isolated ROS domain.')
            result = node.resolve('servo', [90, 45, 180])
            node.send(list(result.degrees))
            node.call(node.enabler, SetBool.Request(data=True))
            node.wait(lambda: node.serial['phase'] in ('streaming', 'fault'), timeout=10)
            node.check_link()
            started = monotonic()
            while monotonic() - started < args.duration:
                node.check_link()
                rclpy.spin_once(node, timeout_sec=0.05)
            state = node.serial.copy()
            expected = '90,45,180'
            if state['last_sent'] != expected or state['last_ack'] != expected or state['last_done'] != expected:
                raise RuntimeError(f'No matching home ACK/DONE: {state}')
            if state['acknowledged'] < 5 or state['completed'] < 1:
                raise RuntimeError(f'Insufficient hardware replies: {state}')
            node.call(node.enabler, SetBool.Request(data=False))
            node.wait(lambda: not node.serial['enabled'])
            report.update(passed=True, protocol_compatible=True, streaming_seconds=monotonic()-started,
                          streaming_status=state, final_status=node.serial.copy())
            print(f"PASS: real ESP32 at {args.port}; {state['sent']} frames, "
                  f"{state['acknowledged']} ACKs, {state['completed']} DONE replies; serial disabled.")
    except Exception as error:
        report['error'] = str(error)
        print(f'Hardware check failed: {error}')
    finally:
        # Always disable before stopping launch, including interrupted tests.
        try:
            if node.enabler.service_is_ready():
                node.call(node.enabler, SetBool.Request(data=False))
        except Exception:
            pass
        if launch is not None and launch.poll() is None:
            launch.send_signal(signal.SIGINT)
            try:
                launch.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGTERM)
                launch.wait(timeout=5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        report_path.write_text(json.dumps(report, indent=2) + '\n')
        print(f'Report: {report_path}')
        print(f'Launch log: {launch_log}')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
