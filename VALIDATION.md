# Validation record — 3 November 2026

Environment: Ubuntu 24.04.3 x86_64, ROS 2 Jazzy, Python 3.12.
Upstream checkout: `67cadab` from
`https://github.com/ArtemisFTW/Dum-E-Robotic-Arm-v0.git`.

- Both ROS packages build with colcon.
- The build and packaged colcon tests also pass from the final Desktop folder;
  colcon reports 19 tests with no errors, failures or skips.
- 19 new core/serial tests pass. They include source parity with the upstream
  solver, MATLAB reference matrices/Jacobians, 400 FK/IK round trips, quantized
  failure holds, generated URDF/DH agreement, framing, startup delay, stale
  command and ACK timeouts, reset/error handling, partial writes, bounded input,
  dry-run isolation, and real pyserial I/O through a Linux pseudo-terminal.
- All 57 original Python controller tests pass with desktop display access.
- All three original C++ ESP32 host suites pass: protocol, coordinated motion,
  and simulated firmware.
- Both actual ROS nodes pass `scripts/verify_ros2.py`, including command delivery
  through DDS and pyserial, firmware ACK/DONE return traffic, unit/mapping checks,
  input rejection, stale message rejection, missing-ACK shutdown, reset shutdown,
  and kinematics-node-death shutdown.
- VS Code user installation reports version `1.144.1` (x64).
- The installed launch and command client pass dry-run checks for the enable
  guard, servo targets, XYZ IK, DH radians, invalid input, pose-loop completion,
  disable and status. See `scripts/verify_cli.py`.
- Optional RViz and robot_state_publisher launch successfully with the calculated
  home target and shut down cleanly through ROS launch.
- VS Code opens the Desktop project; the official Python, Pylance, debugger and
  environment extensions are installed.

## Real ESP32 check after connection

The connected CP2102 USB-UART device was detected at `/dev/ttyUSB0`, with stable
path `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`.
After Ubuntu authentication, the current device received a user read/write ACL,
and the account was added to `dialout` for access after a new login.

`scripts/verify_hardware.py` passed using both actual ROS nodes and the real
serial port: 61 home-target frames sent, 60 matching ACKs, and 60 matching DONE
replies in approximately three seconds of streaming. No firmware changes or
flashing were needed. Only the existing `[90,45,180]` servo target was sent.
Serial was explicitly disabled and the test nodes stopped afterward.

The first attempt revealed an initial DDS discovery race before the serial port
opened. The client now waits for the bridge to receive its pending target before
returning. After that fix, all 19 packaged tests, CLI checks and emulated ROS
integration checks passed again, as did the real ESP32 check.

Evidence: [JSON result](test-results/esp32-home-20261103T021032Z.json) and
[launch log](test-results/esp32-home-20261103T021032Z.log).

Physical shaft positions were not measured. Loaded motion, physical calibration,
servo power and mechanical clearance still require bench observation. ACK/DONE
confirms firmware command/PWM completion, not encoder feedback.

## Original GUI through ROS 2

The original Arm Console layout and its Control/Loop pages are adapted into the
ROS package. The original Windows sources and ESP32 firmware remain unchanged.
`gui.sh` starts both control nodes plus the GUI client. Its application-menu
entry is **Dum-E Arm Console (ROS 2)**.

- `scripts/verify_gui.py` passed against real ROS service nodes with local GUI
  FK/IK functions disabled, demonstrating that calculations run in the backend.
- Verified joint sliders, FK, staged XYZ solving, invalid-target hold, read-only
  offline previews and pose-list editing, clear dry-run status, Connect/Disconnect,
  pause/resume, model/mapping dialog, and mixed joint/XYZ loop playback with
  matching DONE and the existing two-second hold.
- The same GUI connected to the real ESP32 using only `[90,45,180]`, received
  20 matching ACKs and 20 matching DONE replies, then disconnected successfully.
  Evidence: [GUI hardware log](test-results/gui-hardware.log).
- `scripts/verify_gui_launch.py` passed: no automatic target or serial open at
  startup, and clean shutdown of all three processes when the GUI exits.
- After GUI integration, all 19 packaged tests and the existing CLI and ROS/PTY
  integration scripts passed again.

The GUI never opens serial directly. The serial configure service rejects
changes while streaming; changing a GUI connection requires Disconnect first.
