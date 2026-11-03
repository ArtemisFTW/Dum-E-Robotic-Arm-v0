# ROS 2 operating guide

The expanded [Architecture and Testing manual](ARCHITECTURE_AND_TESTING.md)
explains the implementation and provides the full staged bench-test procedure.

## Installed environment

This computer has Ubuntu 24.04 and ROS 2 Jazzy desktop, colcon, NumPy, pyserial,
PyYAML and pytest. The migration uses those system packages, with no pip changes
to system Python. VS Code is installed for this user at
`~/.local/opt/VSCode-linux-x64`, with a command at `~/.local/bin/code` and an
application-menu launcher. This installation needs no administrator password.
The official Python extension, Pylance, debugger and Python environment extension
are installed as well.
For future VS Code updates, replace the official Linux x64 archive installation
or install Microsoft's `.deb` with administrator access.

Open the project using:

```bash
~/.local/bin/code "$HOME/Desktop/Dum-E Robotic Arm v0"
```

The project's VS Code terminal profile loads ROS and this workspace. Its tasks
include Build ROS 2 and Run ROS 2 dry run. No global shell startup files are edited.
The helper scripts work even if `~/.local/bin` is not yet in the current shell PATH.

## Workspace and build

```text
ros2_ws/src/
  dum_e_interfaces/          Timestamped command message and read-only solver service
  dum_e_control/
    dum_e_control/core/     Original solver/model/config, packaged for ROS
    dum_e_control/          Two nodes, serial engine, client and URDF generator
    launch/arm.launch.py    Starts both control nodes; optional RViz
    config/                Parameters, sample poses, RViz configuration
    test/                  Core parity, reference, URDF and serial checks
```

Build after editing ROS source:

```bash
./build_ros2.sh
```

Dependencies for a fresh Ubuntu 24.04 computer, after installing ROS 2 Jazzy:

```bash
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions \
  python3-numpy python3-serial python3-yaml python3-pytest python3-tk
```

The [ROS Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
and [Microsoft's Linux setup guide](https://code.visualstudio.com/docs/setup/linux)
are the primary installation references.

## Starting and commanding

For the same graphical interface as the Windows project, run `./gui.sh`. It
starts the two control nodes plus the Arm Console client. Jog/FK/IK, live sliders,
staged Solve XYZ, pause/resume, Model/mapping, activity, command preview, the
port selector, Connect/Disconnect, and the mixed joint/XYZ Loop page are retained.
Loop playback still waits for matching DONE and then holds for two seconds.

The GUI initially previews the backend's current command. Editing while
disconnected uses read-only ROS calculations and does not select a live target.
**Connect & stream** sends the displayed valid target and enables the selected
connection through the serial node. **ROS 2 dry run (no hardware)** is a distinct
port option and uses synthetic replies. Select `/dev/ttyUSB0` or the stable
ESP32 device path for real control. Changing the selected port while connected
requires Disconnect. The 115200 baud rate remains fixed.

To preselect the connected board while leaving serial disabled:

```bash
./gui.sh dry_run:=false \
  port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

To attach only the GUI to an already running pair of nodes, source
`scripts/ros_env.sh` and run `ros2 run dum_e_control gui`. Do not start a second
pair of control nodes for the same device. The combined `gui.sh` launcher shuts
down its nodes when the GUI closes. Normal GUI closure requests Disconnect;
force-killing only a separately attached GUI does not stop the independent
control nodes. Use one operator at a time.

`./run_ros2.sh` defaults to dry run and disabled serial. It does not select a
startup target or open hardware. `./arm.sh home` selects the firmware's home
target; `./arm.sh enable` separately starts streaming. All clients must use the
same `ROS_DOMAIN_ID`. Localhost discovery is the default in the helper scripts.
To use other computers, explicitly configure ROS discovery and networking.

```bash
./arm.sh servo 70 30 40        # Final servo coordinates, degrees
./arm.sh dh 0.34906585 0.52359878 -0.87266463  # Mathematical joint coordinates, radians
./arm.sh xyz 0.1291673 0.0469705 0.2119948     # Position in base_link, metres
./arm.sh home                 # Select [90,45,180] servo degrees
./arm.sh enable               # Open explicit port or enable dry-run output
./arm.sh status
./arm.sh disable              # End serial stream
```

Do not run the original Tk controller or an Arduino serial monitor against the
same device while the bridge is enabled. Only the ROS serial bridge owns the port.
The bridge requests exclusive access on Linux. Faults are latched: check the
reported reason, correct it, and explicitly enable again. There is no automatic
reconnect or automatic recovery motion.

## ROS API

Names below use the default `/dum_e` namespace. ROS joint coordinates are DH
radians and Cartesian coordinates are metres. Servo degrees appear only at
explicit actuator input/output boundaries.

| Name | Type | Meaning |
|---|---|---|
| `/dum_e/target_position` | `geometry_msgs/PointStamped` | XYZ in metres; frame empty or `base_link` |
| `/dum_e/joint_target` | `sensor_msgs/JointState` | Exactly `joint_1`, `joint_2`, `joint_3`, any order, positions in DH radians |
| `/dum_e/servo_target` | `std_msgs/Float64MultiArray` | Three servo degrees; raw limits checked before rounding |
| `/dum_e/servo_commands` | `dum_e_interfaces/ServoCommand` | Timestamp plus three final integer servo degrees; 20 Hz after first valid target |
| `/dum_e/joint_targets` | `sensor_msgs/JointState` | Accepted, rounded DH target in radians; no physical measurements |
| `/dum_e/tool_target` | `geometry_msgs/PointStamped` | FK of accepted target in metres |
| `/dum_e/kinematics/status` | `std_msgs/String` | JSON: target, FK, accepted/rejected counters and last error |
| `/dum_e/serial_bridge/status` | `std_msgs/String` | JSON: enabled, phase, dry-run flag, port, pending command, counters, last frames, fault |
| `/dum_e/serial_bridge/tx` | `std_msgs/String` | Actual complete written frame, or dry-run frame |
| `/dum_e/serial_bridge/rx` | `std_msgs/String` | Firmware reply line; synthetic replies in dry run |
| `/dum_e/solve_target` | `dum_e_interfaces/srv/SolveTarget` | Read-only validation: `servo` degrees, `dh` radians, `xyz` metres; never selects a target |
| `/dum_e/home` | `std_srvs/srv/Trigger` | Select home target; serial enable remains separate |
| `/dum_e/serial_bridge/enable` | `std_srvs/srv/SetBool` | Explicit enable/disable; enabling requires a fresh valid target |
| `/dum_e/serial_bridge/configure` | `dum_e_interfaces/srv/ConfigureSerial` | Choose port/dry-run mode only while disabled; does not open a device |

Equivalent standard ROS commands, after `source scripts/ros_env.sh`:

```bash
ros2 topic pub --once /dum_e/target_position geometry_msgs/msg/PointStamped \
  '{header: {frame_id: base_link}, point: {x: 0.1291673, y: 0.0469705, z: 0.2119948}}'
ros2 service call /dum_e/serial_bridge/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /dum_e/serial_bridge/status
```

The kinematics node continues publishing the last accepted command when inputs
are idle or an invalid target is rejected, matching the original controller's
hold-target behavior. If that node stops, the bridge stops transmitting after
0.5 seconds without a fresh command. The existing ESP32 then freezes its current
PWM setpoints after its separate one-second link timeout. Missing matching ACKs
for two seconds, serial errors, partial writes, firmware errors, and controller
reset/hold reports also disable the bridge. Commands with stale/future timestamps
are ignored. Normal shutdown closes the port; no home command is sent on exit.

The command client waits for the serial bridge to receive the accepted target
before returning. This avoids enabling too early during initial DDS discovery.

Parameters are startup configuration. Edit `config/control.yaml` and rebuild,
then restart; changing a ROS parameter live does not reconfigure the serial engine.
The explicit configure service is the supported way for the GUI to change its
connection selection while disabled. The bridge status reports that active
selection. The solver service additionally returns model metadata, DH coordinates,
conditioning and rounding error for the GUI; its optional reference field keeps
pose-list branch selection consistent without selecting a live target.

## Calibration and geometry

The port preserves standard DH geometry and the existing mapping:

| Joint | a (mm) | d (mm) | alpha (degrees) | Servo mapping |
|---|---:|---:|---:|---|
| 1 | 10.35 | 45.74 | 90 | `J1 = 90 - theta1` |
| 2 | 125.7 | 6.24 | -180 | `J2 = theta2` |
| 3 | 105.0 | 6.20 | 0 | `J3 = 90 + theta3` |

All servo commands are whole degrees in `[0,180]`. Quantization uses the original
half-up rule. IK checks both base-radius and elbow branches, joint limits,
singularities, independent FK and the effect of rounding. It selects the valid
branch nearest the current commanded pose. Inputs are accepted atomically.

The original `Code/arm_interface/robot_config.json` remains the Windows
controller's model. The ROS package has a matching copy at
`ros2_ws/src/dum_e_control/dum_e_control/core/robot_config.json`. To try a measured
calibration without editing either default, pass an absolute file path:

```bash
./run_ros2.sh robot_config:=/absolute/path/to/robot_config.json visualize:=true
```

Both the solver and generated URDF use that model. The serial bridge applies no
additional direction/offset transform. RViz is a schematic of the DH chain, not
the SolidWorks assembly or a collision model. The firmware home is fixed at
`[90,45,180]`; changes to startup home require coordinated firmware/source edits.

This is three-DOF position control. It does not plan collision-free paths,
enforce straight Cartesian motion, control arbitrary tool orientation, or use
encoder feedback. ESP32 `DONE` means its commanded PWM setpoints arrived; it
does not prove actual shaft arrival. Dry-run ACK/DONE is immediate and synthetic;
it does not simulate servo timing or dynamics. RViz follows the target directly.

## Pose lists and looping

Sample file: `ros2_ws/src/dum_e_control/config/poses.yaml`.

```yaml
poses:
  - mode: servo
    values: [70, 30, 40]
  - mode: xyz
    values: [0.1291673, 0.0469705, 0.2119948]
```

```bash
./arm.sh loop ros2_ws/src/dum_e_control/config/poses.yaml --cycles 1 --hold 2
./arm.sh loop ros2_ws/src/dum_e_control/config/poses.yaml --cycles 0
```

The client resolves and validates the entire list through the running kinematics
node before sending any pose. It requires the bridge to be enabled already.
For each pose it waits for a matching new `DONE`, then holds for two seconds by
default. It stops advancing on a bridge fault or a 20-second completion timeout.
Ctrl+C stops cycling and leaves the current target active. `./arm.sh disable`
ends transmission. List coordinates are resolved to commands before playback.
Use one operator/command client at a time; the API does not arbitrate competing
publishers or reserve control ownership.

## ESP32 and serial permissions

Use the existing sketch and both headers in `Code/esp32_arm_receiver`; the
[firmware README](Code/esp32_arm_receiver/README.md) contains pin assignments,
power wiring and flashing instructions. The STM32 project is an independent
single-servo bench test and is not the three-joint receiver.

List available Linux ports:

```bash
python3 -m serial.tools.list_ports -v
ls -l /dev/serial/by-id/
```

Prefer a stable `/dev/serial/by-id/...` path. If opening it reports permission
denied and the port's group is `dialout`, add your account with administrator
access and log out/in before retrying:

```bash
sudo usermod -aG dialout "$USER"
```

The connected CP2102 ESP32 has the stable path
`/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`.
`./run_hardware.sh` uses that path; set `DUM_E_PORT` to override it for another
board. The launcher leaves serial disabled until an explicit enable command.

Your account was added to `dialout` through Ubuntu authentication. **Log out and
back in once for persistent group access in all applications.** A temporary
per-user ACL also grants access to the currently connected device, so the
existing connection works immediately. That ACL may disappear after unplugging.
Do not run ROS or VS Code as root.

After the user connected the ESP32, a home-only hardware test passed: 61 frames
sent and 60 matching ACK/DONE replies received. The bridge was disabled afterward.
The firmware was not edited or flashed. Only the existing `[90,45,180]` target
was sent, and physical shaft positions were not measured.

The matching firmware uses 115200 8N1, ASCII `J1,J2,J3\n`, nominal 20 Hz targets,
and GPIO25/26/27 servo outputs. It performs coordinated motion at up to
50 degrees/second for the longest-travel joint. Keep the existing external servo
power and common-ground arrangement described in the firmware guide.

## Verification

From the project root:

```bash
source scripts/ros_env.sh
cd ros2_ws
colcon test --packages-select dum_e_control --event-handlers console_direct+
colcon test-result --verbose
cd ..
python3 scripts/verify_ros2.py
python3 scripts/verify_cli.py
python3 scripts/verify_gui.py
python3 scripts/verify_gui_launch.py
python3 -m unittest discover -s Code/arm_interface/tests -v
```

To explicitly repeat the real-device home-only check (opening serial can cause
startup home movement):

```bash
ROS_DOMAIN_ID=93 python3 scripts/verify_hardware.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

It records a JSON result and launch log in `test-results/`, then disables serial
and stops its nodes. Run it with other controllers for this device closed.

The original Tk tests need a working graphical session. DDS integration checks
need local networking; the desktop app's sandbox may require allowing that test
to run outside the sandbox. `verify_ros2.py` uses only a Linux pseudo-terminal,
never a physical device. It exercises normal commands, units, replies, read-only
solving, invalid inputs, stale commands, missing ACK, controller reset and node
death. Firmware host tests are documented in the firmware README.

See [VALIDATION.md](VALIDATION.md) for this checkout's executed results. These
software checks do not establish physical calibration or arm motion.
