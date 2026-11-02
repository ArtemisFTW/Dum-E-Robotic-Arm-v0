# Robotic Arm — Master Documentation

Updated: **3 November 2026**. Project root: `E:\Work\Robotic arm`.

This is the central guide for the current three-revolute-joint (RRR) arm: its mathematical model, servo calibration, desktop software, ESP32 firmware, wiring, operation, and verification. Values below were checked against the current source files. Software tests establish command and calculation behavior; physical accuracy, load capacity, collision clearance, and actual shaft motion require bench measurement.

## 1. Key facts at a glance

| Item | Current implementation |
|---|---|
| Mechanism | 3 DOF: base yaw, shoulder, elbow; position IK |
| Units | Lengths in mm; entered joint angles in degrees |
| Convention | Standard DH: `Rz(theta) Tz(d) Tx(a) Rx(alpha)` |
| DH lengths `a` | `[10.35, 125.7, 105]` mm |
| DH offsets `d` | `[45.74, 6.24, 6.2]` mm |
| DH twists `alpha` | `[90, -180, 0]` degrees |
| Servo mapping | `J1 = 90 - theta1`; `J2 = theta2`; `J3 = 90 + theta3` |
| Servo command limits | Integer `0–180` degrees on every joint |
| Startup/home | Servo `[90,45,180]`; DH `[0,45,90]` degrees |
| App XYZ | `[DH_X,DH_Y,DH_Z]`; Z upward |
| Main controller | Classic ESP32/WROOM, Arduino-ESP32 3.x |
| Servo signals | J1 GPIO25; J2 GPIO26; J3 GPIO27 |
| Serial | UART0, 115200 baud, 8N1; ASCII `J1,J2,J3` plus newline |
| PC command stream | Latest accepted target every 50 ms, nominally 20 Hz |
| Servo PWM | 50 Hz, 16-bit LEDC, nominal 500–2500 microseconds |
| Normal motion | Shared duration across all joints; nominal maximum 50 degrees/s |
| Boot/reset | Directly commands home, waits 2 seconds, then accepts targets |
| Lost link | After more than 1 second without a valid frame, freezes current PWM |
| Loop | Waits for matching `DONE`, holds 2 seconds, advances and repeats |
| Feedback | ACK and PWM completion only; no measured shaft-position feedback |

**Important operating behavior:** opening the COM port may reset the board and move the arm toward home. Pause input and Stop loop allow the current target move to finish. Disconnect eventually freezes the current PWM setpoint after the link timeout. None of these is an immediate physical emergency stop.

## 2. Project files and sources of truth

```text
E:\Work\Robotic arm\
├── MASTER_DOCUMENTATION.md       This guide
├── IA2_RoboticArm_3DOF.pdf        Existing project report
├── CAD\                         SolidWorks parts/assemblies and neutral/print exports
└── Code\
    ├── arm_interface\           Python/Tkinter desktop controller
    ├── esp32_arm_receiver\      Three-servo UART receiver and coordinated motion
    └── servo_test_f446re\       Separate STM32 single-servo bench-test project
```

### Desktop controller

| File | Responsibility |
|---|---|
| [app.py](Code/arm_interface/app.py) | Tkinter widgets, Control/Loop pages, staged XYZ solving, serial controls, activity/error display, playback integration |
| [robot_config.json](Code/arm_interface/robot_config.json) | Active geometry, servo offsets/directions, XYZ signs, near-singularity threshold |
| [kinematics.py](Code/arm_interface/kinematics.py) | Model validation, DH matrices, FK, position Jacobian, analytical IK, servo/DH conversions; home and command limits |
| [model.py](Code/arm_interface/model.py) | Accepted command state, input validation, whole-degree rounding, post-rounding checks, last-valid-command behavior |
| [uart_transport.py](Code/arm_interface/uart_transport.py) | One serial worker, latest-target slot, repeated writes, ACK/DONE status and error handling |
| [pose_loop.py](Code/arm_interface/pose_loop.py) | Isolated pose validation and completion-driven repeating sequence |
| [requirements.txt](Code/arm_interface/requirements.txt) | NumPy and pyserial requirements; Tkinter comes with Python |
| [launch.bat](Code/arm_interface/launch.bat) | Desktop launch shortcut after dependencies are installed |
| [tests/](Code/arm_interface/tests/) | Kinematics, UI state/widgets, pose loop, transport, MATLAB-exported fixture |
| [README.md](Code/arm_interface/README.md) | Detailed desktop guide |

### ESP32 controller

| File | Responsibility |
|---|---|
| [esp32_arm_receiver.ino](Code/esp32_arm_receiver/esp32_arm_receiver.ino) | UART0, LEDC setup/output, boot home/dwell, target acceptance, link timeout, OK/DONE/error reports |
| [servo_protocol.h](Code/esp32_arm_receiver/servo_protocol.h) | Atomic three-angle parsing, range checks, partial-frame timeout, integer angle-to-pulse conversion, home |
| [servo_motion.h](Code/esp32_arm_receiver/servo_motion.h) | Shared-progress PWM interpolation, retargeting, heartbeat continuity, timeout hold/resume |
| [tests/](Code/esp32_arm_receiver/tests/) | Host C++ parser, simulated firmware, and coordinated-motion harnesses |
| [README.md](Code/esp32_arm_receiver/README.md) | Wiring, flashing, protocol and compilation instructions |

### MATLAB reference package

The current MATLAB package lives outside this root at:

`C:\Users\hrish\Desktop\new files\MATLAB`

| File | Responsibility |
|---|---|
| `RRR_model.m` | Shared MATLAB DH geometry |
| `DH.mlx` | Standard-DH transform/live function |
| `RRR_FK.m` | Full transform, four frame origins, geometric position Jacobian |
| `RRR_IK_solve.m` | Analytical branches, limits, FK validation and diagnostic report |
| `RRR_servo_map.m` | Shared mounting calibration |
| `RRR_cartesian_frame.m` | DH/app XYZ conversion |
| `RRR_IK.m` | Target bench, branch comparison, plots and CSV/MAT reporting |
| `workspace_generation.m` | Joint-limited sampling, plots and workspace exports |
| `RRR_joint_slider.m` | Interactive 3D mathematical viewer; no hardware control |
| `test_RRR_IK.m`, `test_RRR_joint_slider.m` | MATLAB regression/viewer checks |
| `RRR_IK_README.md` | Detailed MATLAB package guide |

Python loads `robot_config.json`; MATLAB loads `RRR_model.m`. They are separate files and must be kept consistent. The app does not need MATLAB installed. `tests/matlab_reference.json` is an exported reference fixture, not a live MATLAB connection.

### CAD and existing report

`CAD/3link_assembly.SLDASM` is the top-level three-link assembly file present in this root. Other assemblies cover the base, shoulder, elbow, brackets and servo/horn fits. `.SLDPRT`/`.SLDASM` are native SolidWorks files; `.STEP` files support neutral exchange; `.3mf`/`.STL` files support print/mesh workflows. These files were inventoried for this guide; their mates, dimensions and print suitability were not revalidated.

Treat the active code/configuration as the executable reference when reading the existing PDF or older notes. Historical `h/L1/L2` models and `RRR_IK_original_20260922.m` are not the current model.

## 3. Coordinate definitions and DH parameters

Use three distinct quantities:

- `theta = [theta1,theta2,theta3]`: signed mathematical DH joint coordinates.
- `J = [J1,J2,J3]`: final integer servo commands; these are what the sliders and UART use.
- `[X,Y,Z]`: Cartesian position in mm of the origin of DH frame 3 relative to DH frame 0.

The modeled end point is the frame-3 origin. No extra gripper/tool translation is currently appended. A real tool tip needs an explicitly measured tool transform. DH frame origins and the simple MATLAB link drawing are not a full collision model of the brackets or mechanism.

### Current standard-DH table

| Transform | Variable | `a` (mm) | `d` (mm) | `alpha` (degrees) |
|---|---|---:|---:|---:|
| 0 → 1 | theta1 | 10.35 | 45.74 | +90 |
| 1 → 2 | theta2 | 125.7 | 6.24 | -180 |
| 2 → 3 | theta3 | 105.0 | 6.20 | 0 |

For each row, `a` is the common-normal length, `d` the offset along the preceding Z axis, `theta` the rotation about that Z axis, and `alpha` the twist about the new X axis. This is standard DH, not modified DH.

```text
A_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)

      [ cθ  -sθ*cα   sθ*sα   a*cθ ]
A_i = [ sθ   cθ*cα  -cθ*sα   a*sθ ]
      [  0      sα      cα      d  ]
      [  0       0       0      1  ]

T03 = A1 * A2 * A3
```

All trigonometric expressions in this guide use degrees. Python converts to radians before calling standard math functions; MATLAB uses `sind`, `cosd`, and `atan2d`.

The current app/viewer signs are `[1,1,1]`, so displayed XYZ equals raw DH XYZ. Previous display conventions may reverse Y. Do not apply that old reversal or swap X and Z to current targets. Verify the physical base origin and frame-3 reference point before comparing absolute ruler measurements.

## 4. Final forward-kinematics equations

Define:

```text
delta = d2 - d3 = 0.04 mm
beta  = theta2 - theta3
r     = a1 + a2*cos(theta2) + a3*cos(beta)
```

The final position is:

```text
x = r*cos(theta1) + delta*sin(theta1)
y = r*sin(theta1) - delta*cos(theta1)
z = d1 + a2*sin(theta2) + a3*sin(beta)
```

With the current dimensions:

```text
r = 10.35 + 125.7*cos(theta2) + 105*cos(theta2-theta3)
x = r*cos(theta1) + 0.04*sin(theta1)
y = r*sin(theta1) - 0.04*cos(theta1)
z = 45.74 + 125.7*sin(theta2) + 105*sin(theta2-theta3)
```

The `-180` degree second twist reverses the third joint axis, producing **theta2 - theta3**. The signed quantity `r` can be negative. The true cylindrical radius is `rho = sqrt(r*r + delta*delta)`.

For completeness, define `c1=cos(theta1)`, `s1=sin(theta1)`, `cb=cos(beta)`, `sb=sin(beta)`. The complete end-frame rotation and homogeneous transform are:

```text
      [ c1*cb   c1*sb  -s1 ]
R03 = [ s1*cb   s1*sb   c1 ]
      [    sb     -cb    0 ]

      [ c1*cb   c1*sb  -s1   x ]
T03 = [ s1*cb   s1*sb   c1   y ]
      [    sb     -cb    0   z ]
      [     0       0    0   1 ]
```

The software independently multiplies DH transforms for FK rather than relying solely on the simplified position equations.

### Reference poses

These are calculated targets, not measured hardware positions. Straight poses may be used in Jog/FK but are singular and rejected by position IK.

| Servo `J1,J2,J3` (degrees) | DH `theta1,theta2,theta3` (degrees) | Meaning | X (mm) | Y (mm) | Z (mm) |
|---|---|---|---:|---:|---:|
| 90,0,90 | 0,0,0 | Straight along +X | 241.05 | -0.04 | 45.74 |
| 90,45,180 | 0,45,90 | Startup/home | 173.48 | -0.04 | 60.38 |
| 90,90,90 | 0,90,0 | Shoulder upright, links straight | 10.35 | -0.04 | 276.44 |
| 90,180,90 | 0,180,0 | Shoulder points left, links straight | -220.35 | -0.04 | 45.74 |
| 90,0,0 | 0,0,-90 | Distal link upright | 136.05 | -0.04 | 150.74 |
| 90,0,180 | 0,0,90 | Distal link downward | 136.05 | -0.04 | -59.26 |
| 70,30,40 | 20,30,-50 | Nonsingular worked pose | 129.1673 | 46.9705 | 211.9948 |

At the mathematical zero pose, the four FK frame origins are `[0,0,0]`, `[10.35,0,45.74]`, `[136.05,-6.24,45.74]`, and `[241.05,-0.04,45.74]` mm. The small final Y offset is `d3-d2`; the one-decimal app readout can display it as zero.

## 5. Final inverse-kinematics equations and workspace checks

The IK target is **position only**. Three joints cannot independently assign arbitrary XYZ and arbitrary end-effector orientation. Orientation follows from the selected joint solution through `R03`.

For target `[x,y,z]`, define:

```text
rho = sqrt(x*x + y*y)
h   = z - d1
```

1. Reject `rho < abs(delta)`, allowing numerical tolerance. This is the excluded central cylinder: radius `0.04 mm` for this model.
2. Try both signed radial branches:

   ```text
   r = +sqrt(rho*rho - delta*delta)
   r = -sqrt(rho*rho - delta*delta)
   S = r - a1
   L = sqrt(S*S + h*h)
   ```

3. A radial branch is geometrically reachable only if:

   ```text
   abs(a2-a3) <= L <= a2+a3
   20.7 mm   <= L <= 230.7 mm
   ```

4. For each reachable radial branch, calculate:

   ```text
   theta1 = atan2(y,x) + atan2(delta,r)
   D      = (S*S + h*h - a2*a2 - a3*a3)/(2*a2*a3)
   e      = atan2(+sqrt(1-D*D), D)   # also use the negative square-root branch
   theta2 = atan2(h,S) - atan2(a3*sin(e), a2+a3*cos(e))
   theta3 = -e
   ```

Only tiny numerical roundoff is clamped at square-root/cosine boundaries after the reach test. An unreachable target is not converted into a reachable one by clipping.

There are up to four discrete candidates: two base-radius choices times two planar elbow choices. Coincident boundary branches need not produce four distinct solutions. Full-turn equivalents are considered within joint limits. The special continuous-family cases are handled in the solver; with the current unequal link lengths and nonzero lateral offset, those equal-length/free-axis degeneracies do not apply generally.

### Acceptance pipeline in the desktop app

```text
Finite XYZ and UI bounds
  → app XYZ to DH XYZ
  → signed-radius and two-link geometry
  → all candidate branches and equivalent angles
  → DH-to-servo conversion and physical command limits
  → independent DH FK position check
  → Jacobian singularity/near-singularity rejection
  → choose valid branch nearest the current servo commands
  → round to integer servo degrees
  → recheck limits, conditioning and achieved FK displacement
  → publish the complete accepted triple
```

The nearest branch minimizes `sum((candidate_servo-current_servo)^2)`. Unrounded FK tolerance is `1e-8*(a2+a3)` mm. Input commands use nearest-degree rounding; positive half degrees round up. Invalid raw joint values are rejected before rounding, so `180.1` cannot become an accepted `180`.

Post-rounding FK displacement is bounded in `model.py` by:

```text
rounding_bound = radians(0.5) *
                 (2*(a2+a3) + abs(a1) + abs(d2-d3) + a2) + 1e-6 mm
```

This is a conservative software bound, about 5.03 mm for this model, not a promised physical accuracy. The app displays the achieved rounded-command FK and target error. Servo backlash, mounting, load deflection and calibration add separate errors.

The ±500 mm XYZ slider span is an input range, not the workspace. The annulus above is a branch reach test, not a spherical workspace centered on the base. Servo limits, singularity rejection, and physical obstructions further constrain usable poses.

## 6. Jacobian and singularity policy

The geometric position Jacobian uses each preceding frame's joint Z axis:

```text
Jv_i = z_(i-1) cross (p_end - p_(i-1))
v_xyz = Jv * theta_dot
```

Although input angles are degrees, `theta_dot` in this relation is in **radians/s**, giving Cartesian velocity in mm/s. Multiply rates in degrees/s by `pi/180` first.

For `u = a2*sin(theta2)+a3*sin(beta)` and `v = a2*cos(theta2)+a3*cos(beta)`:

```text
     [ -r*s1 + delta*c1   -u*c1   a3*sin(beta)*c1 ]
Jv = [  r*c1 + delta*s1   -u*s1   a3*sin(beta)*s1 ]
     [                0       v     -a3*cos(beta) ]
```

Conditioning is evaluated using the smallest singular value of `Jv/(a2+a3)`:

- `sigma_min <= 1e-10`: singular.
- `1e-10 < sigma_min <= 0.001`: near singular with the current configuration.
- Above `0.001`: passes this conditioning check.

Collinear planar links and zero signed radius are singular configurations. Manual Jog/FK accepts joint-limit-valid straight poses; app IK rejects singular/near-singular candidates and rechecks after rounding. MATLAB's reporting tool can retain and rank singular/near-singular rows for analysis; a MATLAB recommendation is therefore not automatically an app-accepted hardware command.

## 7. Servo calibration and physical angle interpretation

General mapping:

```text
servo_i = offset_i + direction_i * theta_i
theta_i = (servo_i - offset_i)/direction_i

offsets    = [90, 0, 90] degrees
directions = [-1,+1,+1]
```

| Joint | Servo command | DH angle | Interpretation |
|---|---|---|---|
| J1 | 0 / 90 / 180 | +90 / 0 / -90 | Base direction reversed relative to DH yaw |
| J2 | 0 / 90 / 180 | 0 / 90 / 180 | Shoulder horizontal / upright / pointing left |
| J3 | 0 / 90 / 180 | -90 / 0 / +90 | With shoulder horizontal: distal link up / straight / down |

Thus effective DH limits are `[-90,+90]`, `[0,180]`, `[-90,+90]` degrees. The mounting reference servo `[90,0,90]` corresponds to mathematical `[0,0,0]` and links along +X. Do not restore the older all-90-offset mapping.

The conversion occurs before FK and after IK. **UART already contains final servo angles; the ESP32 must not reverse J1 or add offsets again.** No modulo-180 conversion or saturation is used to hide a limit violation.

Calibrate horn alignment and verify directions using small unloaded moves. DH dimensions describe geometry; they do not measure shaft zero, horn alignment, or servo linearity. If changing home, update both `HOME_JOINTS` in Python and `arm::HOME` in `servo_protocol.h`, then verify the pose and clearance.

## 8. How the code works end to end

```text
User Jog/FK command or explicitly solved XYZ target
    ↓
InterfaceState validates and stores one integer servo triple
    ↓
SerialStreamer replaces its latest-target slot
    ↓ every 50 ms after the startup allowance
ASCII UART0 frame: J1,J2,J3\n
    ↓
ESP32 FrameParser validates the whole frame atomically
    ↓
ServoMotion retargets from the current PWM setpoints
    ↓ every eligible 20 ms update
LEDC updates GPIO25/26/27 pulse widths
    ↓
OK = target accepted; DONE = target PWM setpoints reached
    ↓
App status updates; Loop can start its two-second dwell
```

### Desktop state and transport

`InterfaceState` is the accepted command authority. Editing an IK coordinate only stages a target; **Solve XYZ** evaluates all three coordinates together. Changing modes or editing the pose list does not implicitly send a new solved pose. A failed IK request stays editable while the accepted triple and UART frame remain unchanged.

`SerialStreamer` runs separately from the GUI. It maintains one latest-command slot instead of queuing slider history, so old targets are not replayed. It waits 2 seconds after opening the port, then repeatedly writes the latest complete frame. Serial open/read/write failures close the connection and report the error; incomplete writes also close the link. Reconnection is explicit.

The achieved XYZ display is FK of the accepted **target** triple. It is neither the measured tip position nor the intermediate setpoint during a firmware move. Activity and pose lists are held in memory for the app session.

### ESP32 startup and execution

At boot, the firmware initializes UART0 and PWM, directly applies `[90,45,180]`, and holds startup home for 2000 ms. Incoming move frames during that interval are drained and discarded. It then sends `READY` and a home `DONE`. Actual shaft positions at reset are unknown, so that first home command does not use the subsequent shared-duration ramp.

The main loop processes bounded serial work, expires stale partial frames, handles timeout state, and advances motion without blocking delay-based sweeps. Invalid frames never partially update a joint triple. A PWM initialization/write fault latches an error requiring restart; do not assume all channels updated successfully after that fault.

## 9. Coordinated motion and PWM equations

Servo targets remain integer degrees, but intermediate PWM values use integer microseconds. Endpoint conversion is:

```text
pulse_us(J) = 500 + floor((J*2000 + 90)/180)

J=0 → 500 us; J=90 → 1500 us; J=180 → 2500 us
```

At 50 Hz the PWM period is 20,000 microseconds. LEDC duty is calculated by integer rounding of `pulse_us * 65536 / 20000`.

For current start pulse `p0_i` and target pulse `p1_i`, firmware uses:

```text
longest_us = max(abs(p1_i-p0_i))
duration_ms = ceil(longest_us * 180 * ms_per_degree / 2000)
ms_per_degree = SERVO_STEP_INTERVAL_MS = 20

progress = elapsed_ms / duration_ms
p_i = p0_i + trunc_toward_zero((p1_i-p0_i)*elapsed_ms/duration_ms)
```

The configured ideal longest-joint rate is `1000/20 = 50 degrees/s`; integer pulse rounding and update timing give small quantization effects. All moving channels use the same progress and first reach final PWM on the same final update. A zero-distance target requires no interpolation.

Example: nominal travels `[90,45,180]` degrees use about 3.6 seconds, with rates about `[25,12.5,50]` degrees/s. An isolated 90-degree longest move takes about 1.8 seconds. These describe commanded trajectories, not measured mechanical speed or simultaneous shaft arrival.

- Updates occur no more often than every 20 ms (`ServoMotion::UPDATE_MS`).
- Repeated identical targets are heartbeats and do not restart a move.
- A different target starts a new shared move from the current intermediate PWM.
- Delayed firmware iterations do not catch up with large jumps; elapsed motion advances one update at a time, so wall-clock completion can be later.
- Lost-link hold freezes intermediate pulse widths; a later valid frame can resume/retarget from them.
- Increasing `SERVO_STEP_INTERVAL_MS` slows the nominal rate; e.g. 40 gives 25 degrees/s. Change the sketch constant and upload again. The 20 ms PWM/motion update period is a separate setting.

This is linear joint-space setpoint interpolation. It does not implement acceleration shaping, Cartesian straight-line motion, torque control, obstacle avoidance, or mechanical synchronization feedback.

## 10. Hardware wiring and firmware installation

Use the matching classic ESP32/WROOM firmware, rather than assuming these pins apply to C3/S3 boards.

| Connection | Destination |
|---|---|
| J1 yellow signal | ESP32 GPIO25 |
| J2 yellow signal | ESP32 GPIO26 |
| J3 yellow signal | ESP32 GPIO27 |
| Servo brown wires | External servo supply negative/GND |
| Servo red wires | External regulated servo supply positive |
| ESP32 GND | Same external supply negative/GND |
| PC USB data cable | ESP32 development-board USB connector |

Power servos from an external supply sized for their voltage and simultaneous load, with suitable wiring. Do not power three servos from ESP32/USB power pins or connect the servo positive rail to a GPIO. The existing [receiver wiring guide](Code/esp32_arm_receiver/README.md) contains the OT5320M datasheet link and supply-sizing notes. GPIO signals are 3.3 V; recognition and actual pulse timing still need physical checks.

The development board's USB-UART bridge normally provides UART0 already. If using a separate 3.3 V TTL adapter, connect adapter TX to GPIO3/RX0, adapter RX to GPIO1/TX0, and common ground. Avoid two adapters driving the same UART input.

### Upload with Arduino IDE

1. Keep servo power off while changing wiring or flashing; secure the base and clear the home/movement envelope before powering the mechanism.
2. Open `Code/esp32_arm_receiver/esp32_arm_receiver.ino`. Keep both header files beside the sketch.
3. Install/select **esp32 by Espressif Systems 3.x** and **ESP32 Dev Module** for a classic WROOM board.
4. Select the board COM port and upload. If required, hold BOOT during connection, then release.
5. Close Serial Monitor before the desktop app opens that COM port.
6. Expect reset to command home directly. The 2-second dwell is a startup allowance, not physical arrival confirmation.

## 11. How to use the arm

### Install and launch the desktop controller

Python 3.10+ with Tkinter is required. Run in PowerShell:

```powershell
Set-Location 'E:\Work\Robotic arm\Code\arm_interface'
python -m pip install -r requirements.txt
python app.py
```

After installation, `launch.bat` can start the app. It starts at `[90,45,180]` and remains disconnected until explicitly connected.

### Connect and perform a first move

1. Confirm the correct firmware, common ground, external power, secure mounting, and clearance for startup home.
2. Start the app. Click **Refresh ports**, choose the ESP32 port, and use **115200** baud.
3. Click **Connect & stream**. Opening the port may reset/home the board. Wait for **STREAMING** and incoming ACKs; the app's startup allowance is time-based, not a measured-position handshake.
4. Begin with small Jog changes around a known clear pose. Check each joint's direction and mechanical clearance before larger travel.
5. Inspect **Last UART write**, sent count and ACK count. A successful local write alone does not establish receiver acceptance or physical movement.
6. A new explicit connection sends the currently displayed accepted command. Only a fresh app instance initializes the UI to home.

### Jog and FK

- **Jog:** joint sliders/boxes directly set servo-domain angles. Enter or leaving a numeric box applies the value; valid changes stream when connected.
- **FK:** the same joint controls remain active; Cartesian values are read-only calculated positions.
- Both modes use whole-degree commands in `0–180`. Read the signed DH angles separately.
- Shortcuts: **Ctrl+1** Jog, **Ctrl+2** FK, **Ctrl+3** IK. **Escape** restores a numeric editor.

### IK / Cartesian positioning

1. Choose **IK**. Joint controls lock and XYZ initially matches the accepted command's FK.
2. Enter all three target coordinates in mm. Edits stage the target and do not move the arm.
3. Click **Solve XYZ**. Only a fully validated rounded servo triple replaces the current command.
4. Inspect achieved XYZ and target error. If rejected, read the message and correct the target; the old accepted command continues streaming.
5. **Use current pose** restores the target to the accepted command's FK.

Worked example: target `[129.1673,46.9705,211.9948]` mm has a solution near DH `[20,30,-50]` degrees, corresponding to servo `[70,30,40]`. The nearest valid branch depends on the current command. Check the calculated pose and clearance before applying it on hardware.

### Repeating a pose sequence

1. Open **Loop** beside **Control**. Select **Joints** or **XYZ**, enter three values and click **Add pose**. Both types can be mixed.
2. Adding/editing a pose validates on isolated state without moving the arm. XYZ entries use the same IK, limits, singularity and integer checks.
3. Use **Update selected**, remove or reorder controls as needed. **Use current command** fills the editor without sending a move. There is no fixed pose-count cap or persistence between app sessions.
4. Connect UART and wait for **STREAMING** with recent ACKs. Click **Send loop**; the app revalidates the list and locks manual/list editing during playback.
5. Each target waits for a fresh matching `DONE`, then holds for two full seconds before advancing. The list repeats until stopped.
6. Missing matching `DONE` for 20 seconds, stale ACKs over 2 seconds, firmware errors or a connection failure stop sequence advancement. Firmware without `DONE` support must be updated.
7. **Stop loop** stops cycling but continues streaming the current target, so its move may finish. A stopped loop does not clear the pose list.

A nominal 180-degree longest travel can take about 3.6 seconds plus the 2-second dwell; communication/update delays can extend it. Dwell starts after software PWM completion, not a measured settled-tip signal.

### Pause, disconnect and shutdown

| Action | Actual behavior |
|---|---|
| Pause input | Locks input and stops loop cycling; continues repeating the current target; an in-progress move can finish |
| Stop loop | Ends sequence advancement; keeps streaming the current target; an in-progress move can finish |
| Disconnect / close app | Stops host streaming; after >1 second without a valid frame, firmware freezes current PWM, potentially mid-move |
| Board reset | Commands startup home directly again |
| Invalid IK | Preserves the previous accepted target and frame; an existing move toward that target can continue |
| Invalid UART frame | Does not replace the accepted target or refresh its valid-frame heartbeat; ongoing motion continues until completion or link timeout |

PWM remains active during software hold. Plan shutdown with the mechanism supported and clear before removing actuator power; removing power also removes powered holding torque. Use a suitable physical power-stop arrangement for an immediate stop requirement.

## 12. Serial protocol and diagnostics

**115200 baud, 8 data bits, no parity, one stop bit, no hardware flow control.**

```text
PC → ESP32:  90,45,180\n
ESP32 → PC: OK,90,45,180\n
ESP32 → PC: DONE,90,45,180\n
```

Here `\n` denotes one actual line-feed byte, not a literal backslash followed by `n`. The app generates exactly three integer fields. Firmware additionally accepts surrounding spaces/tabs and CRLF. Decimals, signs, missing/extra fields and values outside `0–180` are rejected. Partial frames expire after more than 250 ms and are discarded through the following newline; the parser has a 48-byte buffer.

| Firmware output | Meaning |
|---|---|
| `HOLD,STARTUP_HOME,90,45,180` | Boot home commanded; startup dwell active |
| `READY,ARM_UART0,115200` | Startup dwell finished; move frames can be accepted |
| `OK,J1,J2,J3` | Valid target accepted, possibly still moving |
| `DONE,J1,J2,J3` | Target PWM trajectory has completed, or accepted target is already settled |
| `HOLD,LINK_TIMEOUT` | More than 1000 ms since a valid frame; intermediate PWM frozen |
| `ERR,INVALID_FRAME_OR_JOINT_LIMIT` | Frame rejected; accepted command not replaced |
| `ERR,PWM_FAILURE_RESTART_REQUIRED` | Latched PWM fault; inspect cause and restart |

An identical heartbeat for a settled target can generate another `DONE`. The loop player requires a completion newer than its pose-start baseline and matching that pose's target. There are no encoder readings in these messages.

## 13. MATLAB analysis and viewer use

Run from MATLAB:

```matlab
cd('C:/Users/hrish/Desktop/new files/MATLAB')
robot = RRR_model();
[T, points, J] = RRR_FK([20 30 -50], robot);
servo = RRR_servo_map([20 30 -50], 'dhToServo'); % [70 30 40]
RRR_IK                  % Edit targets/options for branch and diagnostic reports
workspace_generation    % Joint-limited workspace plots and exports
RRR_joint_slider        % Interactive viewer only; no UART hardware control
results = test_RRR_IK;
test_RRR_joint_slider
```

Keep `DH.mlx` on the MATLAB path. The DH live function and symbolic regression checks use Symbolic Math Toolbox; see the MATLAB README for dependencies and generated outputs.

The viewer's physical controls use `q = servo - 90`, so the corresponding mathematical DH angles are `[-q1,q2+90,q3]`. Its home controls are `[0,-45,90]`, corresponding to servo `[90,45,180]`. Do not feed viewer `q` values directly into `RRR_FK` as DH angles.

Workspace exports and branch tables are mathematical sampling/diagnostic results. They do not prove physical reach without collisions or supported payload. MATLAB may display blocked or singular branches intentionally for analysis; apply the desktop controller's stricter acceptance policy before commanding hardware.

## 14. STM32 single-servo test project

`Code/servo_test_f446re` is a separate NUCLEO-F446RE bench test. It is not the three-joint ESP32 UART receiver and does not accept the desktop `J1,J2,J3` protocol.

- Servo PWM: **PA6 / TIM3_CH1**, Nucleo **D12**; 50 Hz, nominal 500–2500 microseconds.
- Console: **USART2**, PA2 TX / PA3 RX, 115200 baud through the ST-LINK virtual COM routing on the expected board.
- Startup is a single-servo 90-degree command. Send an integer `0–180` followed by newline to move/hold; `sweep` repeats full travel, and an angle cancels sweep.
- Motion uses 5 microsecond steps every 20 ms, about 8 seconds for a full-range traversal; this differs from the ESP32 shared 50 degrees/s policy.
- `Core/Src/servo_console.c` implements the bench application. `main.c` initializes it and polls it. USART2 reception depends on the enabled NVIC interrupt and `USART2_IRQHandler()` calling the HAL handler.
- Use the detailed [OT5320M_SERVO_TEST_GUIDE.md](Code/servo_test_f446re/OT5320M_SERVO_TEST_GUIDE.md) for clocks, timer settings, wiring, build and flashing. Do not substitute its board pinout or protocol for the ESP32 setup.

## 15. Troubleshooting

| Symptom | First checks / interpretation |
|---|---|
| COM port missing or busy | USB data cable, correct board port, Refresh ports, close Serial Monitor and other port users |
| Sent count rises but no ACK | Matching baud, correct firmware/port, board reset/startup state; local writes do not prove receiver delivery |
| ACKs but no physical motion | External servo power, shared ground, signal pin order, linkage obstruction; ACK is target acceptance only |
| Unexpected move on connect | Port opening may reset the board and invoke direct startup home |
| J1 moves opposite the model | Check horn/reference orientation and app mapping; do not add a second firmware inversion |
| Horizontal pose has unexpected height | Verify DH frame-0 origin, `d1=45.74`, frame-3 reference point and tool offset physically |
| IK target is rejected | Reach annulus, axial cylinder, servo-domain limits, singularity threshold and integer-rounding checks |
| Straight pose works in Jog but not IK | Expected singularity policy; use a nonsingular Cartesian target |
| Pause/Stop loop still lets it move | They preserve streaming of the current target; they are input/playback stops |
| Disconnect freezes before the target | Expected mid-move link-timeout hold; current PWM is frozen after the timeout |
| Sequence does not advance | Recent ACK, matching fresh DONE, current firmware with DONE support, 2-second dwell and 20-second arrival timeout |
| Jitter or shaking | Check base clamping, horn/bracket play, supply integrity and wiring; inspect PWM timing before attributing the cause to interpolation |
| Speed change has no effect | Edit the ESP32 sketch constant and upload; restarting Python does not update firmware |
| PWM failure error | Inspect board/pin/core configuration and fault cause; firmware requires restart; verify outputs before operating again |

Diagnose one change at a time. Record the command frame, ACK/DONE, supply condition and observed behavior so software and mechanical causes can be distinguished.

## 16. Verification and maintenance

### Verification performed for this documentation revision

On **3 November 2026**:

- Checked current Python configuration, FK/IK/state/transport/playback sources, ESP32 sketch/headers, local READMEs, and MATLAB model/FK/mapping files.
- Ran the desktop suite: **57 tests passed**, including kinematics/reference fixtures, GUI behavior, integer mapping, invalid-input holds, loop sequencing and fake-serial transport.
- Rebuilt and ran all three ESP32 host harnesses with C++11 warning checks: **parser, firmware and motion passed**. Coverage includes 10,000 live parser frames and 400 synchronized motion cases across 25/50 degrees/s settings.
- Checked the equations and reference-pose values against executable FK and the documented configuration.

This revision did not upload firmware, operate the arm, rerun MATLAB itself, compile against the ESP32 Arduino core, or validate CAD assembly geometry. Host firmware tests use simulated UART/LEDC. Physical shaft accuracy, timing, supply behavior, load limits and collision-free travel remain separate bench checks.

### Repeat the desktop suite

```powershell
Set-Location 'E:\Work\Robotic arm\Code\arm_interface'
python -m unittest discover -s tests -v
```

### Rebuild/run the ESP32 host harnesses

Requires a working `g++` C++11 compiler:

```powershell
Set-Location 'E:\Work\Robotic arm\Code\esp32_arm_receiver'
g++ -std=c++11 -Wall -Wextra -Werror -pedantic tests\test_protocol.cpp -o tests\test_protocol.exe
.\tests\test_protocol.exe
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I tests\stubs tests\test_firmware.cpp -o tests\test_firmware.exe
.\tests\test_firmware.exe
g++ -std=c++11 -Wall -Wextra -Werror -pedantic tests\test_motion.cpp -o tests\test_motion.exe
.\tests\test_motion.exe
```

Only run a harness after its compilation succeeds. The receiver README also provides the workstation-specific Arduino CLI compile command and core configuration.

### When changing the model or controller

1. Update Python `robot_config.json` and MATLAB `RRR_model.m` together. If the frame/mounting changes, update both mapping implementations too.
2. Keep twists `[90,-180,0]` for this analytical IK. A different twist convention requires a new derivation and matching tests.
3. Keep servo command limits consistent in Python `JOINT_LIMITS` and firmware `MIN_ANGLE`/`MAX_ANGLE`; check derived DH limits, home and loops.
4. If intentionally changing the MATLAB reference, regenerate the app fixture using `Code/arm_interface/tests/export_matlab_reference.m`, then rerun tests.
5. If firmware constants, pinout, home or protocol change, rebuild/upload and check the matching app behavior. Avoid treating stale executables/logs as fresh verification.
6. Perform small unloaded physical checks of startup, directions, endpoint clearance, lost-link behavior and loop completion before loaded operation.
7. Update this guide and the component READMEs to match the verified behavior.

## 17. Ready-to-say project explanation

> This is a three-joint robotic arm controlled by a Python desktop application and an ESP32. The geometry uses a standard-DH model with millimetre dimensions. Forward kinematics calculates the end-frame position from joint angles; analytical inverse kinematics checks both base-radius choices and both elbow branches. The controller rejects unreachable, out-of-limit and singular targets, then converts a valid solution into integer servo commands using the mounting offsets and directions. The PC repeatedly sends the latest accepted triple over UART, and the ESP32 generates servo PWM with one shared trajectory duration so all commanded PWM endpoints finish together. A repeating pose loop waits for software completion and then holds for two seconds. The system has command acknowledgements but no shaft-position feedback, so actual accuracy, arrival and load performance must be measured on the mechanism.
