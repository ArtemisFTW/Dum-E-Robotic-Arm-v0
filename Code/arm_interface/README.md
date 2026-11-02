# Arm Console — local FK/IK interface

A Python/Tkinter desktop interface using the current MATLAB standard-DH model.
FK, IK, physical joint limits and calibrated servo command conversion are active.
**Live UART control is enabled.** Explicitly select the ESP32 COM port and click
**Connect & stream** to send commands. The app never connects automatically.
Matching firmware and wiring instructions are in `..\esp32_arm_receiver\README.md`.

## Start

Requires Python 3.10+ with Tkinter (included in the standard Windows Python installer).
Install dependencies once, then start the app:

```powershell
cd 'E:\Work\Robotic arm\Code\arm_interface'
python -m pip install -r requirements.txt
python app.py
```

After installation, double-click `launch.bat`. NumPy handles matrix operations;
pyserial supplies port enumeration and live UART transport. MATLAB is not needed
to run the app.

## Try the interface

- **Jog:** three live joint sliders with paired numeric boxes. The command preview
  changes immediately as a slider moves; a connected ESP32 receives updates at
  20 Hz. Press Enter or leave a numeric box to apply it.
- **FK:** joint sliders and boxes remain editable. Cartesian controls become
  read-only FK position readouts. Joint sliders always show **servo commands**;
  the signed DH angles are displayed separately below them.
- **IK:** X/Y/Z sliders and boxes become editable; joint controls are disabled.
  Sliders, Enter and focus changes only prepare the target. **Solve XYZ** solves
  all three boxes as one target and sends only a valid rounded command. IK starts
  at the current FK position without changing commands.
  **Use current pose** restores the target to the currently accepted command's FK.
- **Limits:** J1 = J2 = J3 = 0–180°. Invalid, out-of-limit,
  NaN, and infinite numeric input shows a bottom error message and preserves the
  previous accepted command. Joint sliders, numeric boxes, angle readouts and
  command frames use whole-degree integers. Fractional joint input rounds to the
  nearest degree (half degrees round up for servo commands). XYZ sliders move
  at 0.1 mm precision; numeric targets accept extra decimal places. Slider arrow
  keys step by 1. The achieved XYZ readout uses the actual rounded servo commands.
- **Pause input:** locks both sets of controls. A connected UART continues
  repeating the last accepted target; PWM remains active. It also stops loop
  cycling. Servos can finish the current speed-limited move.
- **UART panel:** refresh/select the ESP32 COM port at **115200 baud**, then
  **Connect & stream**. After a two-second board-startup allowance the app sends
  the currently displayed frame every 50 ms, with no queue of stale slider values.
  Opening the port can reset the board and command startup home. Disconnect or
  close the app to stop sending. After its one-second link timeout the ESP32
  freezes the current PWM position, including during a move.
- **Command preview:** newline-delimited ASCII in `J1,J2,J3` order, e.g. `90,45,60\n`.
  Copy frame includes the actual newline character. Every angle is an integer,
  e.g. `110,60,40\n`; decimal points are never included in command frames.
- **Angle calculations:** the command panel shows each accepted DH angle's offset
  and direction conversion, e.g. `J1 = round(90 − (20)) = 70°`. These DH values
  correspond to the accepted, rounded commands, rather than an unrounded IK
  candidate. Invalid IK keeps these calculations and commands unchanged.
  **Last UART write** shows the exact most recent complete frame (with the newline
  displayed as `\n`), or `none` before transmission in the current connection.
  A successful write is distinct from an ESP32 ACK or physical position feedback.
- **Activity:** recent local mode changes and messages. Nothing is written to disk.
- **Shortcuts:** Ctrl+1 Jog, Ctrl+2 FK, Ctrl+3 IK; Escape restores a numeric box.

The app starts at the requested servo home command **`90,45,180`**, mapping to DH
`[0,45,90]°`. The ESP32 independently commands that home on every boot/reset,
holds it for two seconds, then accepts live frames. These are commanded angles,
not measured shaft positions. This home pose is nonsingular in the current
model. Straight-link poses remain available in manual Jog/FK, while IK rejects
singular solutions as before. The Cartesian
span of −500 to +500 mm is only a UI range; actual reach is checked by the solver.
Bottom messages remain visible while the main controls scroll on smaller screens.

## Reduced speed and Loop page

Upload the updated `esp32_arm_receiver.ino` with both `servo_protocol.h` and
`servo_motion.h` alongside it, then restart the app. Post-startup moves are
coordinated across all three joints using a **shared move duration**. The joint
with the longest travel is limited to **50 degrees/second**; shorter moves use
proportionally lower speeds so every moving joint reaches its final commanded
position together. The ESP32 updates all three PWM commands every 20 ms using
integer microseconds, while app angles and UART targets remain integer degrees.
Tune `SERVO_STEP_INTERVAL_MS` in the sketch (default 20 ms per degree of the
longest move; larger is slower) and upload again. The app continues sending target
integers at 20 Hz; repeated frames do not restart movement. The initial startup
home is still applied directly because the shaft positions at reset are unknown.

1. Open **Loop** beside **Control**. Choose **Joints** (servo degrees, 0–180) or
   **XYZ** (app frame, mm), enter three values, then **Add pose**. Both types can
   be mixed. XYZ poses use all existing IK, limit, singularity and rounding
   checks. Invalid poses are rejected without changing the arm's command.
2. Add as many poses as needed; the scrollable list has no fixed pose-count cap.
   It shows the input, resolved integer servo angles, and FK coordinates. Select
   a row to edit it with **Update selected**, remove it, or move it up/down.
   **Use current command** fills the editor without sending anything. Lists live
   in memory for this app session.
3. Connect UART and wait for **STREAMING**, then press **Send loop**. Every pose
   is revalidated before playback. The list repeats in order until stopped.
   The list and manual controls lock during playback; you can still switch pages
   to inspect the current accepted command and angle calculations.
4. The ESP32 responds `OK,J1,J2,J3` when accepting a target and
   `DONE,J1,J2,J3` when its speed-limited PWM setpoints reach that target.
   The app then holds for **two full seconds** before sending the next pose.
   A move may take up to 3.6 seconds for 180 degrees, plus the two-second hold.
5. **Stop loop** ends cycling and keeps streaming the current target; the arm
   may finish moving to it. Disconnect, firmware errors, missing ACKs, or no
   matching DONE within 20 seconds stop sequence advancement. Older firmware
   without DONE must be updated to run loops.

ACK/DONE report software commands, not encoder measurements. The achieved FK
readout is calculated from the accepted rounded **target**, rather than physical
shaft feedback or an intermediate point in the speed-limited move. Joint-space
interpolation does not enforce Cartesian straight lines or collision avoidance.

## DH equations and servo mapping

The equations are ported from the current files in
`C:\Users\hrish\Desktop\new files\MATLAB`: `RRR_model.m`, `RRR_FK.m` and
`RRR_IK_solve.m`. Geometry and calibration are in `robot_config.json`. Open
**Model / mapping** to inspect the active model and mapping. Edit the JSON and
restart to change them.

| Joint | a (mm) | d (mm) | alpha (°) | Default servo conversion | Physical command limits | Effective DH limits |
|---|---:|---:|---:|---|---|---|
| J1 | 10.35 | 45.74 | 90 | `J1 = 90 - theta1` | 0–180° | −90 to +90° |
| J2 | 125.7 | 6.24 | −180 | `J2 = theta2` | 0–180° | 0 to +180° |
| J3 | 105 | 6.2 | 0 | `J3 = theta3 + 90` | 0–180° | −90 to +90° |

The general mapping is `servo = offset + direction * theta`, inverted before FK.
Each direction is +1 or −1; the configured directions are **[-1,+1,+1]** and
offsets are **[90,0,90]°**. **J1 is reversed:** servo 0/90/180° maps to DH +90/0/−90°.
J2 servo 0/90/180° means horizontal/upright/left, mapping to DH 0/90/180°.
J3 servo 90° means straight; with J2 horizontal, J3 0° points the distal link
up and 180° points it down. J3 maps to DH −90/0/+90°. The conversion is applied once
in this mapping, before FK and after IK. UART frames already contain the final
servo angles; the ESP32 must not invert J1 again.
This mounting calibration follows the supplied photograph at servo `[90,0,90]`:
DH `[0,0,0]`, with the links straight along +X. The old all-90° offsets wrongly
treated that pose as vertical. No X/Z swap is needed. DH geometry does not
measure horn alignment. No modulo-180 mapping or saturation is used.

Let `delta = d2-d3` and `r = a1+a2*cos(theta2)+a3*cos(theta2-theta3)`:

```text
x = r*cos(theta1) + delta*sin(theta1)
y = r*sin(theta1) - delta*cos(theta1)
z = d1 + a2*sin(theta2) + a3*sin(theta2-theta3)
```

Trig uses degrees. FK independently multiplies all three standard-DH transforms.
IK tries both signed radii and both elbow branches, validates candidates by DH
FK, converts to servo space, checks physical limits and picks the acceptable
branch nearest the current commands. The position Jacobian's normalized minimum
singular value rejects singular/near-singular IK candidates (threshold 0.001).

### Displayed/app XYZ frame

The standard-DH FK equations above remain the mathematical reference. The app
and MATLAB viewer display **`[X,Y,Z] = [DH_X,DH_Y,DH_Z]`**, with Z upward.
Y has been reversed from the previous display convention, exchanging +Y and −Y.
IK targets and Cartesian sliders use this same app
frame: the solver receives `[X,Y,Z]` in DH coordinates. The transformation
is applied once in each direction. Configure `cartesian_axis_signs` in the JSON.

For example, DH `[20,30,-50]°` gives displayed/app XYZ
`[129.1673,46.9705,211.9948] mm`, and command **`70,30,40\n`**.
Home is servo `[90,45,180]`, DH `[0,45,90]`; its displayed XYZ is
`[173.48,-0.04,60.38] mm`.

Physical reference checks using the existing dimensions (mm):

| Servo J1,J2,J3 (°) | Pose | App X | App Y | App Z |
|---|---|---:|---:|---:|
| 90,0,90 | Photograph: straight, maximum +X | 241.05 | -0.04 | 45.74 |
| 90,45,180 | Startup home | 173.48 | -0.04 | 60.38 |
| 90,90,90 | Shoulder upright | 10.35 | -0.04 | 276.44 |
| 90,180,90 | Shoulder points left | -220.35 | -0.04 | 45.74 |
| 90,0,0 | Distal link upright | 136.05 | -0.04 | 150.74 |
| 90,0,180 | Distal link downward | 136.05 | -0.04 | -59.26 |

The small Y residual is `d3-d2 = -0.04 mm`, displayed as zero at 0.1 mm precision.
The horizontal Z value is the existing model's `d1 = 45.74 mm`, measured from
the DH base origin. Verify that origin and the end-effector reference point on
the hardware if an absolute height differs; no dimensions were inferred from
the photograph. IK commands now use the corrected mounting; Jog/FK continue to
send the same entered integer servo angles. Firmware inversion is unnecessary.

## Failure and hold behavior

Unreachable targets, axial exclusion, invalid servo-domain joint limits,
singularities, near-singularities, and failed FK checks show a bottom message.
The last accepted joint triple and command frame remain unchanged. Failed XYZ
targets remain editable so subsequent edits can fix them. After a valid solve,
the error clears and the joint sliders reflect the converted servo commands.
IK errors persist until resolved, reset to the current pose, or the mode changes.
Whole-degree servo rounding is checked again for singularity and FK displacement
(up to ±0.5° rounding per joint); the achieved
XYZ/error is displayed. This validates positions, not collision-free motion paths.

The worker thread repeatedly sends only the latest accepted integer command.
Malformed frames and joint-limit violations are independently rejected by the
ESP32. Serial write/open errors close the link and appear at the bottom; there
is no automatic reconnect. ACK counts confirm the ESP32 accepted the target,
not that the shaft reached its angle. No recent ACK displays a
warning. Close Serial Monitor and other programs using the same COM port first.

## Verify

```powershell
python -m unittest discover -s tests -v
```

The suite includes 37 MATLAB-exported DH-transform/Jacobian/IK reference cases,
400 random FK→IK round trips, independent closed-form and finite-difference
Jacobian checks, reversed servo directions, quantized output, failure holds,
joint limits and interface tests. `tests/matlab_reference.json` is an exported
reference fixture, so normal tests do not require MATLAB. Regenerate it by running
`tests/export_matlab_reference.m` in MATLAB after intentional model changes.
The export only writes the app's fixture and does not change the MATLAB sources.
Additional transport tests use a fake serial device to check actual threaded
writes, latest-value streaming, integer framing, pause/IK holds, startup delay,
ACKs, disconnection and partial writes. These checks do not prove servo
calibration, physical USB/UART delivery or arm motion.
