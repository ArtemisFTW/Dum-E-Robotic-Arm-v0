# ESP32 UART0 arm receiver — OT5320M, live three-joint control

Use with the desktop app in `E:\Work\Robotic arm\Code\arm_interface`.
The sketch targets a **classic ESP32/WROOM** using **Arduino-ESP32 3.x**.
It uses built-in LEDC PWM; no ESP32Servo library is required.

## Wiring

| Connection | ESP32 / supply |
|---|---|
| J1 servo yellow signal | **GPIO25** |
| J2 servo yellow signal | **GPIO26** |
| J3 servo yellow signal | **GPIO27** |
| All servo brown wires | External supply negative / GND |
| All servo red wires | External regulated servo supply positive |
| ESP32 GND | Same external supply negative / GND |
| Computer | ESP32 USB data connector |
| UART0, if using a separate 3.3 V USB-UART adapter | Adapter TX → GPIO3/RX0; adapter RX ← GPIO1/TX0; common GND |

The classic development board's USB-UART bridge already routes USB serial to
UART0. With that connector, do not add extra TX/RX jumpers or a second UART
driver. Use a USB **data** cable and close Serial Monitor before connecting the app.

Power the three servos from an external regulated supply, not the ESP32/USB
5 V or 3.3 V pins. Use the servo supply voltage appropriate to the OT5320M and
connect grounds. For a 6 V setup, the datasheet's 2.3 A stall figure per servo
totals 6.9 A for three; allow supply and wiring margin. Do not connect a 6 V
servo rail to an ESP32 power or GPIO pin. The pulse outputs are 3.3 V logic;
verify recognition by the servos because their sheet does not specify the input
threshold. The OT5320M sheet specifies brown/red/yellow wiring and 180° at
500–2500 μs. See the [OT5320M datasheet](https://robu-prod-media.s3.ap-south-1.amazonaws.com/uploads/2021/04/OT5320M-Datasheet-1.pdf).

## Flash in Arduino IDE

1. Open `esp32_arm_receiver.ino`. Keep `servo_protocol.h` and `servo_motion.h` beside the sketch.
2. Install/select **esp32 by Espressif Systems 3.x** in Boards Manager.
3. Select **ESP32 Dev Module** (classic WROOM), then the board's COM port.
4. Use Upload. If necessary hold BOOT while it starts connecting, then release.
5. On every boot/reset the firmware commands **J1=90°, J2=45°, J3=180°**.
   It holds that home command for **2 seconds**, drains/discards incoming move
   frames during that interval, then announces `READY,ARM_UART0,115200`.

**Power-up/reset now causes motion toward home**, as requested. The first
bench check should have the mechanism unloaded/clear of obstructions. The
two-second interval is a settling allowance, not measured confirmation that
the servos reached home; there is no shaft-position feedback.

## Use the app for live jog

```powershell
cd 'E:\Work\Robotic arm\Code\arm_interface'
python -m pip install -r requirements.txt
python app.py
```

1. Restart the app to load this version. It starts with **90,45,180**.
2. Click **Refresh ports**, select the ESP32 COM port and keep **115200** baud.
3. Click **Connect & stream**. Opening USB serial may reset the board and home
   the arm. The app allows a two-second startup wait; the firmware also enforces
   its boot home interval. Commands during that interval are discarded and the
   next repeated frame is accepted when ready.
4. In **Jog**, move J1/J2/J3 sliders. The newest integer command is transmitted
   every **50 ms (20 Hz)**. All three joints update on each accepted frame.
5. In FK, joint entry streams the same way. In IK, valid converted results stream;
   invalid targets repeat the previous accepted triple, so the arm holds.
6. Check **sent** and **ACK** counts. `OK,90,45,60` acknowledges validated command
   application to PWM, not physical shaft position. No recent ACK means check
   baud, firmware, USB connection, and the selected COM port.
7. **Pause input** locks the interface and continues sending the last command.
   **Disconnect** stops the stream and closes the port. A running ESP32 holds
   its last pulse widths. A board reset instead performs startup home again.

The app never auto-connects or auto-reconnects. A new explicit connection sends
the currently displayed command; only a fresh app instance starts at home.

## Protocol and behavior

**115200 baud, 8 data bits, no parity, 1 stop bit; newline-terminated ASCII.**

```text
90,45,180\n
120,60,90\n
```

`\n` represents the actual line-feed byte, not two literal characters. CRLF is
also accepted. All three values are **servo-domain integers**. The app already
applies the mounting correction: **J1 = 90 − theta1**, **J2 = theta2**,
J3 = 90 + theta3. Do **not** invert J1 or add offsets again on the ESP32.
J1 servo 0/90/180° corresponds to DH +90/0/−90°; J2 uses 0/90/180°
(horizontal/upright/left), while J3 uses −90/0/+90° (90° servo is straight).
The app's XYZ mapping `[DH_X,DH_Y,DH_Z]` is only a coordinate-frame
conversion and does not change this UART contract or PWM output mapping.

| Joint | Accepted commands | PWM range at those endpoints |
|---|---|---|
| J1 | 0–180° | 500–2500 μs |
| J2 | 0–180° | 500–2500 μs |
| J3 | 0–180° | 500–2500 μs |

PWM is 50 Hz with 16-bit LEDC duty resolution. Angle and pulse calculations use
integers. After startup, all three joints use **one shared move duration**,
calculated from the longest joint travel at a maximum of **50 degrees/second**.
Each joint follows the same fraction of its own travel, so shorter moves run
slower and all moving PWM channels reach their targets on the **same update**.
For example, 90°, 45°, and 180° travel take about 3.6 seconds together, at
25, 12.5, and 50 degrees/second respectively.

PWM commands update together every **20 ms** using integer microseconds. This
gives finer coordinated motion than whole-degree intermediate steps; UART
inputs, app angles and final targets remain whole-degree integers. Change
`SERVO_STEP_INTERVAL_MS` (milliseconds per degree of the longest move; default
20) in the sketch and upload again to tune speed: larger values are slower.
This halves the previous 40 ms/degree delay. The original direct-target firmware
had no numerical speed cap, so its actual speed would need measuring to calculate
an exact midpoint in physical speed.
Repeated frames do not restart the common trajectory. Retargeting starts from
the current intermediate PWM widths and calculates a new shared duration.
A delayed MCU loop never catches up through a large jump. This coordinates
commanded positions; actual shaft arrival is not measured, and there is no
separate acceleration profile. Startup home is still commanded directly because
the initial shaft angles are unknown.

`OK,J1,J2,J3` acknowledges target acceptance. `DONE,J1,J2,J3` reports that all
three commanded PWM angles have reached the target; it is also returned on
heartbeats while at that target. Neither response is measured shaft feedback.
The app's Loop page waits for a matching fresh DONE, holds for two seconds, and
then advances to the next pose. The input frame format remains unchanged.

The receiver requires a complete valid triple before changing any commanded
joint. Floats, signs, extra/missing fields, garbage, long frames, embedded NUL,
and out-of-limit values are rejected. Partial lines expire after 250 ms and are
discarded through the next newline. Invalid frames preserve all previous
targets; errors are rate limited. An existing speed-limited move can continue
towards the last accepted target after a malformed frame. After one second without a valid command,
`HOLD,LINK_TIMEOUT` is emitted once and the last PWM widths continue. Link loss
freezes the current intermediate PWM command and cancels remaining travel. It
does **not** home or detach the servos. PWM setup/write failure emits an error
and requires a restart. The firmware has no FK/IK or collision model.

## Verify without connected hardware

From this folder, with a C++11 compiler and the app's Python requirements:

```powershell
g++ -std=c++11 -Wall -Wextra -Werror -pedantic tests\test_protocol.cpp -o tests\test_protocol.exe
.\tests\test_protocol.exe
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I tests\stubs tests\test_firmware.cpp -o tests\test_firmware.exe
.\tests\test_firmware.exe
g++ -std=c++11 -Wall -Wextra -Werror -pedantic tests\test_motion.cpp -o tests\test_motion.exe
.\tests\test_motion.exe
python -m unittest discover -s ..\arm_interface\tests -v
```

The parser harness checks 10,000 frames plus boundary/error cases. The sketch
harness uses simulated UART/LEDC to check startup home, dwell, output mapping,
target acknowledgements, completion reports, speed limits, invalid-frame hold,
mid-move timeout hold, and recovery. The motion harness checks shared progress
and simultaneous finish across 200 varied moves, the speed envelope, repeated
targets, retargeting, no catch-up jumps, holds/resume and clock rollover. The
Python tests exercise actual worker threading against a fake serial device,
including live slider publication and last-valid-command behavior.

`arduino-cli.local.yaml` points at this workstation's installed core. Compile
without flashing from PowerShell:

```powershell
& 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' --config-file arduino-cli.local.yaml compile --fqbn esp32:esp32:esp32 --build-path build --warnings all .
```

Compilation and simulated tests do not verify physical wiring, USB delivery,
power supply, pulse timing, calibration, or actual arm motion. Do the board
upload and unloaded home/live-jog check before attaching loads.

Implementation references: [Espressif UART API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/serial.html)
and [Espressif LEDC API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html).
