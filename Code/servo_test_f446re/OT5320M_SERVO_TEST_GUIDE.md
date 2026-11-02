# OT5320M full-range servo test with USART2 angle control

The firmware now starts at **90 degrees**, accepts an integer **0–180** through **USART2**, and holds that target until the next valid command. Send **`sweep`** to run **0 → 180 → 0** repeatedly. Sending an angle cancels the sweep and holds the new target.

The old code used **1200–1800 µs**, explaining its smaller travel. This version uses the OT5320M's published **500–2500 µs** span. Commanded angles are nominal; use a protractor or angle fixture to verify the physical shaft position.

Project folder: `E:\Work\Robotic arm\Code\servo_test_f446re`.

## 1. Check the pinout and connect the servo power

These header locations assume **NUCLEO-F446RE / MB1136**. For another STM32F446RE board, locate the same MCU pins using that board's schematic.

| Function | MCU pin | Nucleo connection |
|---|---|---|
| Servo pulse | **PA6 / AF2 / TIM3_CH1** | **D12, CN5 pin 5** |
| Serial2 transmit | **PA2 / AF7 / USART2_TX** | Normally routed internally to ST-LINK virtual COM |
| Serial2 receive | **PA3 / AF7 / USART2_RX** | Normally routed internally from ST-LINK virtual COM |
| Board ground | GND | CN6 pin 6 or 7 |
| Board power and programming | ST-LINK USB | CN1 USB connector |

The [STM32F446 datasheet, alternate-function table](https://www.st.com/resource/en/datasheet/stm32f446re.pdf) documents the MCU pin functions. The [Nucleo manual, Table 19 and section 7.10](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf) documents the headers and virtual COM routing.

The [OT5320M datasheet](https://robu-prod-media.s3.ap-south-1.amazonaws.com/uploads/2021/04/OT5320M-Datasheet-1.pdf) specifies **brown = GND, red = VCC, yellow = signal**. It lists a **4.0–8.4 V** supply and **2.3 A stall current at 6 V**. For this unloaded test, use a regulated **6 V supply capable of at least 3 A**.

```text
PC USB ───────────────── ST-LINK USB on Nucleo
                         └─ virtual COM port ↔ USART2 (PA2 / PA3)

External +6 V ────────── Servo RED
External negative ────── Servo BROWN
        └─────────────── Nucleo GND
Nucleo PA6 / D12 ─────── Servo YELLOW
```

Keep the servo power wiring directly between its supply and servo. Do not supply the servo from the board's 3.3 V or 5 V pin. Do not connect the 6 V servo rail to the board power pins or GPIO.

PA6 outputs 3.3 V logic. The servo sheet does not specify an input-high threshold; direct operation must be checked on the bench. Your reported successful sweep confirms operation of your existing signal connection under the conditions tested.

Secure the servo body and remove the arm linkage/load before checking the full endpoints. The first move to centre can be sudden because the STM32 has no shaft-position feedback. Keep servo power off and yellow disconnected while changing or flashing firmware.

## 2. Open the existing STM32CubeIDE project

1. Open STM32CubeIDE.
2. Import `E:\Work\Robotic arm\Code\servo_test_f446re` through **File → Import → General → Existing Projects into Workspace**, if needed.
3. Right-click the project and select **Refresh / F5** so the IDE sees the added source files.
4. The firmware has already been edited. Do not paste the old limited-sweep code back into `main.c`.
5. Open `servo_sweep_test.ioc` to inspect the following configuration. Code regeneration is optional; verify the settings below before regenerating.

## 3. Check system pins and clock configuration

In **Pinout & Configuration → System Core → SYS**, use **Serial Wire** debugging and **SysTick** as the HAL timebase. Keep PA13/PA14 available for SWD.

In **System Core → RCC**, leave HSE and LSE disabled for this test. In **Clock Configuration**, select the internal **16 MHz HSI directly as SYSCLK**, with the PLL unused.

| Clock | Setting | Frequency |
|---|---|---|
| System clock source | HSI | 16 MHz |
| AHB | /1 | 16 MHz HCLK |
| APB1 | /1 | 16 MHz PCLK1 |
| APB2 | /1 | 16 MHz PCLK2 |
| TIM3 timer input | With these prescalers | **16 MHz** |

This is the existing clock setup. Do not change the timer prescaler below unless the actual TIM3 clock changes. The internal HSI has tolerance; verify timing with a scope if available.

## 4. Check the servo PWM configuration

In **Timers → TIM3**:

| Setting | Value |
|---|---|
| Clock Source | Internal Clock |
| Channel 1 | PWM Generation CH1 |
| Output pin | PA6 / AF2_TIM3 |
| Prescaler | **15** |
| Counter Mode | Up |
| Counter Period / ARR | **19999** |
| Clock Division | DIV1 |
| Auto-reload preload | Enable |
| CH1 mode | PWM mode 1 |
| Initial pulse / CCR1 | **1500** |
| Polarity | High |
| Fast Mode | Disable |
| PA6 GPIO | Alternate-function push-pull, no pull, low speed |
| Timer interrupts / DMA | Disabled |

```text
TIM3 count frequency = 16,000,000 / (15 + 1) = 1,000,000 Hz
One count            = 1 microsecond
Frame period         = (19999 + 1) microseconds = 20 ms = 50 Hz
CCR1 = 500           → 0.5 ms high, nominal 0 degrees
CCR1 = 1500          → 1.5 ms high, nominal 90 degrees
CCR1 = 2500          → 2.5 ms high, nominal 180 degrees
```

The servo sheet specifies the pulse-width span; 50 Hz is the bench-test frame rate used here, rather than a frame-frequency guarantee from that sheet.

## 5. Configure Serial2 / USART2

**If startup text appears but entering an angle produces no acknowledgement:** the current failure was traced to a missing USART2 NVIC enable and interrupt handler. The receive code uses interrupts, whereas the startup text uses polling transmit; transmit can therefore work while reception does not. The source and `.ioc` have been corrected. Keep 115200 and NLCR, flash the corrected ELF, then send `45` and expect `Target=45deg, pulse=1000us. Moving, then holding.`

In **Connectivity → USART2**:

| Setting | Value |
|---|---|
| Mode | Asynchronous |
| Baud rate | **115200** |
| Word length | **8 bits** |
| Parity | None |
| Stop bits | 1 |
| Data direction | Transmit and Receive |
| Hardware flow control | None |
| Oversampling | 16 |
| TX | **PA2 / AF7_USART2** |
| RX | **PA3 / AF7_USART2** |
| GPIO | Alternate-function push-pull, no pull |
| NVIC | **USART2 global interrupt enabled**, priority 5, subpriority 0 |

The `.ioc`, initialization code, UART HAL enable, GPIO setup and IRQ handler have been updated. `USART2_IRQHandler()` calls `HAL_UART_IRQHandler(&huart2)`. The application receives bytes with interrupts and parses complete lines in the main loop.

If the `.ioc` is already open in CubeMX when these files are edited, close and reopen/reload it from disk before regenerating. In **USART2 → NVIC Settings**, verify **USART2 global interrupt** is checked, priority 5, and save. Under **System Core → NVIC → Code generation**, keep generation of the USART2 IRQ handler and its HAL call enabled. After regeneration, confirm `HAL_NVIC_EnableIRQ(USART2_IRQn)` remains in the UART MSP initialization and `USART2_IRQHandler()` still calls `HAL_UART_IRQHandler(&huart2)`. The build helper and test runner now reject a missing interrupt configuration; a successful ordinary firmware link alone cannot detect this omission because the startup file supplies a weak default handler.

For a standard Nucleo, use the **ST-LINK USB virtual COM port**. No extra serial wires are needed. The [Nucleo manual, section 7.10](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf) describes the default routing: **SB13/SB14 ON; SB62/SB63 OFF**. Check these only if your board was modified or the VCP connection does not work.

For a different board without USB-to-USART2 routing, use a **3.3 V TTL USB-UART adapter**: adapter TX → PA3, adapter RX ← PA2, and common GND. Confirm that PA2/PA3 are physically routed to the chosen connector and are not also driven by another serial device. Do not connect an RS-232 voltage-level port.

## 6. Understand the edited code

| File | Purpose |
|---|---|
| `Core/Src/main.c` | Initializes GPIO, TIM3 and USART2; starts the application and polls it continuously |
| `Core/Src/servo_console.c` | Angle mapping, gradual motion, hold/sweep behavior, line parser, receive queue and UART callbacks |
| `Core/Inc/servo_console.h` | Application API and debugger-visible command variables |
| `Core/Src/stm32f4xx_hal_msp.c` | Peripheral clocks and PA6 / PA2 / PA3 alternate functions |
| `Core/Src/stm32f4xx_it.c` | USART2 interrupt handler |
| `Core/Inc/stm32f4xx_hal_conf.h` | Enables TIM and UART HAL modules |
| `Drivers/STM32F4xx_HAL_Driver/*/stm32f4xx_hal_uart.*` | UART driver copied from installed STM32CubeF4 V1.28.3 |

`main.c` calls this after the peripheral initializers:

```c
ServoConsole_Init(&htim3, &huart2);
```

Its infinite loop calls:

```c
ServoConsole_Poll();
```

In `servo_console.c`, the endpoint constants are:

```c
#define SERVO_ZERO_US          500U
#define SERVO_CENTER_US        1500U
#define SERVO_FULL_US          2500U
#define SERVO_STEP_US          5U
#define SERVO_STEP_MS          20U
#define SERVO_ENDPOINT_MS      500U
```

The [OT5320M sheet](https://robu-prod-media.s3.ap-south-1.amazonaws.com/uploads/2021/04/OT5320M-Datasheet-1.pdf) lists **180-degree operating travel at 500–2500 µs**. The separate potentiometer travel specification is not a command range. The code maps 0/90/180 to the three constants above, with interpolation and rounding between them.

Motion advances by 5 µs every 20 ms, giving approximately **8 seconds for a complete 0-to-180-degree traverse**. Serial input is processed during movement, so a new valid angle can replace the target immediately. The physical servo still needs time to reach that target.

## 7. Build and flash

### Option A: STM32CubeIDE

1. Refresh the project with **F5**.
2. Select **Project → Clean**, then **Build Project**. The IDE should discover `servo_console.c` and the UART driver in the existing Core and Drivers source folders.
3. Check the build console includes both `servo_console.c` and `stm32f4xx_hal_uart.c`, and finishes without errors.
4. If either file is missing from the build, check that it is not excluded from the active build configuration and refresh/rebuild again.
5. Flash the newly built `Debug/servo_sweep_test.elf` using **Debug As → STM32 C/C++ Application**, ST-LINK, SWD.
6. Click **Resume** if execution pauses at `main()`.

### Option B: included local build helper

Run in PowerShell:

```powershell
Set-Location 'E:\Work\Robotic arm\Code\servo_test_f446re'
python build_firmware.py
```

This compiles and links the complete firmware using the installed ARM GCC under `C:\ST`, then creates:

```text
Build_serial_test/servo_sweep_test.elf
Build_serial_test/servo_sweep_test.hex
Build_serial_test/servo_sweep_test.bin
```

It does not flash the board. To use this build, select **Build_serial_test/servo_sweep_test.elf** as the C/C++ Application in the STM32CubeIDE debug configuration, or program the `.hex` with STM32CubeProgrammer. Do not accidentally flash the old Debug ELF after building only with the helper. A raw `.bin` needs Flash start address **0x08000000**; the `.hex` includes addresses.

## 8. Open the serial monitor

1. Connect the PC to the **ST-LINK USB connector** on the board.
2. In Windows Device Manager, look under **Ports (COM & LPT)** for **STMicroelectronics STLink Virtual COM Port (COMx)**. Use the actual listed COM number.
3. Open that COM port in your serial monitor with **115200 baud, 8 data bits, no parity, 1 stop bit, no flow control**.
4. Set line ending to **Newline**, **Carriage return**, or **Both NL & CR**. A command is applied only after its line terminator arrives.
5. Press RESET with the monitor open. Expect:

```text
OT5320M USART2 ready: 115200 8N1. Target=90deg (1500us).
Enter 0..180 + Enter; sweep; hold; help.
```

No Arduino `Serial2` library is used: Serial2 here means the STM32's **USART2 peripheral**.

## 9. Verify individual angles and holding

1. Run the firmware with the servo initially disconnected. If you have a scope, confirm approximately 20 ms frame period and 1.5 ms high at PA6.
2. Confirm the board and servo supply share ground. Turn on the external servo supply, then connect yellow to PA6 while the board is running.
3. The initial target is **90** and stays there. Send these values one line at a time:

```text
90
45
135
10
170
0
180
90
```

4. Let each move finish and measure the horn angle before sending the next command. A full-range move takes about 8 seconds.
5. Expected acknowledgements include:

```text
Target=0deg, pulse=500us. Moving, then holding.
Target=90deg, pulse=1500us. Moving, then holding.
Target=180deg, pulse=2500us. Moving, then holding.
```

6. The servo keeps receiving PWM and holds the most recent valid target indefinitely. Serial silence or disconnecting the terminal does not cause it to recenter. RESET starts it at 90 again; the last target is not saved to Flash.
7. Inputs such as `-1`, `181`, `90.5` or `90abc` are rejected without changing the target or mode. Empty lines are ignored. Spaces around an integer are accepted. Overlong or corrupted lines are discarded; follow the displayed recovery message and resend.

The acknowledgement reports a **command**, not a measured shaft angle or arrival confirmation. `servo_command_us` and `servo_target_angle` in the debugger are also command estimates.

## 10. Test the full repeated sweep

Send this lowercase command followed by Enter:

```text
sweep
```

The servo moves to nominal 0, then repeatedly traverses **0 → 180 → 0**, pausing **0.5 seconds** at each endpoint.

To leave the sweep and hold a chosen angle, send a number, for example:

```text
75
```

To hold the pulse currently being output, including during a move, send:

```text
hold
```

`hold` stops changing the PWM command; the shaft may still finish settling. It does not remove torque. `help` prints available commands.

Disconnect the external servo supply to stop unexpected motion. Stopping the debugger/CPU is not a reliable way to disable torque because the timer can keep sending its last PWM pulse.

## 11. Measure and calibrate the actual range

| Command | Nominal pulse | Measured angle / observations |
|---|---|---|
| 0 | 500 µs | |
| 45 | 1000 µs | |
| 90 | 1500 µs | |
| 135 | 2000 µs | |
| 180 | 2500 µs | |

The old 1200–1800 µs span represented about **54 degrees total** under the nominal mapping. The wider pulse span addresses that restriction, but actual endpoints and linearity still depend on your servo and horn reference.

If the servo buzzes harshly, pushes a mechanical stop, or draws excessive current at an endpoint, remove power and narrow `SERVO_ZERO_US` / `SERVO_FULL_US` before retrying. Keep the constants within **500–2500 µs** and preserve ZERO < CENTER < FULL. Rebuild and flash after changing them. Do not extend beyond the published span to force an angle.

A horn mounting offset shifts your measured zero. Establish the horn reference at a known command and measure total travel separately. Before using an arm linkage, establish mechanical clearance and joint limits independently from these servo commands.

## 12. Troubleshoot

| Symptom | Check |
|---|---|
| No serial text | Correct ST-LINK COM port, 115200 8N1, no flow control; open monitor then RESET; click Resume if paused in debug |
| Startup text appears, but commands get no acknowledgement | Keep 115200 / NLCR; verify USART2 global IRQ is enabled and its handler calls the HAL, as described in Step 5; flash the corrected ELF |
| Text is garbled | Baud/clock mismatch; verify 16 MHz HSI clock and USART2 115200 configuration |
| Old limited sweep still runs | Flash the new ELF and confirm its selected path; remove old pasted servo loop code |
| Servo moves then stays | Expected manual hold; send another integer or `sweep` |
| UART overflow/error | Press Enter to discard the damaged line, then resend one complete command |
| Servo does not reach an angle yet | Wait for the gradual motion; full-range travel takes about 8 seconds |
| Endpoints still do not cover the measured travel | Check pulse width at PA6, supply voltage under motion and horn reference; record actual measurements |
| Jitter, resets or buzzing | Check external supply capacity, voltage sag, ground routing and mechanical binding |
| Linker cannot find ServoConsole or UART functions | Refresh and clean/build so the IDE includes the new application and driver source files |

Normal power-down: disconnect yellow, switch off servo power, then disconnect board USB.

## 13. Validation status

The complete firmware has compiled and linked using the installed ARM toolchain, producing ELF/HEX/BIN files. Core/application code is compiled with `-Wall -Wextra -Werror`; vendor HAL uses its normal warning level. **403 host simulation checks passed**, covering every integer angle, command parsing, hold/sweep behavior, serial error recovery and tick rollover. Repeat them from the project folder with:

```powershell
python tests\test_servo_console.py
```

The tests use host GCC and simulate the HAL around the actual application source. They also check the saved UART interrupt configuration and its source-level connection to the HAL. The build helper verifies that the linked ELF contains a strong USART2 interrupt handler. These checks do not replace flashing the STM32 and measuring the actual servo angles or USB serial connection.
