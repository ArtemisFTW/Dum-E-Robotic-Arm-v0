"""Run the real servo_console.c against a minimal simulated HAL using host GCC.

This tests software behavior only. Run: python tests/test_servo_console.py
"""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


PROJECT = Path(__file__).resolve().parents[1]
MOCK_HAL = r'''
#ifndef MOCK_HAL_H
#define MOCK_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef struct { uint32_t compare; uint32_t counter; } TIM_HandleTypeDef;
typedef struct { uint32_t instance; } UART_HandleTypeDef;
#define TIM_CHANNEL_1 0U
#define USART2_IRQn 38
#define __HAL_TIM_SET_COMPARE(t,c,v) ((t)->compare = (v))
#define __HAL_TIM_SET_COUNTER(t,v) ((t)->counter = (v))
#define __HAL_UART_CLEAR_OREFLAG(u) ((void)(u))
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
void HAL_NVIC_DisableIRQ(int);
void HAL_NVIC_EnableIRQ(int);
void Error_Handler(void);
#endif
'''
HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "PRODUCTION_SOURCE"

static uint32_t tick;
static TIM_HandleTypeDef timer;
static UART_HandleTypeDef uart;
static uint8_t *receive_destination;
static unsigned replies;
static unsigned aborts;
static unsigned checks;

uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *t, uint32_t c)
{ (void)t; (void)c; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{ (void)u; assert(n == 1); receive_destination = b; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, uint8_t *b, uint16_t n, uint32_t t)
{ (void)u; (void)b; (void)n; (void)t; ++replies; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u)
{ (void)u; ++aborts; return HAL_OK; }
void HAL_NVIC_DisableIRQ(int n) { (void)n; }
void HAL_NVIC_EnableIRQ(int n) { (void)n; }
void Error_Handler(void) { abort(); }

static void reset(void)
{
  tick = 0;
  ServoConsole_Init(&timer, &uart);
  assert(servo_command_us == 1500 && servo_target_angle == 90);
}
static void send(const char *text)
{
  while (*text)
  {
    *receive_destination = (uint8_t)*text++;
    HAL_UART_RxCpltCallback(&uart);
    ServoConsole_Poll();
  }
}
static void burst(const char *text)
{
  while (*text)
  {
    *receive_destination = (uint8_t)*text++;
    HAL_UART_RxCpltCallback(&uart);
  }
}
static void advance(uint32_t milliseconds)
{
  for (uint32_t i = 0; i < milliseconds; ++i)
  {
    ++tick;
    ServoConsole_Poll();
    assert(timer.compare >= 500 && timer.compare <= 2500);
  }
}
#define CHECK(condition) do { assert(condition); ++checks; } while (0)

int main(void)
{
  uint32_t previous = 0;
  for (uint32_t angle = 0; angle <= 180; ++angle)
  {
    uint32_t pulse = Servo_PulseForAngle(angle);
    CHECK(pulse >= 500 && pulse <= 2500 && pulse > previous);
    CHECK(Servo_AngleForPulse(pulse) == angle);
    previous = pulse;
  }
  CHECK(Servo_PulseForAngle(0) == 500);
  CHECK(Servo_PulseForAngle(90) == 1500);
  CHECK(Servo_PulseForAngle(180) == 2500);

  reset();
  advance(60000);
  CHECK(servo_command_us == 1500 && sweep_enabled == 0);
  send("0");
  CHECK(servo_target_angle == 90);
  send("\n");
  CHECK(servo_target_angle == 0 && target_us == 500);
  advance(4000);
  CHECK(servo_command_us == 500);
  advance(30000);
  CHECK(servo_command_us == 500);
  send("180\n");
  advance(8000);
  CHECK(servo_command_us == 2500);

  unsigned before = replies;
  send("45\r\n");
  CHECK(servo_target_angle == 45 && replies == before + 1);
  advance(8000);
  CHECK(servo_command_us == 1000);
  const char *invalid[] = {"-1\n", "181\n", "90.5\n", "90abc\n", "1 2\n",
                          "99999999999999999999999999999\n", "sweepBAD\n", "\n", "\00190\n"};
  for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
  {
    send(invalid[i]);
    CHECK(servo_target_angle == 45 && target_us == 1000 && sweep_enabled == 0);
  }
  send(" \t135 \t\r");
  CHECK(servo_target_angle == 135 && target_us == 2000);
  send("00090\n");
  CHECK(servo_target_angle == 90);
  send("12\b5\n");
  CHECK(servo_target_angle == 15);
  send("000000000000000000000000000000000000000000\n");
  CHECK(servo_target_angle == 15);
  send("75\n");
  CHECK(servo_target_angle == 75);

  reset();
  burst("0\n180\n90\n");
  ServoConsole_Poll();
  CHECK(servo_target_angle == 90);
  char overflow[181];
  memset(overflow, '0', sizeof(overflow) - 1);
  overflow[sizeof(overflow) - 1] = '\0';
  burst(overflow);
  CHECK(rx_fault != 0);
  ServoConsole_Poll();
  CHECK(servo_target_angle == 90 && rx_fault == 0 && aborts > 0);
  send("180\n"); /* Tail of damaged line must not apply. */
  CHECK(servo_target_angle == 90);
  send("45\n");
  CHECK(servo_target_angle == 45);

  send("1");
  HAL_UART_ErrorCallback(&uart);
  ServoConsole_Poll();
  send("80\n");
  CHECK(servo_target_angle == 45);
  send("30\n");
  CHECK(servo_target_angle == 30);
  UART_HandleTypeDef other_uart = {0};
  HAL_UART_ErrorCallback(&other_uart);
  CHECK(rx_fault == 0);

  reset();
  send("sweep\n");
  CHECK(sweep_enabled == 1 && servo_target_angle == 0);
  uint32_t minimum = 2500, maximum = 500;
  for (uint32_t i = 0; i < 23000; ++i)
  {
    advance(1);
    if (servo_command_us < minimum) minimum = servo_command_us;
    if (servo_command_us > maximum) maximum = servo_command_us;
  }
  CHECK(minimum == 500 && maximum == 2500 && servo_sweep_count >= 1);
  send("90\n");
  CHECK(sweep_enabled == 0 && servo_target_angle == 90);
  advance(40000);
  CHECK(servo_command_us == 1500);

  reset();
  send("180\n");
  advance(1000);
  CHECK(servo_command_us == 1750);
  send("hold\n");
  CHECK(target_us == 1750 && sweep_enabled == 0);
  advance(10000);
  CHECK(servo_command_us == 1750);

  tick = UINT32_MAX - 10U;
  ServoConsole_Init(&timer, &uart);
  send("180\n");
  advance(40);
  CHECK(servo_command_us == 1510);
  printf("PASS: %u checks; all angles, parsing, hold/sweep, UART recovery, tick rollover.\n", checks);
  return 0;
}
'''


def main():
    sys.path.insert(0, str(PROJECT))
    from build_firmware import verify_uart_setup
    verify_uart_setup(PROJECT)
    print("USART2 interrupt integration checks: PASS", flush=True)
    compiler = shutil.which("gcc")
    if not compiler:
        raise SystemExit("Host GCC is required for software simulation tests.")
    with tempfile.TemporaryDirectory(prefix="servo_console_tests_") as directory:
        temp = Path(directory)
        (temp / "stm32f4xx_hal.h").write_text(MOCK_HAL, encoding="utf-8")
        (temp / "main.h").write_text('#include "stm32f4xx_hal.h"\n', encoding="utf-8")
        harness = HARNESS.replace("PRODUCTION_SOURCE", (PROJECT / "Core/Src/servo_console.c").as_posix())
        source = temp / "test_servo_console.c"
        source.write_text(harness, encoding="utf-8")
        executable = temp / "test_servo_console.exe"
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-I" + str(temp), "-I" + str(PROJECT / "Core/Inc"),
                        str(source), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
