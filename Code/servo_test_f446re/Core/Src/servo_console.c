#include "main.h"
#include "servo_console.h"
#include <stdio.h>
#include <string.h>

/* OT5320M nominal endpoints. Calibrate the physical angles on your servo. */
#define SERVO_ZERO_US          500U
#define SERVO_CENTER_US        1500U
#define SERVO_FULL_US          2500U
#define SERVO_STEP_US          5U
#define SERVO_STEP_MS          20U
#define SERVO_ENDPOINT_MS      500U
#define RX_CAPACITY            128U
#define LINE_CAPACITY          32U

#if (SERVO_ZERO_US < 500U) || (SERVO_FULL_US > 2500U)
#error "Keep OT5320M pulses within 500..2500 us"
#endif
#if (SERVO_ZERO_US >= SERVO_CENTER_US) || (SERVO_CENTER_US >= SERVO_FULL_US)
#error "Require ZERO < CENTER < FULL"
#endif

volatile uint32_t servo_command_us = SERVO_CENTER_US;
volatile uint32_t servo_target_angle = 90U;
volatile uint32_t servo_sweep_count = 0U;

static TIM_HandleTypeDef *servo_timer;
static UART_HandleTypeDef *console_uart;
static uint32_t target_us = SERVO_CENTER_US;
static uint32_t last_step_ms;
static uint32_t endpoint_ms;
static uint8_t sweep_enabled;
static uint8_t sweep_reached_high;
static uint8_t endpoint_waiting;
static uint8_t rx_byte;
static volatile uint8_t rx_queue[RX_CAPACITY];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint8_t rx_fault;
static char line[LINE_CAPACITY];
static uint32_t line_length;
static uint8_t discard_line;

static void Console_Send(const char *text)
{
  /* Reception remains interrupt-driven while a short reply is transmitted. */
  (void)HAL_UART_Transmit(console_uart, (uint8_t *)text,
                          (uint16_t)strlen(text), 100U);
}

static uint32_t Servo_PulseForAngle(uint32_t angle)
{
  if (angle <= 90U)
    return SERVO_ZERO_US + ((SERVO_CENTER_US - SERVO_ZERO_US) * angle + 45U) / 90U;

  return SERVO_CENTER_US +
         ((SERVO_FULL_US - SERVO_CENTER_US) * (angle - 90U) + 45U) / 90U;
}

static uint32_t Servo_AngleForPulse(uint32_t pulse)
{
  if (pulse <= SERVO_CENTER_US)
    return ((pulse - SERVO_ZERO_US) * 90U +
            (SERVO_CENTER_US - SERVO_ZERO_US) / 2U) /
           (SERVO_CENTER_US - SERVO_ZERO_US);

  return 90U + ((pulse - SERVO_CENTER_US) * 90U +
                (SERVO_FULL_US - SERVO_CENTER_US) / 2U) /
               (SERVO_FULL_US - SERVO_CENTER_US);
}

static void Servo_WritePulse(uint32_t pulse)
{
  if (pulse < SERVO_ZERO_US) pulse = SERVO_ZERO_US;
  if (pulse > SERVO_FULL_US) pulse = SERVO_FULL_US;
  __HAL_TIM_SET_COMPARE(servo_timer, TIM_CHANNEL_1, pulse);
  servo_command_us = pulse;
}

static void Servo_SetAngle(uint32_t angle)
{
  target_us = Servo_PulseForAngle(angle);
  servo_target_angle = angle;
  endpoint_waiting = 0U;
}

static void Console_ReportTarget(void)
{
  char reply[96];
  (void)snprintf(reply, sizeof(reply),
                 "Target=%ludeg, pulse=%luus. Moving, then holding.\r\n",
                 (unsigned long)servo_target_angle, (unsigned long)target_us);
  Console_Send(reply);
}

static void Console_Command(char *command)
{
  char *start = command;
  char *end;
  uint32_t angle = 0U;

  while ((*start == ' ') || (*start == '\t')) ++start;
  end = start + strlen(start);
  while ((end > start) && ((end[-1] == ' ') || (end[-1] == '\t'))) --end;
  *end = '\0';
  if (*start == '\0') return;

  if (strcmp(start, "help") == 0)
  {
    Console_Send("Enter an integer 0..180 + Enter; sweep; hold; help.\r\n");
    return;
  }
  if (strcmp(start, "sweep") == 0)
  {
    sweep_enabled = 1U;
    sweep_reached_high = 0U;
    Servo_SetAngle(0U);
    Console_Send("Sweep: 0 -> 180 -> 0 repeatedly. Enter an angle to hold.\r\n");
    return;
  }
  if (strcmp(start, "hold") == 0)
  {
    sweep_enabled = 0U;
    target_us = servo_command_us;
    servo_target_angle = Servo_AngleForPulse(target_us);
    endpoint_waiting = 0U;
    Console_Send("Holding the current commanded pulse.\r\n");
    return;
  }

  /* Check the whole line before applying anything. No atoi/partial parsing. */
  for (const char *p = start; *p != '\0'; ++p)
  {
    if ((*p < '0') || (*p > '9'))
    {
      Console_Send("ERROR: enter an integer from 0 to 180.\r\n");
      return;
    }
    angle = angle * 10U + (uint32_t)(*p - '0');
    if (angle > 180U)
    {
      Console_Send("ERROR: angle must be 0..180. Target unchanged.\r\n");
      return;
    }
  }

  sweep_enabled = 0U;
  Servo_SetAngle(angle);
  Console_ReportTarget();
}

static void Console_ProcessByte(uint8_t byte)
{
  if ((byte == '\r') || (byte == '\n'))
  {
    if (discard_line != 0U)
      Console_Send("ERROR: line discarded. Send the command again.\r\n");
    else if (line_length != 0U)
    {
      line[line_length] = '\0';
      Console_Command(line);
    }
    line_length = 0U;
    discard_line = 0U;
    return;
  }
  if (discard_line != 0U) return;
  if ((byte == 8U) || (byte == 127U))
  {
    if (line_length != 0U) --line_length;
    return;
  }
  if (((byte < 32U) && (byte != '\t')) || (byte > 126U) ||
      (line_length >= sizeof(line) - 1U))
  {
    discard_line = 1U;
    return;
  }
  line[line_length++] = (char)byte;
}

static void Console_RecoverReceive(void)
{
  /* Discard the damaged line rather than applying a truncated number. */
  HAL_NVIC_DisableIRQ(USART2_IRQn);
  (void)HAL_UART_AbortReceive(console_uart);
  __HAL_UART_CLEAR_OREFLAG(console_uart);
  rx_head = 0U;
  rx_tail = 0U;
  rx_fault = 0U;
  line_length = 0U;
  discard_line = 1U;
  if (HAL_UART_Receive_IT(console_uart, &rx_byte, 1U) != HAL_OK)
    rx_fault = 1U;
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  Console_Send("ERROR: UART receive overflow/error. Press Enter, then resend.\r\n");
}

static void Servo_Update(uint32_t now)
{
  if ((uint32_t)(now - last_step_ms) >= SERVO_STEP_MS)
  {
    uint32_t pulse = servo_command_us;
    last_step_ms = now;
    if (pulse < target_us)
    {
      uint32_t difference = target_us - pulse;
      pulse += (difference > SERVO_STEP_US) ? SERVO_STEP_US : difference;
    }
    else if (pulse > target_us)
    {
      uint32_t difference = pulse - target_us;
      pulse -= (difference > SERVO_STEP_US) ? SERVO_STEP_US : difference;
    }
    Servo_WritePulse(pulse);
  }

  if ((sweep_enabled != 0U) && (servo_command_us == target_us))
  {
    if (endpoint_waiting == 0U)
    {
      endpoint_ms = now;
      endpoint_waiting = 1U;
    }
    else if ((uint32_t)(now - endpoint_ms) >= SERVO_ENDPOINT_MS)
    {
      if (servo_target_angle == 180U)
      {
        Servo_SetAngle(0U);
        sweep_reached_high = 1U;
      }
      else
      {
        if (sweep_reached_high != 0U)
        {
          ++servo_sweep_count;
          sweep_reached_high = 0U;
        }
        Servo_SetAngle(180U);
      }
    }
  }
}

void ServoConsole_Init(TIM_HandleTypeDef *timer, UART_HandleTypeDef *uart)
{
  servo_timer = timer;
  console_uart = uart;
  target_us = SERVO_CENTER_US;
  servo_target_angle = 90U;
  servo_sweep_count = 0U;
  sweep_enabled = 0U;
  sweep_reached_high = 0U;
  endpoint_waiting = 0U;
  rx_head = 0U;
  rx_tail = 0U;
  rx_fault = 0U;
  line_length = 0U;
  discard_line = 0U;
  last_step_ms = HAL_GetTick();
  Servo_WritePulse(SERVO_CENTER_US);
  __HAL_TIM_SET_COUNTER(servo_timer, 0U);
  if (HAL_TIM_PWM_Start(servo_timer, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_UART_Receive_IT(console_uart, &rx_byte, 1U) != HAL_OK) Error_Handler();
  Console_Send("\r\nOT5320M USART2 ready: 115200 8N1. Target=90deg (1500us).\r\n"
               "Enter 0..180 + Enter; sweep; hold; help.\r\n");
}

void ServoConsole_Poll(void)
{
  /* Bound parsing work so pasted input cannot starve the servo update. */
  for (uint32_t count = 0U; count < 64U; ++count)
  {
    uint8_t byte;
    if (rx_fault != 0U)
    {
      Console_RecoverReceive();
      break;
    }
    if (rx_tail == rx_head) break;
    byte = rx_queue[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % RX_CAPACITY);
    Console_ProcessByte(byte);
  }
  Servo_Update(HAL_GetTick());
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart != console_uart) return;
  if (rx_fault == 0U)
  {
    uint16_t next = (uint16_t)((rx_head + 1U) % RX_CAPACITY);
    if (next == rx_tail) rx_fault = 1U;
    else
    {
      rx_queue[rx_head] = rx_byte;
      rx_head = next;
    }
  }
  if (HAL_UART_Receive_IT(uart, &rx_byte, 1U) != HAL_OK) rx_fault = 1U;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart == console_uart) rx_fault = 1U;
}
