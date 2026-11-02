#ifndef SERVO_CONSOLE_H
#define SERVO_CONSOLE_H

#include "stm32f4xx_hal.h"

void ServoConsole_Init(TIM_HandleTypeDef *timer, UART_HandleTypeDef *uart);
void ServoConsole_Poll(void);

/* Command estimates only; the servo provides no measured shaft feedback. */
extern volatile uint32_t servo_command_us;
extern volatile uint32_t servo_target_angle;
extern volatile uint32_t servo_sweep_count;

#endif
