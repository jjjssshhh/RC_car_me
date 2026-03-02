/*
 * rcmotor.h
 *
 *  Created on: Feb 24, 2026
 *      Author: jsh-laptop
 */

#ifndef INC_RCMOTOR_H_
#define INC_RCMOTOR_H_


#include <triple_sonic.h>
#include "stm32f4xx_hal.h"
#include "save_queue.h" //여기에 cq,ds선언
#include "stdlib.h"

extern Circular_Queue cq;
extern Dataset ds;
extern UART_HandleTypeDef huart6;

extern uint8_t is_running; // 추종제어 콜백용

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void PID_move(COM *ult_sonic);

void Only_recdord(COM *ult_sonic);
void AUTO_MOVE(COM *ult_sonic);
void PWM_Control(uint8_t direction, uint32_t speed);
void CCR_Control(uint8_t direction, uint32_t speed1, uint32_t speed2);
void ACC_Control(uint8_t direction, uint32_t speed1, uint32_t speed2);
#endif /* INC_RCMOTOR_H_ */
