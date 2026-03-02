/*
 * rc_car.h
 *
 *  Created on: Feb 23, 2026
 *      Author: jsh-laptop
 */

#ifndef INC_TRIPLE_SONIC_H_
#define INC_TRIPLE_SONIC_H_

#include "stm32f4xx_hal.h"
#include "delay.h"
#include "tim.h"


#define TOP 3

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t       pin;
}TRIG;


typedef struct
{
  volatile uint16_t IC_Value1;  // Rising check
  volatile uint16_t IC_Value2;   // Falling check
  volatile uint16_t echoTime;   // High Pulse Time, The time between IC_value1 and IC_value2

  volatile uint16_t captureFlag;   //
  volatile uint8_t  distance;   //
}Sonic;

typedef struct
{
  Sonic num[TOP];
}COM;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim11;

extern COM ult_sonic;

void ult_init(COM *ult_sonic);
void HCSCR04_TRG(COM *ult_sonic);
void machine(TIM_HandleTypeDef *htim, uint32_t channel, Sonic *sonic);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);


#endif /* INC_TRIPLE_SONIC_H_ */

















