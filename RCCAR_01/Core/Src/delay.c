/*
 * delay.c
 *
 *  Created on: Jan 27, 2026
 *      Author: jsh-laptop
 */


#include "delay.h"
extern TIM_HandleTypeDef htim10;

// 타이머의 번호를 확인할 것
void delay_us(uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim10, 0);
  while((__HAL_TIM_GET_COUNTER(&htim10)) < us);
}
