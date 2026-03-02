/*
 * follow_control.h
 *
 *  Created on: Feb 26, 2026
 *      Author: jsh-laptop
 */

#ifndef INC_FOLLOW_CONTROL_H_
#define INC_FOLLOW_CONTROL_H_


#include "stm32f4xx_hal.h"
#include "autodata.h"
#include "triple_sonic.h"
#include "rcmotor.h"
#include "stdlib.h"   //abs

extern const DriveStep track_data[];

extern uint8_t is_running; //콜백플래그용

#endif /* INC_FOLLOW_CONTROL_H_ */
