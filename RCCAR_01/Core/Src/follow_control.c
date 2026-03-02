/*
 * fllow_control.c
 *
 *  Created on: Feb 26, 2026
 *      Author: jsh-laptop
 */


#include "follow_control.h"

#define sonic_max 255

static int32_t dist_L = 0;
static int32_t dist_M = 0;
static int32_t dist_R = 0;
uint32_t idx = 0;

static uint32_t res_f  = 0;
static uint32_t res_v1 = 0;
static uint32_t res_v2 = 0;

uint8_t is_running = 0;


/*
 * 다시 생각해보자..
 * 이게 맞는걸까..
 */
void follow_control(DriveStep *track_data,COM *ult_sonic)
{
//  if(is_running)
//  {
//    // 초 단순 추종제어
//    // 초음파데이터
//    CCR_Control(track_data[idx].flag, track_data[idx].v_l, track_data[idx].v_r);
//    HAL_Delay(track_data[idx].interval);
//    idx++;
//  }
//  else CCR_Control(0,0,0);

  uint16_t alpha = 10;
  uint8_t k = 2;
  static uint32_t prev = 0;
  int threshhold = 5;

  if(prev == 0) prev = HAL_GetTick();

  if(is_running)
  {
    HCSCR04_TRG(ult_sonic); // hal_delay 30
    // y[n] = y[n-1] + 1/8(x[n] - y[n-1])
    // 7/8*y[n] + 1/8*x[n]
    dist_L += ((int32_t)ult_sonic->num[0].distance - dist_L) >> 3;
    dist_M += ((int32_t)ult_sonic->num[1].distance - dist_M) >> 3;
    dist_R += ((int32_t)ult_sonic->num[2].distance - dist_R) >> 3;

    // 초음파 값에 의해 가중치 제어
    alpha = (abs(dist_L - track_data[idx].sonic_L)
            + abs(dist_M - track_data[idx].sonic_M)
            + abs(dist_R - track_data[idx].sonic_R))/3*k; //예민도

    if(alpha > sonic_max) alpha = sonic_max;

    // 직진인 상태에서 차로유지(왼/오 값이 가까워지면 멀어지게)
    res_f = track_data[idx].flag;

    if(track_data[idx].flag == 0)
    {
      if(abs(dist_L - track_data[idx].sonic_L) > threshhold)
      {
        res_f = 3;
      }
      else if(abs(dist_R - track_data[idx].sonic_R) > threshhold)
      {
        res_f = 2;
      }
      else res_f  = 0;
    }
    // 그러다가 현재 소닉값이 너무 작으면 긴급으로 정지,후진하던지 해야 할듯!!!!!== 중요
    if(dist_L < 3 || dist_M < 3 || dist_R < 3)
    {
      res_v1 = 0;
      res_v2 = 0;
    }

    // 가중치 속도 제어, 현재 초음파와 과거 데이터 셋과 비교하여 알파 수정
    res_v1 = (alpha * TIM1->CCR1 + (sonic_max-alpha) * track_data[idx].v_l)/sonic_max;
    res_v2 = (alpha * TIM1->CCR2 + (sonic_max-alpha) * track_data[idx].v_r)/sonic_max;


    CCR_Control(res_f, res_v1, res_v2);
    //어차피 33ms기준동작이므로
    if(HAL_GetTick() - prev > track_data[idx].interval)
    {
      //idx가 최대에 도달하면 idx초기화
      idx++;
      if(idx >= TRACK_DATA_SIZE)
      {
        idx = 0;
      }
      prev = HAL_GetTick();
    }
  }
  else // 멈춤상태
  {
    CCR_Control(0, 0, 0);
    prev = HAL_GetTick();
  }
}
















