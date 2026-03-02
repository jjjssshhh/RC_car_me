/*
 * rc_car.c
 *
 *  Created on: Feb 23, 2026
 *      Author: jsh-laptop
 */


#include <triple_sonic.h>


COM ult_sonic;


TRIG Trig[3] =
{
    {GPIOC,GPIO_PIN_0},
    {GPIOC,GPIO_PIN_1},
    {GPIOC,GPIO_PIN_2}
};


void ult_init(COM *ult_sonic)
{
  for(int i=0; i<TOP;i++)
  {
     ult_sonic->num[i].IC_Value1    = 0;  // Rising check
     ult_sonic->num[i].IC_Value2    = 0;   // Falling check
     ult_sonic->num[i].echoTime     = 0;   // High Pulse Time, The time between IC_value1 and IC_value2
     ult_sonic->num[i].captureFlag  = 0;   //
     ult_sonic->num[i].distance     = 0;   //
  }
}

// master가 10us의 high pulse전송 그리고 도착하는 신호 포착준비
void HCSCR04_TRG(COM *ult_sonic)
{
  for(int i=0; i<TOP;i++)
  {
    HAL_GPIO_WritePin(Trig[i].port, Trig[i].pin, GPIO_PIN_RESET);
    delay_us(1);
    HAL_GPIO_WritePin(Trig[i].port, Trig[i].pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(Trig[i].port, Trig[i].pin, GPIO_PIN_RESET);
    delay_us(1);

    //에코.
    if(i == 0) //PA6-왼쪽
    {
      __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_CC1);
    }
    else if(i==1)//PA7-중앙
    {
      __HAL_TIM_ENABLE_IT(&htim2,TIM_IT_CC1);
    }
    else if(i==2)//PB9 -오른쪽
    {
      __HAL_TIM_ENABLE_IT(&htim11,TIM_IT_CC1);
    }
    HAL_Delay(10); // 블로킹...
  }
}

//void machine(TIM_HandleTypeDef *htim, HAL_TIM_ActiveChannel channel, Sonic *sonic)
//{
//  if(sonic->captureFlag == 0)
//  {
//    sonic->IC_Value1 = HAL_TIM_ReadCapturedValue(htim, channel);
//
//    sonic->captureFlag = 1;
//    __HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_FALLING);
//  }
//  else if(sonic->captureFlag == 1)
//  {
//    sonic->IC_Value2 = HAL_TIM_ReadCapturedValue(htim, channel);
//    if(sonic->IC_Value2 > sonic->IC_Value1)
//    {
//      sonic->echoTime = sonic->IC_Value2 - sonic->IC_Value1;
//    }
//    else if(sonic->IC_Value1 > sonic->IC_Value2)
//    {
//      sonic->echoTime = 0xffff - sonic->IC_Value1 + sonic->IC_Value2;
//    }
//    sonic->distance = sonic->echoTime / 58;
//    __HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_RISING);
//
//    if(htim->Instance == TIM3  && channel == HAL_TIM_ACTIVE_CHANNEL_1)
//      __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
//    else if(htim->Instance == TIM2  && channel == HAL_TIM_ACTIVE_CHANNEL_1)
//      __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
//    else if(htim->Instance == TIM11 && channel == HAL_TIM_ACTIVE_CHANNEL_1)
//      __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
//
//    sonic->captureFlag = 0;
//  }
//}

void machine(TIM_HandleTypeDef *htim, uint32_t channel, Sonic *sonic)
{
  // HAL_TIM_ReadCapturedValue 등에 들어갈 채널 값 정의
  // 보통 채널 1은 TIM_CHANNEL_1 입니다.
  uint32_t tim_ch = TIM_CHANNEL_1;

  if(sonic->captureFlag == 0)
  {
    // 1. Rising Edge 값 읽기
    sonic->IC_Value1 = HAL_TIM_ReadCapturedValue(htim, tim_ch);
    sonic->captureFlag = 1;

    // 2. 극성을 Falling으로 변경 (이 부분이 잘못된 채널값으로 들어가면 작동안함)
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, tim_ch, TIM_INPUTCHANNELPOLARITY_FALLING);
  }
  else if(sonic->captureFlag == 1)
  {
    // 3. Falling Edge 값 읽기
    sonic->IC_Value2 = HAL_TIM_ReadCapturedValue(htim, tim_ch);

    // 4. 시간 계산 (TIM2는 32비트이므로 ARR에 맞춰 계산)
    if(sonic->IC_Value2 > sonic->IC_Value1)
    {
      sonic->echoTime = sonic->IC_Value2 - sonic->IC_Value1;
    }
    else
    {
      uint32_t max_arr = __HAL_TIM_GET_AUTORELOAD(htim);
      sonic->echoTime = max_arr - sonic->IC_Value1 + sonic->IC_Value2;
    }

    sonic->distance = sonic->echoTime / 58;

    // 5. 초기 상태로 복구
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, tim_ch, TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);

    sonic->captureFlag = 0;
  }
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM3  && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    machine(htim, HAL_TIM_ACTIVE_CHANNEL_1,&ult_sonic.num[0]);
//    machine(htim, HAL_TIM_ACTIVE_CHANNEL_1,&sonic_left);
  }
  else if(htim->Instance == TIM2  && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    machine(htim, HAL_TIM_ACTIVE_CHANNEL_1,&ult_sonic.num[1]);
//    machine(htim, HAL_TIM_ACTIVE_CHANNEL_1,&sonic_mid);
  }
  else if(htim->Instance == TIM11 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    machine(htim, HAL_TIM_ACTIVE_CHANNEL_1,&ult_sonic.num[2]);
//    machine(htim, HAL_TIM_ACTIVE_CHANNEL_1,&sonic_right);
  }
}














