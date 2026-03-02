
/*
 * rcmotor.c
 *
 *  Created on: Feb 24, 2026
 *      Author: jsh-laptop
 */

//왼 3000, 오 3100
#include "rcmotor.h"

// FRONT는 40보다 작으면 멈출수있고 크면 좌/우회전으로 멈칫멈칫한다.
// SIDE는 10근처가 적당하다.
// 마지막꺼
//#define FRONT 40
//#define SIDE  16
//#define VV1   9000
//#define VV2   9000
//#define VV3   9000
//#define VV4   7200
//#define VV5   7200
//#define VV6   9000
//#define TIC   1

//// PID_MOVE사용해도 주석처리하면 안됨
#define FRONT 40 //필요없지만 다른함수때문에 선언은 해놀것
#define SIDE  16
#define TIC   3
#define VV1   4500 //4500
#define VV2   4500 //4500
#define VV3   6500 //6500
#define VV4   4500 //5000
#define VV5   4500 //5000
#define VV6   6500 //6500

// PID용
float PVV1     = 4000; //4000 Kv = 30 제지리 게수 0.5
float PVV2     = 4000; //4000

uint8_t pData;
uint8_t flag;
uint8_t ac;
uint8_t sel;
uint8_t send = 0;

static uint8_t running;
// 차의 휠폭 14cm, pi*r1, pi*(r1+14) [cm]
static uint32_t v1 = VV1;
static uint32_t v2 = VV2;
static uint32_t v3 = VV3;
static uint32_t v4 = VV4;
static uint32_t v5 = VV5;
static uint32_t v6 = VV6;

static int32_t sample1 = 0;
static int32_t sample2 = 0;
static int32_t sample3 = 0;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // 조이스틱6, 블루투스1
  if(huart->Instance == USART6)
  {
    //HAL_UART_Transmit(&huart1, &pData, 1, 100); //echo
    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
     //주행기록모드용 명령어셋
    if(pData != '\r' && pData != '\n')
    {
      if(pData == '1')
      {
        flag = 0;
      }
      if(pData == 'w')
      {
        flag = 1;
      }
      if(pData == 's')
      {
        flag = 2;
      }
      if(pData == 'a')
      {
        flag = 3;
      }
      if(pData == 'd')
      {
        flag = 4;
      }

      if(pData == 'p') //가속
      {
        ac = 1;
      }
      else if(pData == 'l')
      {
        ac = 0;
      }

      if(pData == 'b')
      {
        sel = 1;
      }
      else if(pData == 'c')
      {
        sel = 0;
      }
      if(pData == 't')
      {
        send = 1;
      }

      // 추종제어모드용 명령셋
      if(pData == 'e')
      {
        is_running = 1;
        running = 1;
      }
      else if(pData == 'q')
      {
        is_running = 0;
        running = 0;
      }
    }
    HAL_UART_Receive_IT(&huart6, &pData, 1);
  }
}
// PSC = 0 ARR = 1000
// 10us마다 인터럽트 발생 , 최대가속까지 10*1500us 15ms?
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//  if (htim->Instance == TIM9)
//  {
//    // 가속 신호(ac == 1)가 들어왔을 때 목표 속도까지 서서히 증가
//    if (ac == 1)
//    {
//      if(flag == 1 || flag == 2)
//      {
//        if (v1 < 5000) v1 += 1; // 10씩 증가 (상수값은 테스트하며 조절)
//        if (v2 < 5000) v2 += 1;
//      }
//      else if(flag == 3)
//      {
//        if( v3 < 5500) v3 += 1;
//      }
//      else if(flag == 4)
//      {
//        if( v6 < 5500) v6 += 1;
//      }
//    }
//    // 가속 신호가 없으면 기본 속도로 감속 또는 유지
//    else if(ac == 0)
//    {
//      if (v1 > 3500) v1 -= 50;
//      if (v2 > 3500) v2 -= 50;
//      if( v3 > 4000) v3 -= 50;  // 초음파센서의 값이 작아지면 감속크기 매우크게 또는 급정지 필요
//      if( v6 > 4000) v6 -= 50;
//    }
//    // 현재 동작 중인 모드(flag)에 따라 실시간으로 PWM 반영
//  }
}

uint8_t auto_flag = 0;
uint32_t diff1; //초음파이므로 충분한 용량임
uint32_t diff2;
uint32_t diff3;
int delta_1;
int delta_2;
int delta_3;


// 속도을 정밀하게 제어
/*
 * R = 비례, L : 크면 전류변화량이 커짐 C : 크면
 * Ri + Li' + Cidt = V
 * Ri + L(i-io)/(t-to) + C(Q-Qo)*(t-t0) = V
 *
 * Kv * v + Kd * v' + Ki * vdt = Vo
 * Kv * v + Kd * (v-vo)/(t-to) + Ki * (S-So)*(t-to) = Vo
 *
 * 거리S가 중요한데. 중앙 초음파센서의 거리로 하면 적절한가? - 아니면 초음파 3개거리의 평균값으로?
 * 아니다 속도제어는 2개 모터니까. 1개로하고 보정치로 하면 될듯.
 *
 */
static float pid_out;
static uint8_t pid_flag;
static int32_t sp1;
static int32_t sp2;
static int32_t sp3;
static int32_t pre_sp1;
static int32_t pre_sp3;
static int32_t Va;
static int32_t Vb;
static int32_t Vc;

static float Error;
static float sum_error;
static float pid_pre_s;
//static int speed_out;

static uint32_t prev;


// P제어 15초 잘나온게 계수가 30이었을 것임 다시 만들수 있으려나도 모르겠음
//PID_MOVE용
uint32_t Kv = 60; // 60(30), 바꾸면 벽에 박음
#define Kd 500    // 500 * delta(error) * 0.033 = 16.5, 바꾸면 코너탈출시 기우뚱?
#define Ki 0.03    // 0.03 , 올렸더니 덜덜거림
// 사실상 Kp = 60, Kd = 15, Ki = 1임
// 높일수록 확돌릴수 있는 범위 증가
#define PID_INTEVAL 3000 // 3000 Kv = 30 /


/*
 * 취업용으로 진화시키는 방법
 * 1. curr을 없애고 상수화한다.
 * 2. 직선구간에서 불필요한 움직임을 제한하기 위해 Error < n인 구간에서는 속도가 바뀌지 않게 한다.
 * 3. Error와 pid_out의 그래프를 그린다.
 * 4. 각 이득이 이 값인 이유를 설명할 줄 알아야 한다.
 * - 배터리 풀충에서 작동한다..
 */
void PID_move(COM *ult_sonic)
{
  uint32_t curr;

  if(running)
  {
    pid_pre_s = Error;  //이전 값
    pre_sp1 = sp1;      //직전 시야로 코스주행 안정적 판단
    pre_sp3 = sp3;
    HCSCR04_TRG(ult_sonic);

    sp1 = ult_sonic->num[0].distance;
    sp2 = ult_sonic->num[1].distance;
    sp3 = ult_sonic->num[2].distance;

    curr = HAL_GetTick() - prev;
    //if(curr == 0) curr = 0.03; //방어 코드. 잘작동할지는 모름

    Error = sp1 - sp3;

    if(sp2 < 40 && sp2 > 15) // 동작하는거 확인 근데 좌회전은 되는데 우회전이 안됨
    {
      //Kv = 60; // 60으로 잘되는듯? 30이 아니고?
      pid_flag = 0; //갱신
    }
    else if(sp2 <= 15) // 15
    {
      //Kv = 1000;
      pid_flag = 1;
      if(Error>0) // 좌회전 - 너무 가까우면 제자리회전으로 변경
      {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
      }
      else
      {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
      }
    }
    else
    {
      //Kv = 30;
      pid_flag = 0; //갱신
      //직진세팅
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0); //좌
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0); //우
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
    }

    sum_error += Error; //오차 누적은 에러가 계산된 후에 실행!!!-제미나이

    if(sum_error > 60) sum_error = 60; // 길의 최대 폭.
    else if(sum_error < -60) sum_error = -60;
    pid_out = Kv * Error + Kd * (Error - pid_pre_s)/curr + Ki * (float)sum_error*curr;
    if      (pid_out > PID_INTEVAL)  pid_out =  PID_INTEVAL;
    else if (pid_out < -PID_INTEVAL) pid_out = -PID_INTEVAL;
    Va = PVV1 + pid_out; // 디버깅 + 대입용
    Vb = PVV2 - pid_out;
    Vc = PVV2 + pid_out;

    if(pid_flag) //
    {
      //if(PVV2 + pid_out < 3100) TIM1->CCR2 = 3100;
      //if(PVV1 + pid_out < 3100) TIM1->CCR1 = 3100;

      TIM1->CCR2 = (uint32_t) Va*0.5; // 왼 0.5 - 너무 확돌아서 뒤가털리면 배터리를 앞으로 가게하거나 값을 더 줄인다.
      TIM1->CCR1 = (uint32_t) Vc*0.5; // 오
    }
    else // 왼쪽의 거리가 크면 오른쪽에 힘을 더준다.
    {
      //if(PVV2 - pid_out < 3100) TIM1->CCR2 = 3100;
      //if(PVV1 + pid_out < 3100) TIM1->CCR1 = 3100;

      TIM1->CCR2 = (uint32_t) Vb; // 왼
      TIM1->CCR1 = (uint32_t) Va; // 오
    }

    prev = HAL_GetTick();
  }
  else
  {
    //직진세팅
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0); //좌
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0); //우
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
    //초기화
    sum_error   = 0;
    TIM1->CCR1  = 0;
    TIM1->CCR2  = 0;
    //pid_pre_s   = 0;
    //pid_flag    = 0;
  }
}


// x[n] 왼쪽 193, 중앙 130, 오른쪽 86 이 값들 이상이면 신뢰불가     / 입력값
// y[n] 왼쪽 187, 중앙 160, 오른쪽 79(86) 이 값들 이상이면 신뢰불가 / 필터링값

void auto_move(COM *ult_sonic)
{
  if(is_running)
  {
    HCSCR04_TRG(ult_sonic);
    sample1 = ult_sonic->num[0].distance; // 왼쪽
    sample2 = ult_sonic->num[1].distance; // 정면
    sample3 = ult_sonic->num[2].distance; // 오른쪽

    // 중앙유지 전진
    if(sample2 <= FRONT)
    {
      if(sample1 > sample3) //좌회전
      {
        //CCR_Control(3, 4200, 4200);
        if(auto_flag != 1)
        {
          v3 = VV3;
          v4 = VV4;
        }
        auto_flag = 1;
        if(sample1 > diff1)
        {
          v3 += TIC;
          v4 -= TIC;
        }
        else if(sample1 < diff1)
        {
          v3 -= TIC;
          v4 += TIC;
        }
        CCR_Control(3,v3,v3);
        //ACC_Control(3, v3, v4);
      }
      else    // 우회전
      {
        //CCR_Control(4, 4200, 4200);
        if(auto_flag != 2)
        {
          v5 = VV5;
          v6 = VV6;
        }
        auto_flag = 2;
        if(sample3 > diff3)
        {
          v5 += TIC;
          v6 -= TIC;
        }
        else if(sample3 < diff3)
        {
          v5 -= TIC;
          v6 += TIC;
        }
        CCR_Control(4,v6,v6);
        //ACC_Control(4, v5, v6);
      }
    }

    // 너무 가까워지면 회전
    else if(sample1 < SIDE) // 왼쪽 벽 근접
    {
      if(auto_flag != 3)
      {
        v5 = VV5;
        v6 = VV6;
      }
      auto_flag = 3;
      if(sample3 > diff3)
      {
        v5 += TIC;
        v6 -= TIC;
      }
      else if(sample3 < diff3)
      {
        v5 -= TIC;
        v6 += TIC;
      }
      CCR_Control(4,v6,v6);
      //ACC_Control(4, v5, v6);
    }
    else if(sample3 < SIDE) // 오른쪽 벽 근접
    {
      if(auto_flag != 4)
      {
        v3 = VV3;
        v4 = VV4;
      }
      auto_flag = 4;
      if(sample1 > diff1)
      {
        v3 += TIC;
        v4 -= TIC;
      }
      else if(sample1 < diff1)
      {
        v3 -= TIC;
        v4 += TIC;
      }
      CCR_Control(3,v3,v3);
      //ACC_Control(3, v3, v4);
    }
    // 전진의 가/감속
    else
    {
      if(auto_flag != 5)
      {
        v1 = VV1;
        v2 = VV2;
      }
      auto_flag = 5;
      if(sample2 > 60)
      {
        v1 += TIC;
        if(v1 > 8000) v1 = 8000;
      }
      else
      {
        v1 -= 10;
        if(v1 < 3000) v1 = 3000;
      }
      ACC_Control(1, v1, v1);
    }
    diff1 = sample1;
    diff2 = sample2;
    diff3 = sample3;
  }
  else
  {
    ACC_Control(0, 0, 0); // 정지
  }
}




void Only_recdord(COM *ult_sonic)
{
  HCSCR04_TRG(ult_sonic); // hal_delay 30 - 전체 인터벌은 33ms임
//  // 계수는 1/8임
//  sample1 += ((int32_t)ult_sonic->num[0].distance - sample1) >> 3;
//  sample2 += ((int32_t)ult_sonic->num[1].distance - sample2) >> 3;
//  sample3 += ((int32_t)ult_sonic->num[2].distance - sample3) >> 3;
  sample1 = ult_sonic->num[0].distance;
  sample2 = ult_sonic->num[1].distance;
  sample3 = ult_sonic->num[2].distance;
  ds.flag   = sample1;
  ds.dist_L = sample2;
  ds.dist_M = sample3;
  ds.dist_R = delta_1;
  ds.CCR_L  = delta_2;
  ds.CCR_R  = delta_3;

  delta_1 = sample1 - diff1;
  delta_2 = sample2 - diff2;
  delta_3 = sample3 - diff3;

  if(is_running)
  {
    Sampling_Process(&cq, &ds, flag);

    Dump_Queue_To_UART(&cq);
  }
  else;
  diff1 = sample1;
  diff2 = sample2;
  diff3 = sample3;
}

void Save_driving_recdord(COM *ult_sonic)
{
  HCSCR04_TRG(ult_sonic); // hal_delay 30 - 전체 인터벌은 33ms임
  // 계수는 1/8임
  sample1 += ((int32_t)ult_sonic->num[0].distance - sample1) >> 3;
  sample2 += ((int32_t)ult_sonic->num[1].distance - sample2) >> 3;
  sample3 += ((int32_t)ult_sonic->num[2].distance - sample3) >> 3;
  ds.flag   = flag;
  ds.dist_L = sample1;
  ds.dist_M = sample2;
  ds.dist_R = sample3;
  ds.CCR_L  = TIM1->CCR1;
  ds.CCR_R  = TIM1->CCR2;

  Sampling_Process(&cq, &ds, flag);

  if(send == 1)
  {
    send = 0;
    CCR_Control(0, 0, 0);
    Dump_Queue_To_UART(&cq);
  }

    //블루투스 모드
  if(sel == 0)
  {
    if(flag == 0) // 정지
    {
      CCR_Control(flag, 0, 0);
    }
    else if(flag == 1) // 전진
    {
      CCR_Control(flag, v1, v2);
    }
    else if(flag == 2) // 후진
    {
      CCR_Control(flag, v1, v2);
    }
    else if(flag == 3) // 좌회전
    {
      CCR_Control(flag, v3, v4);
    }
    else if(flag == 4) // 우회전
    {
      CCR_Control(flag, v5, v6);
    }
  }
  else if(sel == 1)
  {
    if(flag == 0) // 정지
    {
      CCR_Control(0, 0, 0);
    }
    if(flag == 1) // 전진
    {
      if(sample2 > 30 && sample1 > 30 && sample3 > 30)
      {
        CCR_Control(flag, v1, v2);
      }
      else CCR_Control(0, 0, 0);
    }
    else if(flag == 2) // 후진
    {
      CCR_Control(flag, v1, v2);
    }
    else if(flag == 3) // 좌회전
    {
      if(sample2 > 30 && sample1 > 30 && sample3 > 30)
      {
        CCR_Control(flag, v3, v4);
      }
      else CCR_Control(0, 0, 0);
    }
    else if(flag == 4) // 우회전
    {
      if(sample2 > 30 && sample1 > 30 && sample3 > 30)
      {
        CCR_Control(flag, v5, v6);
      }
      else CCR_Control(3, 0, 0);
    }
  }
}


void PWM_Control(uint8_t direction, uint32_t speed)
{
  // 1. 속도 제어 (CCR 레지스터)
  TIM1->CCR1 = speed; // 왼쪽 (PA8)
  TIM1->CCR2 = speed; // 오른쪽 (PA9)

  switch(direction)
  {
    case 1: // 전진
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
      break;

    case 2: // 후진
      // 왼쪽 모터 (PC8, PC6)
      // 오른쪽 모터 (PC5, PC9)
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
      break;

    case 3: //좌회전
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
      break;

    case 4://우회전
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
      break;

    default: // 정지
      TIM1->CCR1 = 0;
      TIM1->CCR2 = 0;
      break;
  }
}
// 시간, 변수가 추가되어야 함.
// 이 스피드 값이 루프를 돌면 돌수록 증가해야 함
// 단 초음파 센서의 우선도를 더 높게 해야 함.
// 잠깐 플래그 다 1씩 더함
void CCR_Control(uint8_t direction, uint32_t speed1, uint32_t speed2)
{

  switch(direction)
  {
    case 1: // 전진
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
      TIM1->CCR1 = speed1; // 왼쪽 (PA8)
      TIM1->CCR2 = speed2; // 오른쪽 (PA9)
      break;

    case 2: // 후진
      // 왼쪽 모터 (PC8, PC6)
      // 오른쪽 모터 (PC5, PC9)
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
      TIM1->CCR1 = speed1; // 왼쪽 (PA8)
      TIM1->CCR2 = speed2; // 오른쪽 (PA9)
      break;

    case 3: //좌회전
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
      TIM1->CCR1 = speed1; // 왼쪽 (PA8)
      TIM1->CCR2 = speed2; // 오른쪽 (PA9)
      break;

    case 4: //우회전
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
      TIM1->CCR1 = speed1; // 왼쪽 (PA8)
      TIM1->CCR2 = speed2; // 오른쪽 (PA9)
      break;

    default: // 정지
      TIM1->CCR1 = 0;
      TIM1->CCR2 = 0;
      break;
  }
}


void ACC_Control(uint8_t direction, uint32_t speed1, uint32_t speed2)
{

  switch(direction)
  {
    case 1: // 전진
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
      TIM1->CCR1 = speed1; // 왼쪽 (PA8)
      TIM1->CCR2 = speed2; // 오른쪽 (PA9)
      break;

    case 2: // 후진
      // 왼쪽 모터 (PC8, PC6)
      // 오른쪽 모터 (PC5, PC9)
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
      TIM1->CCR1 = speed1; // 왼쪽 (PA8)
      TIM1->CCR2 = speed2; // 오른쪽 (PA9)
      break;

    case 3: //좌회전
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
      TIM1->CCR1 = speed1; // 왼쪽 (PA8)
      TIM1->CCR2 = speed2; // 오른쪽 (PA9)
      break;

    case 4: //우회전
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
      TIM1->CCR1 = speed1; // 왼쪽 (PA8)
      TIM1->CCR2 = speed2; // 오른쪽 (PA9)
      break;

    default: // 정지
      TIM1->CCR1 = 0;
      TIM1->CCR2 = 0;
      break;
  }
}


