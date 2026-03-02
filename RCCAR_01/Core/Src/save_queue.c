/*
 * save_queue.c
 *
 *  Created on: Feb 25, 2026
 *      Author: jsh-laptop
 */


#include "save_queue.h"
#include "stdlib.h"

Circular_Queue cq;
Dataset ds;

// 증가시키고 넣고, 뺴고 증가시키고
// 리어가 가리킬때 넣고, 프론트에서 뺀다. 선입선출

void init(Circular_Queue *cq)
{
  for(int i=0; i<SIZE;i++)
  {
    cq->data[i].flag    = 0;
    cq->data[i].dist_L  = 0;
    cq->data[i].dist_M  = 0;
    cq->data[i].dist_R  = 0;
    cq->data[i].CCR_L   = 0;
    cq->data[i].CCR_R   = 0;
    cq->data[i].interval = 0;
  }
  cq->front = 0;
  cq->rear = 0;
}

bool isempty(Circular_Queue *cq)
{
  if(cq->front == cq->rear)
  {
    return true;
  }
  return false;
}

// 꽉찰때는 0 ~ 999, 455,456과 같음
bool isfull(Circular_Queue *cq)
{
  if( (cq->rear+1)% SIZE == cq->front)
  {
    return true;
  }
  return false;
}



bool enqueue(Circular_Queue *cq, Dataset *ds)
{
  if(isfull(cq)) return false;
  //cq->data[cq->rear].distance = data;
  cq->data[cq->rear].flag  = ds->flag;
  cq->data[cq->rear].dist_L  = ds->dist_L;
  cq->data[cq->rear].dist_M  = ds->dist_M;
  cq->data[cq->rear].dist_R  = ds->dist_R;
  cq->data[cq->rear].CCR_L   = ds->CCR_L;
  cq->data[cq->rear].CCR_R   = ds->CCR_R;
  cq->data[cq->rear].interval = ds->interval;
  cq->rear = (cq->rear + 1) % SIZE;
  return true;
}
// 값을 바꾸려면 *를 넣어야 함
bool dequeue(Circular_Queue *cq, Dataset *ds)
{
  if(isempty(cq)) return false;
  //*data = cq->data[cq->front].distance;
  ds->flag = cq->data[cq->front].flag ;
  ds->dist_L = cq->data[cq->front].dist_L ;
  ds->dist_M = cq->data[cq->front].dist_M ;
  ds->dist_R = cq->data[cq->front].dist_R ;
  ds->CCR_L = cq->data[cq->front].CCR_L  ;
  ds->CCR_R = cq->data[cq->front].CCR_R  ;
  ds->interval = cq->data[cq->front].interval ;

  cq->front = (cq->front + 1) % SIZE;

  return true;
}

void printqueue(Circular_Queue *cq)
{
  if(isempty(cq)) return;

  int ptr = cq->front;
  while(ptr != cq->rear)
  {
    ptr = (ptr+1)%SIZE;
  }
}

// 샘플링, 초음파센서 안에 30ms의 딜레이가 있으므로 반드시 이것보다 짧은 샘플링은 불가함
void Sampling_Process(Circular_Queue *cq, Dataset *ds, uint8_t flag)
{
  // 과거ccr1이 현재값과 달라지면.
  static uint32_t prev;

  /**
   * 기록조건
   * 1. 코너링 하는 동안    30ms
   * 2. 타임아웃(직선구간) 300ms
   */
  //좌/우회전시 매틱마다 저장
  if(flag > 2)
  {
    ds->interval = HAL_GetTick() - prev; // 간격을 기록해서 저장 hal_delay(ds->interval)가능
    enqueue(cq, ds);
    prev = HAL_GetTick();
  }
  // 타임아웃(직선구간 저장)
  else if(HAL_GetTick() - prev > 300)
  {
    ds->interval = HAL_GetTick() - prev;
    enqueue(cq, ds);
    prev = HAL_GetTick();
  }
}


//기록값 뿌리기 - while문있음
void Dump_Queue_To_UART(Circular_Queue *cq)
{
    char buf[128];
    Dataset temp_ds;

    // 노트북(UART2)으로 먼저 시작 알림
    //HAL_UART_Transmit(&huart6, (uint8_t*)"--- LAPTOP LOG START ---\r\n", 26, 100);

    while (dequeue(cq, &temp_ds))
    {
//        int len = sprintf(buf, "%d,%d,%d,%d,%lu,%lu,%lu\r\n",temp_ds.flag,
//                          temp_ds.dist_L, temp_ds.dist_M, temp_ds.dist_R,
//                          temp_ds.CCR_L, temp_ds.CCR_R, temp_ds.interval);
      int len = sprintf(buf, "%d,%d,%d,%d,%d,%d,%d\r\n",temp_ds.flag,
                        temp_ds.dist_L, temp_ds.dist_M, temp_ds.dist_R,
                        temp_ds.CCR_L, temp_ds.CCR_R, temp_ds.interval);
        // 노트북(UART2)으로 쏜다! (여기서 파일 저장용 로그가 생성됨)
        HAL_UART_Transmit(&huart6, (uint8_t*)buf, len, 100);

        // (선택) 휴대폰(UART1)으로도 보고 싶다면 한 줄 더 추가
        // HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 100);
    }
}








