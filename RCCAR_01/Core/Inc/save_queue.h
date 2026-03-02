/*
 * save_queue.h
 *
 *  Created on: Feb 25, 2026
 *      Author: jsh-laptop
 */

#ifndef INC_SAVE_QUEUE_H_
#define INC_SAVE_QUEUE_H_

#include "stm32f4xx_hal.h"
#include "stdbool.h"

#define SIZE 401 // 순환큐는 1개를 쓰지 않는다.
//#define SIZE 41 // 순환큐는 1개를 쓰지 않는다.

typedef struct
{
  uint8_t flag;
  uint8_t dist_L;
  uint8_t dist_M;
  uint8_t dist_R;
  uint32_t CCR_L;
  uint32_t CCR_R;
  uint16_t interval;
} sample_data;

typedef struct
{
  sample_data data[SIZE];
  int front;
  int rear;
} Circular_Queue;

typedef struct
{
  uint8_t  flag;
  uint8_t  dist_L;
  uint8_t  dist_M;
//  uint8_t  dist_R;
//  uint32_t CCR_L;
//  uint32_t CCR_R;
  int dist_R;
  int CCR_L;
  int CCR_R;
  uint16_t interval;
} Dataset;

extern Circular_Queue cq;
extern Dataset ds;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

void init(Circular_Queue *cq);
bool isempty(Circular_Queue *cq);
bool isfull(Circular_Queue *cq);
bool enqueue(Circular_Queue *cq, Dataset *ds);
bool dequeue(Circular_Queue *cq, Dataset *ds);
void Sampling_Process(Circular_Queue *cq, Dataset *ds, uint8_t flag);
void printqueue(Circular_Queue *cq);
void Dump_Queue_To_UART(Circular_Queue *cq);
#endif /* INC_SAVE_QUEUE_H_ */








