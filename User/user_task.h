#ifndef _USER_TASK_H_
#define _USER_TASK_H_

#include "stdint.h"
#include "main.h"

/* 全局外设句柄声明 */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* 全局通信变量声明 */
extern uint8_t Serial_RxData;
extern uint8_t Serial_RxFlag;
extern uint8_t Serial1_RxByte;
extern uint8_t BlueSerial_RxByte;

void vTask_Balance(void);
void vTask_Speed(void);
void vTask_Comms(void);
void vTask_UI(void);

#endif
