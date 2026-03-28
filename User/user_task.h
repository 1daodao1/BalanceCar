#ifndef _USER_TASK_H_
#define _USER_TASK_H_

#include "stdint.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

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

/* 任务句柄，用于获取任务信息 */
extern TaskHandle_t BalanceTask_Handler;
extern TaskHandle_t SpeedTask_Handler;
extern TaskHandle_t CommsTask_Handler;
extern TaskHandle_t UITask_Handler;

void vTask_Balance(void);
void vTask_Speed(void);
void vTask_Comms(void);
void vTask_UI(void);
void StartTimerForRunTimeStats(void);
uint32_t GetTimerForRunTimeStats(void);

#endif
