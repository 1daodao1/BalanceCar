/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "user_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Balance */
osThreadId_t BalanceHandle;
const osThreadAttr_t Balance_attributes = {
  .name = "Balance",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Speed */
osThreadId_t SpeedHandle;
const osThreadAttr_t Speed_attributes = {
  .name = "Speed",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Comms */
osThreadId_t CommsHandle;
const osThreadAttr_t Comms_attributes = {
  .name = "Comms",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UI */
osThreadId_t UIHandle;
const osThreadAttr_t UI_attributes = {
  .name = "UI",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ButtonQueue */
osMessageQueueId_t ButtonQueueHandle;
const osMessageQueueAttr_t ButtonQueue_attributes = {
  .name = "ButtonQueue"
};
/* Definitions for TimerQueue */
osMessageQueueId_t TimerQueueHandle;
const osMessageQueueAttr_t TimerQueue_attributes = {
  .name = "TimerQueue"
};
/* Definitions for PrintQueue */
osMessageQueueId_t PrintQueueHandle;
const osMessageQueueAttr_t PrintQueue_attributes = {
  .name = "PrintQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void vBalance(void *argument);
void vSpeed(void *argument);
void vComms(void *argument);
void vUI(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ButtonQueue */
  ButtonQueueHandle = osMessageQueueNew (1, sizeof(uint8_t), &ButtonQueue_attributes);

  /* creation of TimerQueue */
  TimerQueueHandle = osMessageQueueNew (1, sizeof(uint16_t), &TimerQueue_attributes);

  /* creation of PrintQueue */
  PrintQueueHandle = osMessageQueueNew (1, sizeof(uint8_t *), &PrintQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Balance */
  BalanceHandle = osThreadNew(vBalance, NULL, &Balance_attributes);

  /* creation of Speed */
  SpeedHandle = osThreadNew(vSpeed, NULL, &Speed_attributes);

  /* creation of Comms */
  CommsHandle = osThreadNew(vComms, NULL, &Comms_attributes);

  /* creation of UI */
  UIHandle = osThreadNew(vUI, NULL, &UI_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	BalanceTask_Handler = (TaskHandle_t)BalanceHandle;
  SpeedTask_Handler = (TaskHandle_t)SpeedHandle;
  CommsTask_Handler = (TaskHandle_t)CommsHandle;
  UITask_Handler = (TaskHandle_t)UIHandle;
	
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_vBalance */
/**
  * @brief  Function implementing the Balance thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_vBalance */
void vBalance(void *argument)
{
  /* USER CODE BEGIN vBalance */
	
  /* Infinite loop */
  for(;;)
  {
    vTask_Balance();
  }
  /* USER CODE END vBalance */
}

/* USER CODE BEGIN Header_vSpeed */
/**
* @brief Function implementing the Speed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vSpeed */
void vSpeed(void *argument)
{
  /* USER CODE BEGIN vSpeed */
	
  /* Infinite loop */
  for(;;)
  {
    vTask_Speed();
  }
  /* USER CODE END vSpeed */
}

/* USER CODE BEGIN Header_vComms */
/**
* @brief Function implementing the Comms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vComms */
void vComms(void *argument)
{
  /* USER CODE BEGIN vComms */
  /* Infinite loop */
  for(;;)
  {
    vTask_Comms();
  }
  /* USER CODE END vComms */
}

/* USER CODE BEGIN Header_vUI */
/**
* @brief Function implementing the UI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vUI */
void vUI(void *argument)
{
  /* USER CODE BEGIN vUI */
  /* Infinite loop */
  for(;;)
  {
    vTask_UI();
  }
  /* USER CODE END vUI */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

