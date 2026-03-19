#include "main.h"                   // Device header
#include <stdio.h>
#include <stdarg.h>
#include "user_task.h"
#include "cmsis_os2.h" // 为了使用 osDelay
#include <string.h>    // 为了使用 strlen

char BlueSerial_RxPacket[100];
char BlueSerial_TxPacket[100];
uint8_t BlueSerial_RxFlag;

uint8_t BlueSerial_RxByte;
uint8_t Serial1_RxByte;

void BlueSerial_Init(void)
{
	/* 已在CubeMX中初始化USART2 */
	HAL_UART_Receive_IT(&huart2, &BlueSerial_RxByte, 1);
}

/**
 * @brief  通过 DMA 异步发送格式化字符串到蓝牙串口
 * @param  format: 格式化字符串，用法同 printf
 * @param  ...: 可变参数列表
 * @note   1. 该函数具有阻塞性：若上一次发送未完成，会通过 osDelay(1) 等待。
 * 2. 内部使用 vsprintf，请确保 BlueSerial_TxPacket 缓冲区足够大以防溢出。
 * @retval 无
 */
void BlueSerial_Printf_DMA(char *format, ...)
{
    // 等待上一次 DMA 发送完成，防止数据覆盖 (huart2.gState 代表 TX 状态)
    while (huart2.gState != HAL_UART_STATE_READY)
    {
        osDelay(1); // 释放 CPU 控制权给其他任务，防止死等阻塞
    }
    
    va_list arg;
    va_start(arg, format);
    vsprintf(BlueSerial_TxPacket, format, arg);
    va_end(arg);
    
    // 使用 DMA 发送数据
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BlueSerial_TxPacket, strlen(BlueSerial_TxPacket));
}

void BlueSerial_SendByte(uint8_t Byte)
{
	HAL_UART_Transmit(&huart2, &Byte, 1, HAL_MAX_DELAY);

}

void BlueSerial_SendArray(uint8_t *Array, uint16_t Length)
{
	HAL_UART_Transmit(&huart2, Array, Length, HAL_MAX_DELAY);
}

void BlueSerial_SendString(char *String)
{
	uint16_t len = 0;
	while (String[len] != '\0') len++;
	HAL_UART_Transmit(&huart2, (uint8_t *)String, len, HAL_MAX_DELAY);
}

uint32_t BlueSerial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void BlueSerial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		BlueSerial_SendByte(Number / BlueSerial_Pow(10, Length - i - 1) % 10 + '0');
	}
}



void BlueSerial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	BlueSerial_SendString(String);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		static uint8_t RxState = 0;
		static uint8_t pRxPacket = 0;
		uint8_t RxData = BlueSerial_RxByte;
		if (RxState == 0)
		{
			if (RxData == '[' && BlueSerial_RxFlag == 0)
			{
				RxState = 1;
				pRxPacket = 0;
			}
		}
		else if (RxState == 1)
		{
			if (RxData == ']')
			{
				RxState = 0;
				BlueSerial_RxPacket[pRxPacket] = '\0';
				BlueSerial_RxFlag = 1;
			}
			else
			{
				BlueSerial_RxPacket[pRxPacket] = RxData;
				pRxPacket ++;
			}
		}
		HAL_UART_Receive_IT(&huart2, &BlueSerial_RxByte, 1);
	}
	else if (huart->Instance == USART1)
	{
		Serial_RxData = Serial1_RxByte;
		Serial_RxFlag = 1;
		HAL_UART_Receive_IT(&huart1, &Serial1_RxByte, 1);
		
	}
}
