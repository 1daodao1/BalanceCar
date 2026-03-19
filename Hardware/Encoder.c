
#include "main.h"                  // Device header
#include "user_task.h"

/**
  * 函    数：编码器初始化
  * 参    数：无
  * 返 回 值：无
  */
void Encoder_Init(void)
{
	/* 已在CubeMX中初始化TIM3和TIM4 */
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

/**
  * 函    数：获取编码器的增量值
  * 参    数：无
  * 返 回 值：自上此调用此函数后，编码器的增量值
  */
int16_t Encoder_Get(uint8_t n)
{
	/*使用Temp变量作为中继，目的是返回CNT后将其清零*/
	int16_t Temp;
	if(n == 1)
	{
		Temp = __HAL_TIM_GET_COUNTER(&htim3);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		return Temp;
	}
	
	else if(n == 2)
	{
		Temp = __HAL_TIM_GET_COUNTER(&htim4);
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		return Temp;
	}
	return 0;
	
}
