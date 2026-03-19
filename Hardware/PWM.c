#include "main.h"                  // Device header
#include "user_task.h"                // Device header

/**
  * 函    数：PWM初始化
  * 参    数：无
  * 返 回 值：无
  */
void PWM_Init(void)
{
		/* 已在CubeMX中初始化TIM2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	
}

/**
  * 函    数：PWM设置CCR
  * 参    数：Compare 要写入的CCR的值，范围：0~100
  * 返 回 值：无
  * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  */
void PWM_SetCompare1(uint16_t Compare)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Compare);		//设置CCR1的值
}

void PWM_SetCompare2(uint16_t Compare)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Compare);		//设置CCR2的值
}

