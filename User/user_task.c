#include "stm32f1xx_hal.h"                  // Device header
#include "stdint.h"
#include "stdio.h"
#include "freertos.h"
#include "task.h"

//使用xQueueSend()需要包含的头文件
#include "cmsis_os2.h"
#include "queue.h"

//由于我全局变量都申明在此文件下，便于管理
#include "user_task.h"

//将所有部件的接口引入
#include "OLED.h"
#include "LED.h"
#include "Key.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "BlueSerial.h"
#include "PID.h"

#include <math.h>//数学运算库，注意是否改变硬件 FPU 寄存器
#include <string.h>//字符串与内存操作库，注意使用strtok_r()来替代strtok()

#include <stdlib.h>//标准通用工具库
//pvPortMalloc() 代替 malloc()，用 vPortFree() 代替 free()。它们内部有临界区保护，是绝对线程安全的。

//使用信号量需要包含的头文件
#include "semphr.h"

/*MPU6050测试*/
int16_t AX, AY, AZ, GX, GY, GZ;
uint8_t TimerErrorFlag;
int16_t TimerCount;

float AngleAcc;//存放加速度
float AngleGyro;//存放角速度
float Angle;//存储滤波后的稳定角度

uint8_t KeyNum, RunFlag;//具体按键和按键标志位1运行，0结束

int16_t LeftPWM, RightPWM;
int16_t AvePWM, DifPWM;

float LeftSpeed, RightSpeed;
float AveSpeed, DifSpeed;


/* 声明定义在 OLED.c 中的全局变量 */
extern volatile uint8_t g_I2C_BusyFlag;

extern UART_HandleTypeDef huart1;
//二值信号量申明
extern osSemaphoreId_t myBinarySem01Handle;
//互斥信号量申明
extern osMutexId_t myMutex01Handle;

// DWT寄存器用于测量执行时间（周期计数）
#define DWT_CR      *(volatile uint32_t *)0xE0001000  //DWT 控制寄存器，用于开启或关闭计数功能。
#define DWT_CYCCNT  *(volatile uint32_t *)0xE0001004  //它是一个 32 位的计数器，CPU 每经过一个时钟周期，它的值就加 1。
#define DEM_CR      *(volatile uint32_t *)0xE000EDFC  //调试异常监控控制寄存器
#define DEM_CR_TRCENA                   (1 << 24)//激活，供电
#define DWT_CR_CYCCNTENA                (1 << 0)//CPU时钟开始累加计数

// 记录各任务执行时间（微秒）
uint32_t Balance_ExecTime_us = 0;
uint32_t Speed_ExecTime_us = 0;
uint32_t Comms_ExecTime_us = 0;
uint32_t UI_ExecTime_us = 0;

// 记录各任务运行周期（微秒）
uint32_t Balance_CycleTime_us = 0;
uint32_t Speed_CycleTime_us = 0;
uint32_t Comms_CycleTime_us = 0;
uint32_t UI_CycleTime_us = 0;

void StartTimerForRunTimeStats(void)
{
    DEM_CR |= DEM_CR_TRCENA;  //开启跟踪组件的使用权限
    DWT_CYCCNT = 0;  					//将计数器清零
    DWT_CR |= DWT_CR_CYCCNTENA;//正式启用启用循环计数器（CYCCNT）
}

// 这个函数就是 FreeRTOS 用来统计百分比的“尺子”
uint32_t GetTimerForRunTimeStats(void) 
{
    return DWT_CYCCNT; 
}

//关于PID的角动量
PID_t AnglePID = {
	.Kp = 5,
	.Ki = 0.1,
	.Kd = 5,
	
	.OutMax = 60,
	.OutMin = -60,
	
	.OutOffset = 3,
	
	//积分限幅范围
	.ErrorIntMax = 600,
	.ErrorIntMin = -600,
};

//关于PID的角动量
PID_t SpeedPID = {
	.Kp = 2,
	.Ki = 0.05,
	.Kd = 0,
	
	.OutMax = 20,
	.OutMin = -20,//角度环最大倾角范围是正负20，最大调控角度
	
		//积分限幅范围
	.ErrorIntMax = 150,
	.ErrorIntMin = -150,
};

PID_t TurnPID = {
	.Kp = 4,
	.Ki = 3,
	.Kd = 0,
	
	.OutMax = 40,
	.OutMin = -40,//转向环最大倾角范围是正负40
	
		//积分限幅范围
	.ErrorIntMax = 20,
	.ErrorIntMin = -20,
};

void vTask_Balance()
{

	static uint32_t last_start_time = 0;
	
    while(1)
    {
        // 死等 TIM1 中断发来的通知 (一直阻塞，不消耗 CPU，直到 5ms 信号到来)
        // pdTRUE 表示收到后将通知值清零，模拟二值特性
				//在这里使用if会导致程序干等，不利于稳定，
				//根据设置，这里5ms进入一次
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
				
			
				//开始DWT计数
				uint32_t start_time = DWT_CYCCNT;
        if (last_start_time != 0) {
            Balance_CycleTime_us = (start_time - last_start_time) / (SystemCoreClock / 1000000);
        }
				last_start_time = start_time;

        // ==========================================
        // 此时刚刚经过绝对精准的 5ms，开始姿态解算和 PID
        // ==========================================
        
				KeyNum = Key_GetNum();
        if(KeyNum == 1) // K1按键实现启停切换
        {
            if(RunFlag == 0) 
            {
                // 重启前消除历史积分与误差，防止积分饱和导致的开机暴冲
                PID_Init(&AnglePID);
                PID_Init(&SpeedPID);
                PID_Init(&TurnPID);
                RunFlag = 1;
            }
            else
            {
                RunFlag = 0;
            }
        }
        
        // 更新 LED 工作状态指示
        if(RunFlag) { LED_ON(); } else { LED_OFF(); }

			
        MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
        
        GY -= 0; // 消除零点漂移 (根据你实际测试的值调整)
        
        // 1. 加速度计计算角度
        AngleAcc = -atan2(AX , AZ) / 3.14159f * 180.0f;
        AngleAcc -= 0; // 调节机械中值
        
        // 2. 陀螺仪积分计算角度 (注意：这里积分时间 dt 必须是 0.005)
        AngleGyro = Angle + GY / 32768.0f * 2000.0f * 0.005f;
        
        // 3. 互补滤波
        float Alpha = 0.01f; // 滤波系数
        Angle = Alpha * AngleAcc + (1.0f - Alpha) * AngleGyro;
        
        // 4. 倾角过大保护，防止小车倒地后电机狂转
        if (Angle > 50.0f || Angle < -50.0f)
        {
            RunFlag = 0;
					  Motor_SetPWM(1, 0);
						Motor_SetPWM(2, 0);
        }
        
        // 5. PID 计算与电机输出
        if(RunFlag)
        {
            // 直立环 PID
            AnglePID.Actual = Angle;
            PID_Update(&AnglePID);
            AvePWM = -AnglePID.Out; 
            
            // 叠加转向环的输出 (DifPWM 由 Speed_Task 计算并提供)
            LeftPWM = AvePWM + DifPWM / 2;
            RightPWM = AvePWM - DifPWM / 2;
            
            // PWM 限幅 (因为你的 TIM2 周期设置为了 100，所以限幅 100 即满占空比)
            if(LeftPWM > 100) LeftPWM = 100; else if(LeftPWM < -100) LeftPWM = -100;
            if(RightPWM > 100) RightPWM = 100; else if(RightPWM < -100) RightPWM = -100;
            
            // 设置电机速度 (注意：Motor_SetPWM 内部需处理正负号和方向引脚)
            Motor_SetPWM(1, LeftPWM);
            Motor_SetPWM(2, RightPWM);
        }
        else
        {
            // 电机停转
            Motor_SetPWM(1, 0);
            Motor_SetPWM(2, 0);
        }
				
		 Balance_ExecTime_us = (DWT_CYCCNT - start_time) / (SystemCoreClock / 1000000);
    }
}

void vTask_Speed()
{
	// 获取进入时间
	TickType_t xLastWakeTime = xTaskGetTickCount();
	static uint32_t last_start_time = 0;
	
	 while(1)
    {
			//获取时间
				uint32_t start_time = DWT_CYCCNT;
        if (last_start_time != 0) {
            Speed_CycleTime_us = (start_time - last_start_time) / (SystemCoreClock / 1000000);
        }
        last_start_time = start_time;
			
        LeftSpeed = Encoder_Get(1) / 44.0f / 0.05f / 9.27666f;
        RightSpeed = Encoder_Get(2) / 44.0f / 0.05f / 9.27666f;
        
        AveSpeed = (LeftSpeed + RightSpeed) / 2.0f;
        DifSpeed = (LeftSpeed - RightSpeed);
        
        if (RunFlag)
        {
            SpeedPID.Actual = AveSpeed;
            PID_Update(&SpeedPID);
            AnglePID.Target = SpeedPID.Out;
            
            TurnPID.Actual = DifSpeed;
            PID_Update(&TurnPID);
            DifPWM = TurnPID.Out;
        }
        else
        {
            AnglePID.Target = 0;
            DifPWM = 0;
        }
				
				Speed_ExecTime_us = (DWT_CYCCNT - start_time) / (SystemCoreClock / 1000000);
				
				//绝对延迟50ms
     vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
		}
		
}

void vTask_Comms()
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	static uint32_t last_start_time = 0;
	static char pcWriteBuffer[512];//对于内存占用情况定义数组存放数据
	while(1)
	{
		
		uint32_t start_time = DWT_CYCCNT;
    if (last_start_time != 0) {
            Comms_CycleTime_us = (start_time - last_start_time) / (SystemCoreClock / 1000000);
        }
    last_start_time = start_time;
		
    //使用蓝牙获取PID值
		if (BlueSerial_RxFlag == 1)
		{

			
			//strtok strcmp atoi/atof
			
			char *Tag = strtok(BlueSerial_RxPacket, ",");
			if (strcmp(Tag, "key") == 0)//收到按键数据包
			{
				char *Name = strtok(NULL, ",");
				char *Action = strtok(NULL, ",");
				
				
			}
			else if (strcmp(Tag, "slider") == 0)//滑杆	数据包
			{
				char *Name = strtok(NULL, ",");
				char *Value = strtok(NULL, ",");
				
				if (strcmp(Name, "AngleKp") == 0 )
				{
					AnglePID.Kp = atof(Value);//atof转化为浮点数
				}
				else if (strcmp(Name, "AngleKi") == 0 )
				{
					AnglePID.Ki = atof(Value);//atof转化为浮点数
				}
				else if (strcmp(Name, "AngleKd") == 0 )
				{
					AnglePID.Kd = atof(Value);//atof转化为浮点数
				}
				else if (strcmp(Name, "SpeedKp") == 0 )
				{
					SpeedPID.Kp = atof(Value);//atof转化为浮点数
				}
				else if (strcmp(Name, "SpeedKi") == 0 )
				{
					SpeedPID.Ki = atof(Value);//atof转化为浮点数
				}
				else if (strcmp(Name, "SpeedKd") == 0 )
				{
					SpeedPID.Kd = atof(Value);//atof转化为浮点数
				}
				
				//以下对于转向换调控
				else if (strcmp(Name, "TurnKp") == 0 )
				{
					TurnPID.Kp = atof(Value);//atof转化为浮点数
				}
				else if (strcmp(Name, "TurnKi") == 0 )
				{
					TurnPID.Ki = atof(Value);//atof转化为浮点数
				}
				else if (strcmp(Name, "TurnKd") == 0 )
				{
					TurnPID.Kd = atof(Value);//atof转化为浮点数
				}
				else if (strcmp(Name, "Offset") == 0 )
				{
					AnglePID.OutOffset = atof(Value);//atof转化为浮点数
				}
			}
			else if (strcmp(Tag, "joystick") == 0)//摇杆数据包
			{
				int8_t LH = atoi(strtok(NULL, ","));
				int8_t LV = atoi(strtok(NULL, ","));
				int8_t RH = atoi(strtok(NULL, ","));
				int8_t RV = atoi(strtok(NULL, ","));
				
				SpeedPID.Target = LV / 25.0;
				TurnPID.Target = RH / 25.0;
			}
			
			BlueSerial_RxFlag = 0;
		}
		
//		// 打印4个任务的剩余堆栈空间（单位是字，1字=4字节）
//		// 只有在句柄有效的情况下才打印，防止初始化未完成时崩溃
//		if (BalanceTask_Handler && SpeedTask_Handler && CommsTask_Handler && UITask_Handler)
//		{
//			printf("FreeStack - Balance: %lu, Speed: %lu, Comms: %lu, UI: %lu\r\n",
//				   (unsigned long)uxTaskGetStackHighWaterMark(BalanceTask_Handler),
//				   (unsigned long)uxTaskGetStackHighWaterMark(SpeedTask_Handler),
//				   (unsigned long)uxTaskGetStackHighWaterMark(CommsTask_Handler),
//				   (unsigned long)uxTaskGetStackHighWaterMark(UITask_Handler));
//		}
		
		// --- 添加 CPU 占用率打印 ---
    // 建议每隔 1000ms 或 2000ms 打印一次，否则串口会刷屏太快
        static uint32_t last_print_tick = 0;
        if (xTaskGetTickCount() - last_print_tick > pdMS_TO_TICKS(2000)) 
        {
            last_print_tick = xTaskGetTickCount();
            
            printf("\r\n--- Task CPU Usage (DWT Based) ---\r\n");
            printf("Task Name\tAbs Time\tTime %%\r\n");
            printf("----------------------------------------------\r\n");
            
            // 获取统计字符串
            vTaskGetRunTimeStats(pcWriteBuffer);
            
            // 打印生成的表格
            printf("%s", pcWriteBuffer);
            printf("----------------------------------------------\r\n");
        }
		

//       // 打印每个任务的执行时间和周期时间 (单位：微秒)
//    printf("Exec(us) - B:%lu, S:%lu, C:%lu, U:%lu | Cycle(us) - B:%lu, S:%lu, C:%lu, U:%lu\r\n",
//             Balance_ExecTime_us, Speed_ExecTime_us, Comms_ExecTime_us, UI_ExecTime_us,
//             Balance_CycleTime_us, Speed_CycleTime_us, Comms_CycleTime_us, UI_CycleTime_us);
//		
		BlueSerial_Printf("[plot,%f,%f]", TurnPID.Target,DifSpeed);
//		
		Comms_ExecTime_us = (DWT_CYCCNT - start_time) / (SystemCoreClock / 1000000);
//		
				
				
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(80));
	}
	
}

void vTask_UI()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    static uint32_t last_start_time = 0;
    
    while(1)
    {
        // 只有当 DMA 传输不忙时，才进行“绘图”和“更新”
        if (g_I2C_BusyFlag == 0) 
        {
            uint32_t start_time = DWT_CYCCNT; // 记录开始绘图的时间点

            // --- 周期时间统计 ---
            if (last_start_time != 0) 
            {
                UI_CycleTime_us = (start_time - last_start_time) / (SystemCoreClock / 1000000);
            }
            last_start_time = start_time;

            // --- 绘图逻辑 ---
            OLED_Clear(); // 修改 Canvas
            
            // 角度环数据
            OLED_Printf(0, 0, OLED_6X8, "  Angle");
            OLED_Printf(0, 8, OLED_6X8, "P:%05.2f", AnglePID.Kp);
            OLED_Printf(0, 16, OLED_6X8, "I:%05.2f", AnglePID.Ki);
            OLED_Printf(0, 24, OLED_6X8, "D:%05.2f", AnglePID.Kd);
            OLED_Printf(0, 32, OLED_6X8, "T:%05.1f", AnglePID.Target);
            OLED_Printf(0, 40, OLED_6X8, "A:%05.1f", Angle);
            OLED_Printf(0, 48, OLED_6X8, "O:%05.1f", AnglePID.Out);
            OLED_Printf(0, 56, OLED_6X8, "GY:%+05d", GY);
            
            // 速度环数据
            OLED_Printf(50, 0, OLED_6X8, "Speed");
            OLED_Printf(50, 8, OLED_6X8, ":%05.2f", SpeedPID.Kp);
            OLED_Printf(50, 16, OLED_6X8, ":%05.2f", SpeedPID.Ki);
            OLED_Printf(50, 24, OLED_6X8, ":%05.2f", SpeedPID.Kd);
            OLED_Printf(50, 32, OLED_6X8, ":%05.1f", SpeedPID.Target);
            OLED_Printf(50, 40, OLED_6X8, ":%05.1f", AveSpeed);
            OLED_Printf(50, 48, OLED_6X8, ":%05.1f", SpeedPID.Out);
            
            // 转向环数据
            OLED_Printf(88, 0, OLED_6X8, "Turn");
            OLED_Printf(88, 8, OLED_6X8, ":%05.2f", TurnPID.Kp);
            OLED_Printf(88, 16, OLED_6X8, ":%05.2f", TurnPID.Ki);
            OLED_Printf(88, 24, OLED_6X8, ":%05.2f", TurnPID.Kd);
            OLED_Printf(88, 32, OLED_6X8, ":%05.1f", TurnPID.Target);
            OLED_Printf(88, 40, OLED_6X8, ":%05.1f", DifSpeed);
            OLED_Printf(88, 48, OLED_6X8, ":%05.1f", TurnPID.Out);

            // --- 提交更新 ---
            OLED_Update(); // 交换指针并开启 DMA

            // --- 执行时间统计 (此时包含所有的 Printf 耗时) ---
            UI_ExecTime_us = (DWT_CYCCNT - start_time) / (SystemCoreClock / 1000000);
        }
        
        // 绝对延时 100ms
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}




/*
	重定向，让printf打印内容从UART1发出
*/
int fputc(int ch, FILE *f)//重定向，在串口中使用printf函数
{
	
//	uint8_t c = (uint8_t)ch; 
//  // 发送这个确定的 1字节变量 的地址
//	//建议使用分开来写，直接强转可能会有不知名错误
//  HAL_UART_Transmit(&huart1, &c, 1, 10); 
	/*
	对于一个真实的结构体来说，使用“."来访问对应的内部成员
	对于结构体指针来说，使用“->”获取指向方向的寄存器
	*/
	
	/*
	当状态寄存器SR和发送数据寄存器的控制位为空时，表示字节发送完毕
	可以退出while循环
	*/
	while((huart1.Instance->SR & USART_SR_TXE) ==0)
	{
		;
	}
	/*写入数据并触发发送*/
	huart1.Instance->DR = *(uint8_t*) &ch;
	
	return ch;
}
