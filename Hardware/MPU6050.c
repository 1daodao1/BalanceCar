#include "main.h"                  // Device header
#include "MyI2C.h"
#include "MPU6050_Reg.h"

//对于寄存器，如果宏定义位比较多，则可以把他放到外部，例如文件MPU6050_Reg.h文件里面显示
#define MPU6050_ADDRESS  0xD0

//每发送一位，都需要接收应答

//指定地址写的时序
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MyI2C_Start();//起始
	MyI2C_SendByte(MPU6050_ADDRESS);//从机地址+读写位，采用宏定义的方式，方便修改
	MyI2C_ReceiveAck();//接收应答位，可以在这里加判断语句，进行操作
	MyI2C_SendByte(RegAddress);//指定寄存器地址，在MPU6050的当前地址指针中
	MyI2C_ReceiveAck();
	
	//指定地址写多字节的话，对于下面两句进行for循环
	MyI2C_SendByte(Data);//指定写入指定寄存器地址下的数据
	MyI2C_ReceiveAck();
	
	MyI2C_Stop();//终止这个时序
}

//指定地址读
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;//存接收的数据
	
	//第一步，指定地址
	MyI2C_Start();//起始
	MyI2C_SendByte(MPU6050_ADDRESS);//从机地址+读写位，最后一位为0表示写入位，采用宏定义的方式，方便修改
	MyI2C_ReceiveAck();//接收应答位，可以在这里加判断语句，进行操作
	MyI2C_SendByte(RegAddress);//指定寄存器地址，在MPU6050的当前地址指针中
	MyI2C_ReceiveAck();
	
	
	//第二步，转入读的程序，首先重新指定读写位，
	MyI2C_Start();//起始
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
	MyI2C_ReceiveAck();//应答过后，总线控制权交给了从机
	//读多字节，将下面两行用for循环嵌套，最后一个字节给非应答
	Data = MyI2C_ReceiveByte();//存接收到的数据
	MyI2C_SendAck(1);//主机接收字节后，要给从机发送应答,0表示给从机应答，1表示不给，
	//其中接收多字节需要给应答，只需要一个字节，则不需要应答
	MyI2C_Stop();
	
	return Data;
}

void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;//存接收的数据
	
	//第一步，指定地址
	MyI2C_Start();//起始
	MyI2C_SendByte(MPU6050_ADDRESS);//从机地址+读写位，最后一位为0表示写入位，采用宏定义的方式，方便修改
	MyI2C_ReceiveAck();//接收应答位，可以在这里加判断语句，进行操作
	MyI2C_SendByte(RegAddress);//指定寄存器地址，在MPU6050的当前地址指针中
	MyI2C_ReceiveAck();
	
	
	//第二步，转入读的程序，首先重新指定读写位，
	MyI2C_Start();//起始
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
	MyI2C_ReceiveAck();//应答过后，总线控制权交给了从机
	//读多字节，将下面两行用for循环嵌套，最后一个字节给非应答
	for( i = 0 ; i < Count; i ++)
	{
		DataArray[i] = MyI2C_ReceiveByte();//存接收到的数据
		if (i < Count - 1)
		{
			MyI2C_SendAck(0);
		}
		else
		{
			MyI2C_SendAck(1);//主机接收字节后，要给从机发送应答,0表示给从机应答，1表示不给，
		}
		//其中接收多字节需要给应答，只需要一个字节，则不需要应答
	}
	MyI2C_Stop();
	

	
}

void MPU6050_Init(void)
{
	MyI2C_Init();//初始化底层函数，类似于类的继承
	
	//查手册！！配置电源管理器1的内容这里含义是，
	//设备不复位，解除睡眠，不需要循环，无关位给0，温度传感器不失能，最后三位001给x陀螺仪时钟
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
	//电源管理器2，00，不需要循环模式唤醒频率，后6位每个轴的待机位都是0，不需要待机
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
	
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07);//采样率分频，值越小越快，现在1ms刷新一次
	MPU6050_WriteReg(MPU6050_CONFIG, 0x00);
	//由于不同模式下会有不同的时延！需要对其正确选择合适的，这里选择了时延最小的
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);//陀螺仪配置寄存器，这里最大量程
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);//加速度计，寄存器这里最大量程
	//当前加速度的计算方法：显示值*16/32767，16是选择的量程，32767是16位有符号数的取值范围-32767-+32767
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
											int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t Data[14];
	
	MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, Data, 14);
	
	*AccX = (Data[0] << 8) | Data[1];
	*AccY = (Data[2] << 8) | Data[3];
	*AccZ = (Data[4] << 8) | Data[5];
	
	*GyroX = (Data[8] << 8) | Data[9];
	*GyroY = (Data[10] << 8) | Data[11];
	*GyroZ = (Data[12] << 8) | Data[13];
}

//用指针的地址传递，进行多参数的传递,这里是6个输出参数,加速度3位，陀螺仪3位
//也可以使用其他方法，全局变量&结构体
//void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
//											int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
//{
//	uint16_t DataH, DataL;
//	//加速度寄存器X轴的高低8位
//	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
//	*AccX = (DataH << 8) | DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
//	*AccY = (DataH << 8) | DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
//	*AccZ = (DataH << 8) | DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
//	*GyroX = (DataH << 8) | DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
//	*GyroY = (DataH << 8) | DataL;
//	
//	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
//	*GyroZ = (DataH << 8) | DataL;
//}
