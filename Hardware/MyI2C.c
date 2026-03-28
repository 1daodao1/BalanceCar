#include "main.h"

static void MyI2C_Delay(void)
{
    volatile uint32_t i = 1; // 极短延时即可，原来15太长导致占用大量CPU
    while(i--) { __NOP(); }
}

//���庯�������ڲ����˿ڵĺ������з�װ��W��д��R�Ƕ�
void MyI2C_W_SCL(uint8_t BitValue)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET);
	MyI2C_Delay();
}

void MyI2C_W_SDA(uint8_t BitValue)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET);
	MyI2C_Delay();
}

uint8_t MyI2C_R_SDA(void)
{
	uint8_t BitValue;
	BitValue = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET ? 1 : 0;
	MyI2C_Delay();
	return BitValue;
}

void MyI2C_Init(void)
{
	/*����ʱ��*/
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		//����GPIOB��ʱ��
//	
//	/*GPIO��ʼ��*/
//	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//	//��©���ģʽ��Ȼ�������룬����ʱ�������1����ֱ�Ӷ�ȡ�������ݼĴ���
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);						
//	
//	/*����GPIO��ʼ�����Ĭ�ϵ�ƽ*/
//	GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);		
	/* GPIO已在CubeMX中初始化，这里仅设初始状态 */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);	
	
}

//Ϊ�˱�֤�ڶ��η��͵�׼ȷ�ԣ�һ��Ҫ�ȶ�SDA�øߵ�ƽ
void MyI2C_Start(void)
{
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(0);
}

void MyI2C_Stop(void)
{
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);
}

//����һ���ֽ�
void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8 ; i++)
	{
		MyI2C_W_SDA(Byte & 0x80 >> i);
		//�ð�λ��ķ�ʽ��ȡ�����ݵ�ĳһλ��ĳ��λ�������ƣ�����8�Σ���֤һ���ֽ�
		MyI2C_W_SCL(1);//�ͷ�SCL
		MyI2C_W_SCL(0);//����ʱ�Ӹ���һ������
	}
	
}

//����һ���ֽ�
uint8_t MyI2C_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;
	MyI2C_W_SDA(1);
	for (i = 0; i < 8 ; i++)
	{
		MyI2C_W_SCL(1);
		if (MyI2C_R_SDA() == 1){Byte |= (0x80 >> i);}
		MyI2C_W_SCL(0);//����ʱ�Ӹ���һ������
	}
	return Byte;
}

//����Ӧ��
void MyI2C_SendAck(uint8_t AckBit)
{

		MyI2C_W_SDA(AckBit);//������AckBit�ŵ�SDA��
		MyI2C_W_SCL(1);//�ӻ���ȡӦ��
		MyI2C_W_SCL(0);//����ʱ�Ӹ���һ�����壬������һ��Ӧ��
	
	
}

//����Ӧ��
uint8_t MyI2C_ReceiveAck(void)
{
	//��������ʱ��SCL�͵�ƽ��
		uint8_t AckBit;
		MyI2C_W_SDA(1);
	//�����ͷ�SDA����ֹ���Ŵӻ���ͬʱ�Ѹߵ�ƽ����SDA�ϣ��ӻ���ʱ������SDA�ٴ����ͣ�0��ʾ�ӻ�����Ӧ��
		MyI2C_W_SCL(1);
	//SCL�ߵ�ƽ��������ȡӦ��λ
		AckBit = MyI2C_R_SDA();
		MyI2C_W_SCL(0);//����ʱ�Ӹ���һ�����壬������һ��ʱ��Ԫ

		return AckBit;
}
