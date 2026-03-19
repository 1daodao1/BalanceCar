#ifndef __MYI2C_H
#define __MYI2C_H


void MyI2C_Init(void);

void MyI2C_Start(void);

void MyI2C_Stop(void);

//发送一个字节
void MyI2C_SendByte(uint8_t Byte);

//接收一个字节
uint8_t MyI2C_ReceiveByte(void);

//发送应答
void MyI2C_SendAck(uint8_t AckBit);

//接收应答
uint8_t MyI2C_ReceiveAck(void);

#endif
