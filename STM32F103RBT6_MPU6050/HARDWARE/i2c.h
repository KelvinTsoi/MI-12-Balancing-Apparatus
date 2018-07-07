#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "sys.h"

#define ALPHA_SENSOR	0
#define BETA_SENSOR		1

#define ALPHA_IIC_SCL    PBout(6)
#define ALPHA_IIC_SDA    PBout(7)
#define ALPHA_READ_SDA   PBin(7)

#define ALPHA_SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)8<<28;} 
#define ALPHA_SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(u32)3<<28;}

#define BETA_IIC_SCL    PBout(10)
#define BETA_IIC_SDA    PBout(11)
#define BETA_READ_SDA   PBin(11)

#define BETA_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define BETA_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

void IIC_Delay(void);

int MPU_IIC_Init(int type);

void MPU_IIC_Start(int type);

void MPU_IIC_Stop(int type);

void MPU_IIC_Send_Byte(u8 txd, int type);

u8 MPU_IIC_Read_Byte(unsigned char ack, int type);

u8 MPU_IIC_Wait_Ack(int type);

void MPU_IIC_Ack(int type);

void MPU_IIC_NAck(int type);

#endif
















