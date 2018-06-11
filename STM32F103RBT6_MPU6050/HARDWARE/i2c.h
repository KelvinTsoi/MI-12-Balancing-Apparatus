#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "sys.h"

#define MPU_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define MPU_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

#define MPU_IIC_SCL    PBout(10)
#define MPU_IIC_SDA    PBout(11)
#define MPU_READ_SDA   PBin(11)

void MPU_IIC_Delay(void);
void MPU_IIC_Init(void);
void MPU_IIC_Start(void);
void MPU_IIC_Stop(void);
void MPU_IIC_Send_Byte(u8 txd);
u8 MPU_IIC_Read_Byte(unsigned char ack);
u8 MPU_IIC_Wait_Ack(void);
void MPU_IIC_Ack(void);
void MPU_IIC_NAck(void);
void IMPU_IC_Write_One_Byte(u8 daddr, u8 addr, u8 data);
u8 MPU_IIC_Read_One_Byte(u8 daddr, u8 addr);

#endif
















