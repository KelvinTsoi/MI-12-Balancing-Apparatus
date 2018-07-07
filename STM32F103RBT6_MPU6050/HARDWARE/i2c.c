#include "i2c.h"
#include "delay.h"

void IIC_Delay(void)
{
  delay_us(2);
}

int MPU_IIC_Init(int type)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  switch(type)
  {
    case ALPHA_SENSOR:
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);
      break;
    case BETA_SENSOR:
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);
      break;
    default:
      return 1;
  }
  return 0;
}

void MPU_IIC_Start(int type)
{
  switch(type)
  {
    case ALPHA_SENSOR:
      ALPHA_SDA_OUT();
      ALPHA_IIC_SDA = 1;
      ALPHA_IIC_SCL = 1;
      IIC_Delay();
      ALPHA_IIC_SDA = 0;
      IIC_Delay();
      ALPHA_IIC_SCL = 0;
    case BETA_SENSOR:
      BETA_SDA_OUT();
      BETA_IIC_SDA = 1;
      BETA_IIC_SCL = 1;
      IIC_Delay();
      BETA_IIC_SDA = 0;
      IIC_Delay();
      BETA_IIC_SCL = 0;
    default:
      break;
  }
}

void MPU_IIC_Stop(int type)
{
  switch(type)
  {
    case ALPHA_SENSOR:
      ALPHA_SDA_OUT();
      ALPHA_IIC_SCL = 0;
      ALPHA_IIC_SDA = 0;
      IIC_Delay();
      ALPHA_IIC_SCL = 1;
      ALPHA_IIC_SDA = 1;
      IIC_Delay();
      break;
    case BETA_SENSOR:
      BETA_SDA_OUT();
      BETA_IIC_SCL = 0;
      BETA_IIC_SDA = 0;
      IIC_Delay();
      BETA_IIC_SCL = 1;
      BETA_IIC_SDA = 1;
      IIC_Delay();
      break;
    default:
      break;
  }
}

u8 MPU_IIC_Wait_Ack(int type)
{
  u8 ucErrTime = 0;
  switch(type)
  {
    case ALPHA_SENSOR:
      ALPHA_SDA_IN();
      ALPHA_IIC_SDA = 1;
      IIC_Delay();
      ALPHA_IIC_SCL = 1;
      IIC_Delay();
      while(ALPHA_READ_SDA)
      {
        ucErrTime++;
        if(ucErrTime > 250)
        {
          MPU_IIC_Stop(type);
          return 1;
        }
      }
      ALPHA_IIC_SCL = 0;
      return 0;
    case BETA_SENSOR:
      BETA_SDA_IN();
      BETA_IIC_SDA = 1;
      IIC_Delay();
      BETA_IIC_SCL = 1;
      IIC_Delay();
      while(BETA_READ_SDA)
      {
        ucErrTime++;
        if(ucErrTime > 250)
        {
          MPU_IIC_Stop(type);
          return 1;
        }
      }
      BETA_IIC_SCL = 0;
      return 0;
    default:
      break;
  }
  return 1;
}

void MPU_IIC_Ack(int type)
{
  switch(type)
  {
    case ALPHA_SENSOR:
      ALPHA_IIC_SCL = 0;
      ALPHA_SDA_OUT();
      ALPHA_IIC_SDA = 0;
      IIC_Delay();
      ALPHA_IIC_SCL = 1;
      IIC_Delay();
      ALPHA_IIC_SCL = 0;
      break;
    case BETA_SENSOR:
      BETA_IIC_SCL = 0;
      BETA_SDA_OUT();
      BETA_IIC_SDA = 0;
      IIC_Delay();
      BETA_IIC_SCL = 1;
      IIC_Delay();
      BETA_IIC_SCL = 0;
      break;
    default:
      break;
  }
}

void MPU_IIC_NAck(int type)
{
  switch(type)
  {
    case ALPHA_SENSOR:
      ALPHA_IIC_SCL = 0;
      ALPHA_SDA_OUT();
      ALPHA_IIC_SDA = 1;
      IIC_Delay();
      ALPHA_IIC_SCL = 1;
      IIC_Delay();
      ALPHA_IIC_SCL = 0;
      break;
    case BETA_SENSOR:
      BETA_IIC_SCL = 0;
      BETA_SDA_OUT();
      BETA_IIC_SDA = 1;
      IIC_Delay();
      BETA_IIC_SCL = 1;
      IIC_Delay();
      BETA_IIC_SCL = 0;
      break;
    default:
      break;
  }
}

void MPU_IIC_Send_Byte(u8 txd, int type)
{
  u8 t;
  switch(type)
  {
    case ALPHA_SENSOR:
      ALPHA_SDA_OUT();
      ALPHA_IIC_SCL = 0;
      for(t = 0; t < 8; t++)
      {
        ALPHA_IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        ALPHA_IIC_SCL = 1;
        IIC_Delay();
        ALPHA_IIC_SCL = 0;
        IIC_Delay();
      }
      break;
    case BETA_SENSOR:
      BETA_SDA_OUT();
      BETA_IIC_SCL = 0;
      for(t = 0; t < 8; t++)
      {
        BETA_IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        BETA_IIC_SCL = 1;
        IIC_Delay();
        BETA_IIC_SCL = 0;
        IIC_Delay();
      }
      break;
    default:
      break;
  }
}

u8 MPU_IIC_Read_Byte(unsigned char ack, int type)
{
  unsigned char i, receive = 0;
  switch(type)
  {
    case ALPHA_SENSOR:
      ALPHA_SDA_IN();
      for(i = 0; i < 8; i++ )
      {
        ALPHA_IIC_SCL = 0;
        IIC_Delay();
        ALPHA_IIC_SCL = 1;
        receive <<= 1;
        if(ALPHA_READ_SDA)
        {
          receive++;
        }
        IIC_Delay();
      }
      if (!ack)
      {
        MPU_IIC_NAck(type);
      }
      else
      {
        MPU_IIC_Ack(type);
      }
      return receive;
    case BETA_SENSOR:
      BETA_SDA_IN();
      for(i = 0; i < 8; i++ )
      {
        BETA_IIC_SCL = 0;
        IIC_Delay();
        BETA_IIC_SCL = 1;
        receive <<= 1;
        if(BETA_READ_SDA)
        {
          receive++;
        }
        IIC_Delay();
      }
      if (!ack)
      {
        MPU_IIC_NAck(type);
      }
      else
      {
        MPU_IIC_Ack(type);
      }
      return receive;
    default:
      break;
  }
	return receive;
}
