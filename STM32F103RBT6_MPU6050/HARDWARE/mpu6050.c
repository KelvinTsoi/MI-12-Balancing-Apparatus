#include "mpu6050.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"

u8 MPU_Init(int type)
{
  u8 res;
  MPU_IIC_Init(type);
  MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80, type);
  delay_ms(100);
  MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00, type);
  MPU_Set_Gyro_Fsr(3, type);
  MPU_Set_Accel_Fsr(0, type);
  MPU_Set_Rate(50, type);
  MPU_Write_Byte(MPU_INT_EN_REG, 0X00, type);
  MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00, type);
  MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00, type);
  MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80, type);
  res = MPU_Read_Byte(MPU_DEVICE_ID_REG, type);
  if(res == MPU_ADDR)
  {
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01, type);
    MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00, type);
    MPU_Set_Rate(50, type);
  }
  else return 1;
  return 0;
}

u8 MPU_Set_Gyro_Fsr(u8 fsr, int type)
{
  return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3, type);
}

u8 MPU_Set_Accel_Fsr(u8 fsr, int type)
{
  return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3, type);
}

u8 MPU_Set_LPF(u16 lpf, int type)
{
  u8 data = 0;
  if(lpf >= 188)data = 1;
  else if(lpf >= 98)data = 2;
  else if(lpf >= 42)data = 3;
  else if(lpf >= 20)data = 4;
  else if(lpf >= 10)data = 5;
  else data = 6;
  return MPU_Write_Byte(MPU_CFG_REG, data, type);
}

u8 MPU_Set_Rate(u16 rate, int type)
{
  u8 data;
  if(rate > 1000)rate = 1000;
  if(rate < 4)rate = 4;
  data = 1000 / rate - 1;
  data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data, type);
  return MPU_Set_LPF(rate / 2, type);
}

short MPU_Get_Temperature(int type)
{
  u8 buf[2];
  short raw;
  float temp;
  MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf, type);
  raw = ((u16)buf[0] << 8) | buf[1];
  temp = 36.53 + ((double)raw) / 340;
  return temp * 100;;
}

u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz, int type)
{
  u8 buf[6], res;
  res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf, type);
  if(res == 0)
  {
    *gx = ((u16)buf[0] << 8) | buf[1];
    *gy = ((u16)buf[2] << 8) | buf[3];
    *gz = ((u16)buf[4] << 8) | buf[5];
  }
  return res;;
}

u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az, int type)
{
  u8 buf[6], res;
  res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf, type);
  if(res == 0)
  {
    *ax = ((u16)buf[0] << 8) | buf[1];
    *ay = ((u16)buf[2] << 8) | buf[3];
    *az = ((u16)buf[4] << 8) | buf[5];
  }
  return res;;
}

u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf, int type)
{
  u8 i;
  MPU_IIC_Start(type);
  MPU_IIC_Send_Byte((addr << 1) | 0, type);
  if(MPU_IIC_Wait_Ack(type))
  {
    MPU_IIC_Stop(type);
    return 1;
  }
  MPU_IIC_Send_Byte(reg, type);
  MPU_IIC_Wait_Ack(type);
  for(i = 0; i < len; i++)
  {
    MPU_IIC_Send_Byte(buf[i], type);
    if(MPU_IIC_Wait_Ack(type))
    {
      MPU_IIC_Stop(type);
      return 1;
    }
  }
  MPU_IIC_Stop(type);
  return 0;
}

u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf, int type)
{
  MPU_IIC_Start(type);
  MPU_IIC_Send_Byte((addr << 1) | 0, type);
  if(MPU_IIC_Wait_Ack(type))
  {
    MPU_IIC_Stop(type);
    return 1;
  }
  MPU_IIC_Send_Byte(reg, type);
  MPU_IIC_Wait_Ack(type);
  MPU_IIC_Start(type);
  MPU_IIC_Send_Byte((addr << 1) | 1, type);
  MPU_IIC_Wait_Ack(type);
  while(len)
  {
    if(len == 1)*buf = MPU_IIC_Read_Byte(0, type);
    else *buf = MPU_IIC_Read_Byte(1, type);
    len--;
    buf++;
  }
  MPU_IIC_Stop(type);
  return 0;
}

u8 MPU_Write_Byte(u8 reg, u8 data, int type)
{
  MPU_IIC_Start(type);
  MPU_IIC_Send_Byte((MPU_ADDR << 1) | 0, type);
  if(MPU_IIC_Wait_Ack(type))
  {
    MPU_IIC_Stop(type);
    return 1;
  }
  MPU_IIC_Send_Byte(reg, type);
  MPU_IIC_Wait_Ack(type);
  MPU_IIC_Send_Byte(data, type);
  if(MPU_IIC_Wait_Ack(type))
  {
    MPU_IIC_Stop(type);
    return 1;
  }
  MPU_IIC_Stop(type);
  return 0;
}

u8 MPU_Read_Byte(u8 reg, int type)
{
  u8 res;
  MPU_IIC_Start(type);
  MPU_IIC_Send_Byte((MPU_ADDR << 1) | 0, type);
  MPU_IIC_Wait_Ack(type);
  MPU_IIC_Send_Byte(reg, type);
  MPU_IIC_Wait_Ack(type);
  MPU_IIC_Start(type);
  MPU_IIC_Send_Byte((MPU_ADDR << 1) | 1, type);
  MPU_IIC_Wait_Ack(type);
  res = MPU_IIC_Read_Byte(0, type);
  MPU_IIC_Stop(type);
  return res;
}
