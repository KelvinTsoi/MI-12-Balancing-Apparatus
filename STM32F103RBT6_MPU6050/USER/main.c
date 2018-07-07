/**
  ******************************************************************************
  * @author  Kelvin Tsoi
  * @version V2.0.01
  * @date    08-April-2018
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "delay.h"
#include "usart.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include <stdio.h>

typedef enum {FALSE = 0, TRUE = 1} bool;

/* Private functions ---------------------------------------------------------*/

uint8_t UART1TxBuffer[MAX_UART1_TX_BUFFER_SIZE];

uint8_t g_IsUartTxBusy = 0;

uint32_t g_byDmaBufferCurrentTab = 0;

float pitch, roll, yaw = 0;

static bool g_IsInitOk = FALSE;

int g_PingPongOperation = 0;

void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
    if(g_IsInitOk == TRUE && !g_IsUartTxBusy)
    {
      if(g_PingPongOperation == 0)
      {
        if(mpu_dmp_get_data(&pitch, &roll, &yaw, ALPHA_SENSOR) == 0)
        {
          g_PingPongOperation = 1;
					Encode(LEFT_HAND_SIDE, pitch, roll, yaw);
          SendMsg();
        }
      }
    }
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}

void EXTI1_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
    if(g_IsInitOk == TRUE && !g_IsUartTxBusy)
    {
      if(g_PingPongOperation == 1)
      {
        if(mpu_dmp_get_data(&pitch, &roll, &yaw, BETA_SENSOR) == 0)
        {
          g_PingPongOperation = 0;
					Encode(RIGHT_HAND_SIDE, pitch, roll, yaw);
          SendMsg();
        }
      }
    }
    EXTI_ClearITPendingBit(EXTI_Line1);
  }
}

/**
  * @brief
  * @param
  * @retval
  */
int Sensor_Exint(int type)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  switch(type)
  {
    case 0:
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
      GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      EXTI_ClearITPendingBit(EXTI_Line0);
      GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
      EXTI_InitStructure.EXTI_Line = EXTI_Line0;
      EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
      EXTI_InitStructure.EXTI_LineCmd = ENABLE;
      EXTI_Init(&EXTI_InitStructure);
      NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
      NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
      break;
    case 1:
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
      GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      EXTI_ClearITPendingBit(EXTI_Line1);
      GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
      EXTI_InitStructure.EXTI_Line = EXTI_Line1;
      EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
      EXTI_InitStructure.EXTI_LineCmd = ENABLE;
      EXTI_Init(&EXTI_InitStructure);
      NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
      NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
      break;
    default:
      return 1;
  }
  return 0;
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  int ret = 0;

  delay_init();

  USART1_Init(2000000);

  delay_ms(5);

  if((ret = Sensor_Exint(BETA_SENSOR)) != 0)
  {
    //printf("MPU6050 Exint Error! Error code [%d]\r\n", ret);
  }

  if((ret = Sensor_Exint(ALPHA_SENSOR)) != 0)
  {
    //printf("MPU6050 Exint Error! Error code [%d]\r\n", ret);
  }

  if((ret = MPU_Init(BETA_SENSOR)) != 0)
  {
    //printf("MPU6050 Init Error! Error code [%d]\r\n", ret);
    assert_failed((uint8_t *)__FILE__, __LINE__);
  }

  delay_ms(10);

  if((ret = MPU_Init(ALPHA_SENSOR)) != 0)
  {
    //printf("MPU6050 Init Error! Error code [%d]\r\n", ret);
    assert_failed((uint8_t *)__FILE__, __LINE__);
  }

  delay_ms(10);

  while((ret = mpu_dmp_init(BETA_SENSOR)) != 0)
  {
    //printf("MPU6050 DMP Init Error! Error code [%d], Try again!\r\n");
    delay_ms(200);
  }

  delay_ms(10);

  while((ret = mpu_dmp_init(ALPHA_SENSOR)) != 0)
  {
    //printf("MPU6050 DMP Init Error! Error code [%d], Try again!\r\n");
    delay_ms(200);
  }

  delay_ms(10);

  g_IsInitOk = TRUE;

  /* Infinite loop */
  while (1)
  {}
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
