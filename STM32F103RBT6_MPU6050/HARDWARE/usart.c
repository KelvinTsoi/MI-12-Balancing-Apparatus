#include "usart.h"

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void USART1_Init(uint32_t BaudRate)
{
  //...
  GPIO_InitTypeDef GPIO_InitStructure;

  //...
  USART_InitTypeDef USART_InitStructure;

  //...
  DMA_InitTypeDef DMA_InitStructure;

  //...
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Setup Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* GPIO Configuration -------------------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* USART Configuration ------------------------------------------------------*/
  USART_InitStructure.USART_BaudRate = BaudRate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  /* DMA Configuration -------------------------------------------------------*/
  DMA_DeInit(DMA1_Channel4);

  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&UART1TxBuffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = MAX_UART1_TX_BUFFER_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

  //...
  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

  //...
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);

  //...
  USART_ITConfig(USART1, USART_IT_TC, ENABLE);

  /* Enable USARTy DMA TX request */
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

  /* Enable the USART2 */
  USART_Cmd(USART1, ENABLE);

  /* NVIC Configuration ------------------------------------------------------*/
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  /* Enable USART2 DMA TX Finish Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable USART2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int Encode(unsigned char direction, const float pitch, const float roll, const float yaw)
{
  char tmp_pitch[16] = {0x00};
  char tmp_roll[16] = {0x00};
  char tmp_yaw[16] = {0x00};

  char tmp[64] = {0x00};

  if(pitch >= 0.000000)
  {
    sprintf(tmp_pitch, "[+%010f]", pitch);
  }
  else
  {
    sprintf(tmp_pitch, "[%011f]", pitch);
  }

  if(roll >= 0.000000)
  {
    sprintf(tmp_roll, "[+%010f]", roll);
  }
  else
  {
    sprintf(tmp_roll, "[%011f]", roll);
  }

  if(yaw >= 0.000000)
  {
    sprintf(tmp_yaw, "[+%010f]", yaw);
  }
  else
  {
    sprintf(tmp_yaw, "[%011f]", yaw);
  }

  if(LEFT_HAND_SIDE == direction)
  {
    sprintf((char *)tmp, "L%s%s%sE",  tmp_pitch, tmp_roll, tmp_yaw);
  }
  else if(RIGHT_HAND_SIDE == direction)
  {
    sprintf((char *)tmp, "R%s%s%sE",  tmp_pitch, tmp_roll, tmp_yaw);
  }
  else
  {
    return 2;
  }

  if(g_byDmaBufferCurrentTab == 0)
  {
    memset(UART1TxBuffer, 0x00, sizeof(UART1TxBuffer));
  }

  memcpy(UART1TxBuffer + g_byDmaBufferCurrentTab, tmp, SINGLE_INSTRUCTION_SIZE);
  g_byDmaBufferCurrentTab += SINGLE_INSTRUCTION_SIZE;

  return 0;
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int SendMsg()
{
  if(g_IsUartTxBusy == 1)
  {
    return 1;
  }

  if(g_byDmaBufferCurrentTab >= MAX_UART1_TX_BUFFER_SIZE)
  {		
    DMA_SetCurrDataCounter(DMA1_Channel4, g_byDmaBufferCurrentTab);

    DMA_Cmd(DMA1_Channel4, ENABLE);

    g_IsUartTxBusy = 1;
  }

  return 0;
}
