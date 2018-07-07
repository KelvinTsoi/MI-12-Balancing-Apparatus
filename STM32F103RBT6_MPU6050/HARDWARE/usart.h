#ifndef __UART_H
#define	__UART_H

#include "sys.h"  

#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define LEFT_HAND_SIDE      0x00
#define RIGHT_HAND_SIDE     0x01

#define SINGLE_INSTRUCTION_SIZE		41

#define MAX_UART1_TX_BUFFER_SIZE 	SINGLE_INSTRUCTION_SIZE * 20

extern uint8_t UART1TxBuffer[MAX_UART1_TX_BUFFER_SIZE];

extern uint8_t g_IsUartTxBusy;

extern uint32_t g_byDmaBufferCurrentTab;

#define LEFT_HAND_SIDE      0x00
#define RIGHT_HAND_SIDE     0x01

void USART1_Init(uint32_t BaudRate);

int SendMsg(void);

int Encode(unsigned char direction, const float pitch, const float roll, const float yaw);

#endif /* __UART_H */
