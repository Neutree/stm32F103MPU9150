#ifndef _USART_H
#define _USART_H
#include "stm32f10x.h"
#include "stdio.h"
void Usart_Configuration(void);			//����Usart1 Tx->PA9 Rx->PA10
void Usart_NVIC_Configuration(void);//�ж����ȼ��趨
void USART1_Putc(uint8_t data);			//����Usart1����һ��8λ����
void USART1_Puts(uint8_t * buffer);			//����Uart4����һ���ַ���

void USART1_Send_Enter(void);//���ô���1����һ���з�

#endif
