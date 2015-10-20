#ifndef _USART_H
#define _USART_H
#include "stm32f10x.h"
#include "stdio.h"
void Usart_Configuration(void);			//配置Usart1 Tx->PA9 Rx->PA10
void Usart_NVIC_Configuration(void);//中断优先级设定
void USART1_Putc(uint8_t data);			//利用Usart1发送一个8位数据
void USART1_Puts(uint8_t * buffer);			//利用Uart4发送一个字符串

void USART1_Send_Enter(void);//利用串口1发送一换行符

#endif
