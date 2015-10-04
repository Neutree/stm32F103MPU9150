#ifndef _DELAY_H
#define _DELAY_H
#include "stm32f10x.h"
void RCC_HSE_Configuration(void);
void delay(u32 nCount);
void delay_us(u32 nus);
void delay_ms(u16 nms);
void delay_s(u32 ns);
#endif
