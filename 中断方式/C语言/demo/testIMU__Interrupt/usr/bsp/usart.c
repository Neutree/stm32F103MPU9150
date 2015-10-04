#include "usart.h"

void Usart_Configuration(void)			//配置Usart1 Tx->PA9 Rx->PA10
{
	 GPIO_InitTypeDef GPIO_InitStructure; //GPIO库函数结构体
	 USART_InitTypeDef USART_InitStructure;//USART库函数结构体
	 USART_ClockInitTypeDef USART_ClockInitStructure;
	 //使能串口1，GPIOA，AFIO总线
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO|RCC_APB2Periph_USART1,ENABLE);

	 /* Configure USART1 Tx (PA9) as alternate function push-pull */
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//PA9时钟速度50MHz
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用输出
	 GPIO_Init(GPIOA, &GPIO_InitStructure);
	 /* Configure USART1 Rx (PA10) as input floating */
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//上拉输入
	 GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	 USART_InitStructure.USART_BaudRate =115200; //波特率9600
	 USART_InitStructure.USART_WordLength = USART_WordLength_8b; //8位数据
	 USART_InitStructure.USART_StopBits = USART_StopBits_1; //1个停止位
	 USART_InitStructure.USART_Parity = USART_Parity_No; //奇偶失能
	 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //发送、接收使能
	
	 USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	 USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;//空闲时钟为低电平
	 USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;//时钟第二个边沿进行数据捕获
	 USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;//最后一位数据的时钟脉冲不从SCLK输出

	 //使能串口1接收中断
	 USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 

	 USART_ClockInit(USART1, &USART_ClockInitStructure);
	 USART_Init(USART1, &USART_InitStructure);	//初始化结构体
	 USART_Cmd(USART1, ENABLE); //使能串口1
//	 USART_ClearFlag(USART1,USART_FLAG_TC);//清除发送完成标志，不然有时可能会有第一个字符发送不出去的情况
	 USART_GetFlagStatus(USART1, USART_FLAG_TC);//清除发送完成标志，不然有时可能会有第一个字符发送不出去的情况
}
void Usart_NVIC_Configuration(void)
{
	 NVIC_InitTypeDef NVIC_InitStructure;
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	/******************注册串口1中断服务函数************************/
   NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//配置串口中断
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//占先式优先级设置为0
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //副优先级设置为0
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//中断开启
   NVIC_Init(&NVIC_InitStructure);//中断初始化
}
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
/***************************START*********************/
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	USART1->DR = (u8) ch;      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	return ch;
}
#endif 
/***************************END*********************/

void USART1_Putc(uint8_t data)			//利用Usart1发送一个8位数据
{
    USART1->DR = (uint8_t)data; //要发送的字符赋给串口数据寄存器  
	while((USART1->SR&0X40)==0); //等待发送完成  
}
void USART1_Puts(uint8_t * buffer)			//利用Usart1发送一个字符串
{
    while(*buffer)
    {
        USART1->DR= *buffer++;
		while((USART1->SR&0X40)==0);//等待发送完成  
    }
}
void USART1_Send_Enter(void)//利用串口1发送一换行符
{
	USART1_Putc(0x0d);
	USART1_Putc(0x0a);
}

/******************串口1中断服务函数*******************************/
void USART1_IRQHandler(void)
{
	u8 data=0;
   if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
   {
		 data=USART_ReceiveData(USART1);
		USART_SendData(USART1,data);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET){};
	 }
}
