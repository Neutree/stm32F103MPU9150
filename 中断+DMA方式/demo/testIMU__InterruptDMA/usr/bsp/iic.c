/**
* @file iic.c
* @author 1208077207@qq.com
* @version v1.0
* @date 2015-10-4  v0.1
*       2015-10-20 v1.0
* @pre First initialize the SystemClock eg:72M 
* @brief stm32f10x i2c  lib 
*Introduction:
*		中断+DMA方式I2C，需要发送/接收的消息都由一个循环队列管理，
*		IIC中断优先级最好是最高，可以在 iic.h 宏定义中设置
*		此处设置的默认中断优先级为：分组3（8个抢占优先级（0~7对应高~低），2个响应优先级（0~1对应高~低）） ，IIC事件中断抢占优先级为2，响应优先级为0， IIC错误中断抢占式优先级为0，响应优先级为0,DMA抢占优先级为1，相应优先级为0
*		
*		
*Change Log:
*		2015-10-4  7位地址主机读写基本实现
*		2015-10-20 错误容错的处理，基本实现可以应对断开IIC设备硬件连接的紧急情况
*How To Use It：
*		1：工程中引入文件夹下的4个文件（iic.h iic.c queue.h queue.c ） IIC配置（在iic.h开头的I2C  HAL Configuration中注释掉不用的选项，给要使用的选项取消注释）
*		2：IIC硬件初始化      I2C_Init_Config();
*		3: 向队列中添加命令   具体函数见iic.h中的声明
*							 I2C_Add******(); ... ....
*		4:开始执行队列中的命令，执行会有返回值哦，利用这个返回值可以用来实现错误容错
*							  IIC_Start_Next_CMD();
*									
*		
*
* @bug  Just as Master and 7bit adress Mode ,no slave mode   仅仅只考虑到7位地址的主机模式，没有从机模式
* @warning   
* @copyright 
* @attention 
*/


#include "iic.h"



	

 Queue_Mem_Struct IIC_CMD_Queue;
 elemtype IIC_CMD_Current;




void I2C_GPIODeInit(void)//IO设置成默认值
{      
  GPIO_InitTypeDef GPIO_InitStructure;
	
  /* Set GPIO frequency to 50MHz */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* Select Input floating mode */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   
  /* Deinitialize I2Cx SCL Pin */ 
  GPIO_InitStructure.GPIO_Pin = I2C_SCL_GPIO_PIN;
  
  GPIO_Init(I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  /* Deinitialize I2Cx SDA Pin */
  GPIO_InitStructure.GPIO_Pin =I2C_SDA_GPIO_PIN;
  
  GPIO_Init(I2C_SDA_GPIO_PORT, &GPIO_InitStructure); 
}
void I2C_GPIOInit()
{  
  GPIO_InitTypeDef GPIO_InitStructure;
    
  RCC_APB2PeriphClockCmd(I2C_SCL_GPIO_CLK|I2C_SDA_GPIO_CLK,ENABLE);//开启scl  sda时钟
  
	
	/* Enable Pin Remap if PB8 (SCL) and PB9 (SDA) is used for I2C1 复用打开 */
  if ( (I2C_SCL_GPIO_PIN == GPIO_Pin_8) && (I2C_SDA_GPIO_PIN == GPIO_Pin_9))
 {
    /* Enable GPIO Alternative Functions */
    RCC_APB2PeriphClockCmd((RCC_APB2Periph_AFIO),ENABLE);
   
    /* Enable I2C1 pin Remap */
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
  }
  
  /* Set GPIO frequency to 50MHz */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* Select Output open-drain mode */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
                                    
  /* Initialize I2Cx SCL Pin */ 
  GPIO_InitStructure.GPIO_Pin = I2C_SCL_GPIO_PIN;
  GPIO_Init(I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  /* Initialize I2Cx SDA Pin */
  GPIO_InitStructure.GPIO_Pin = I2C_SDA_GPIO_PIN;
  GPIO_Init(I2C_SDA_GPIO_PORT, &GPIO_InitStructure);   
}

///////////////////////////////
///检测总线是否为busy，若是，进行修复
///@retval -1:成功 -0:失败
///////////////////////////////
u8 I2C_CHACK_BUSY_FIX(void)
{
	u8 Time_out=0;
	GPIO_InitTypeDef GPIO_InitStructure;

	while(I2C_GetFlagStatus(I2C, I2C_FLAG_BUSY)&&Time_out<20)
	{
		RCC_APB1PeriphClockCmd(I2C_CLK,DISABLE);//开启I2C时钟
		RCC_APB2PeriphClockCmd(I2C_SCL_GPIO_CLK|I2C_SDA_GPIO_CLK,ENABLE);//开启scl  sda时钟


		/* Set GPIO frequency to 50MHz */
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

		/* Select Output open-drain mode */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
									
		/* Initialize I2Cx SCL Pin */ 
		GPIO_InitStructure.GPIO_Pin = I2C_SCL_GPIO_PIN;
		GPIO_Init(I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

		/* Initialize I2Cx SDA Pin */
		GPIO_InitStructure.GPIO_Pin = I2C_SDA_GPIO_PIN;
		GPIO_Init(I2C_SDA_GPIO_PORT, &GPIO_InitStructure); 
		  
		//模拟方式产生停止信号
		GPIO_ResetBits(I2C_SCL_GPIO_PORT, I2C_SCL_GPIO_PIN);
		GPIO_ResetBits(I2C_SDA_GPIO_PORT, I2C_SDA_GPIO_PIN);
		delay_us(5);
		GPIO_SetBits(I2C_SCL_GPIO_PORT, I2C_SCL_GPIO_PIN);
		delay_us(5);
		GPIO_SetBits(I2C_SDA_GPIO_PORT, I2C_SDA_GPIO_PIN);
		delay_us(5);
		
		I2C_GPIODeInit();//IO设置成默认值
		RCC_APB1PeriphResetCmd(I2C_CLK,ENABLE);//重置clk时钟 防止有错误标志
		RCC_APB1PeriphResetCmd(I2C_CLK,DISABLE);//关闭clk重置
		RCC_APB1PeriphClockCmd(I2C_CLK,ENABLE);//开启I2C时钟
		++Time_out;

	}
	if(Time_out==20)
		return 0;
	return 1;
}
/////////////////////////
///重置I2C总线状态
/////////////////////////
void I2C_Soft_Reset()
{
	I2C->CR1 |= I2C_CR1_SWRST; 
	I2C->CR1 &= ~I2C_CR1_SWRST;
}


////////////////////////////////////
///IIC硬件初始化，使用IIC务必调用
////////////////////////////////////
u8 I2C_Init_Config()
{
	I2C_InitTypeDef I2C_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	I2C_Soft_Reset();//清空I2C相关寄存器  //重要，不能删除
	
////设置成默认值
	I2C->CR1 &= ~I2C_CR1_PE;//I2C失能  I2C_Cmd(I2C,DISABLE); 								//失能 I2C		
	
	I2C_GPIODeInit();//IO设置成默认值
	
	RCC_APB1PeriphResetCmd(I2C_CLK,ENABLE);//重置clk时钟 防止有错误标志 // I2C_DeInit(I2C);  //将IIC端口初始化，否则GPIO不能被操作
	RCC_APB1PeriphResetCmd(I2C_CLK,DISABLE);//关闭clk重置
	RCC_APB1PeriphClockCmd(I2C_CLK,DISABLE);//关闭CLK时钟
	
	//dma默认值
	DMA_DeInit(I2C_DMA_TX_Channel);
	DMA_DeInit(I2C_DMA_RX_Channel);
	
/////初始化	
	//I2C CLK初始化
	RCC_APB1PeriphResetCmd(I2C_CLK,ENABLE);//重置clk时钟 防止有错误标志
	RCC_APB1PeriphResetCmd(I2C_CLK,DISABLE);//关闭clk重置
	RCC_APB1PeriphClockCmd(I2C_CLK,ENABLE);//开启I2C时钟
	
	if(!I2C_CHACK_BUSY_FIX())//检测总线是否被从机拉低（SDA为低）（一般是因为传送过程中出错导致无法一直处于busy状态）
		return 0;
	//IO初始化
	I2C_GPIOInit();
	
	//i2c使能 I2C_Cmd(I2C,ENABLE); 
	I2C->CR1 |= I2C_CR1_PE;
	
	//使能DMA
	I2C->CR2 |= I2C_CR2_DMAEN;
	
	I2C_InitStructure.I2C_ClockSpeed          = I2C_SPEED;                        /* Initialize the I2C_ClockSpeed member */
	I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;                  /* Initialize the I2C_Mode member */
	I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;               /* Initialize the I2C_DutyCycle member */
	I2C_InitStructure.I2C_OwnAddress1         = 0;                             /* Initialize the I2C_OwnAddress1 member */
	I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;                /* Initialize the I2C_Ack member */
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  /* Initialize the I2C_AcknowledgedAddress member */
  
	I2C_Init(I2C,&I2C_InitStructure);//iic初始化
	
	
	
	//DMA初始化
	RCC_AHBPeriphClockCmd(I2C_DMA_CLK,ENABLE);
	/* I2Cx Common Channel Configuration */
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc =  DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte ;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	/* Select I2Cx DR Address register as DMA PeripheralBaseAddress */
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)&(I2C->DR);
	/* Select Memory to Peripheral transfer direction */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_Init(I2C_DMA_TX_Channel,&DMA_InitStructure);
	/* Select Peripheral to Memory transfer direction */
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_Init(I2C_DMA_RX_Channel,&DMA_InitStructure);
	
	
	
	//中断初始化
	NVIC_PriorityGroupConfig(I2C_NVIC_PriorityGroup);
	/* Enable the IRQ channel */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	/* Configure NVIC for I2Cx EVT Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = I2C_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_EVT_PREPRIO;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_EVT_SUBPRIO;
	NVIC_Init(&NVIC_InitStructure);
	I2C->CR2 |= I2C_CR2_ITEVTEN;//                            开启I2C事件中断
 /* Configure NVIC for I2Cx ERR Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = I2C_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_ERR_PREPRIO;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_ERR_SUBPRIO;
    NVIC_Init(&NVIC_InitStructure);
	I2C->CR2 |= I2C_CR2_ITERREN;//                             开启I2C错误中断
	
	I2C->CR2 &= ~I2C_IT_BUF; //使用DMA模式时勿打开 BUF中断

/* Configure NVIC for DMA TX channel interrupt */
      NVIC_InitStructure.NVIC_IRQChannel = I2C_DMA_TX_IRQn ;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_DMATX_PREPRIO;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_DMATX_SUBPRIO;
      NVIC_Init(&NVIC_InitStructure);
	  /* Enable DMA TX Channel TCIT  */
	  I2C_DMA_TX_Channel->CCR |= DMA_IT_TC;  //打开发送完成中断
	   /* Enable DMA TX Channel TEIT  */
	  I2C_DMA_TX_Channel->CCR |= DMA_IT_TE; //打开错误中断
	   /* Enable DMA TX Channel HTIT  */
	  //I2C_DMA_TX_Channel->CCR |= DMA_IT_HT;
/* Configure NVIC for DMA RX channel interrupt */
      NVIC_InitStructure.NVIC_IRQChannel = I2C_DMA_RX_IRQn ;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_DMARX_PREPRIO;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_DMARX_SUBPRIO;
      NVIC_Init(&NVIC_InitStructure);
	  /* Enable DMA TX Channel TCIT  */
	  I2C_DMA_RX_Channel->CCR |= DMA_IT_TC; //打开接收完成中断
	   /* Enable DMA TX Channel TEIT  */
	  I2C_DMA_RX_Channel->CCR |= DMA_IT_TE; //打开接收错误中断
	   /* Enable DMA TX Channel HTIT  */
	  //I2C_DMA_RX_Channel->CCR |= DMA_IT_HT;	  
	IIC_CMD_Queue.State=STATE_READY;//队列状态设置为可以开始下一个任务
	return 1;
}





///////////////////////////
///获取I2C设备的通信健康状况
///@retval -0:健康 -1：出现错误
///////////////////////////
u8 I2C_Get_Health()
{
	if(IIC_CMD_Queue.State==STATE_ERROR)
		return 1;
	else
		return 0;
}



///////////////////////////////
///新的命令出队，IIC_CMD_Queue中最前面的命令到IIC_CMD_Current中
//////////////////////////////
void IIC_Queue_Del()
{
	IIC_CMD_Current=Queue_Del(&IIC_CMD_Queue);//下一个命令出队
}




//////////////////////////////
///开始执行已经出队的当前的命令，命令位于IIC_CMD_Current中
/////////////////////////////
void IIC_Start_CMD()
{
	IIC_CMD_Queue.State = STATE_SEND_ADW;//标志设置为需要发送从机地址+写信号
	IIC_CMD_Queue.Index_Send = 0;//发送、接收计数下标清零
	I2C_AcknowledgeConfig(I2C,ENABLE);	//使能应答
	I2C_GenerateSTART(I2C,ENABLE);		//产生启动信号
}



//////////////////////////////
///开始写队列中下一个一个命令
///如果队列中不存在命令或者硬件正在传送队列中的命令，则无需执行本函数，调用本函数也不会执行任何内容
///@retval -1:成功发送开始信号，可能会执行成功 -2:队列为空，无需开始 -0:发送成功信号失败,I2C总线通信错误
/////////////////////////////
u8 IIC_Start_Next_CMD(void)
{
	static u8 I2C_time_out_count=0;//用来为发送数据时计数，每次进入都保持一个状态，当超过一定数就是出错
	static u8 I2C_Status_Before=0;
	
	if(IIC_CMD_Queue.State==STATE_ERROR)//总线出错了
	{
		return 0;
	}
	if(IIC_CMD_Queue.State!=STATE_READY && I2C_Status_Before==IIC_CMD_Queue.State)//I2C现在的状态和之前的状态相同
	{
		++I2C_time_out_count;
		if(I2C_time_out_count>I2C_TIME_OUT_MAX_TIME)
		{
			I2C_time_out_count=0;
			IIC_CMD_Queue.State=STATE_ERROR;
			return 0;
		}
	}
	else                                 //与之前的值不相同，赋新的值
		I2C_Status_Before=IIC_CMD_Queue.State;
	
	
	if(IIC_CMD_Queue.State==STATE_READY && IIC_CMD_Queue.MemLength>0 )//IIC状态为空闲，并且队列不为空，则发送起始信号
	{
		IIC_Queue_Del();
		IIC_Start_CMD();
		return 1;
	}
	else if(IIC_CMD_Queue.MemLength==0)//队列为空
		return 2;
	return 1;
}




///////////////////////////////////
///添加读取多个字节命令
///注意：此时没有开始执行命令，只是放到队列中了，IIC_Queue_Del();IIC_Start_Next_CMD();语句会执行队列中下一个命令
///////////////////////////////////
void I2C_AddCMD_Read_Bytes(u8 device_addr,u8 register_addr, u8* data_read, u8 num)//参数：设备地址，寄存器地址，读取的数据存放的地址，需要读取数据的个数
{
	elemtype IIC_CMD_Temp;
	if(num>1)
		IIC_CMD_Temp.cmdType=I2C_READ_BYTES;
	else
		IIC_CMD_Temp.cmdType=I2C_READ_BYTE;
	IIC_CMD_Temp.inDataLen=num;
	IIC_CMD_Temp.outDataLen=1;
	IIC_CMD_Temp.pDataIn=data_read;
	IIC_CMD_Temp.DataOut[0]=register_addr;
	IIC_CMD_Temp.slaveAddr=device_addr;
	Queue_En(&IIC_CMD_Queue,IIC_CMD_Temp);//新的命令入队 
}







///////////////////////////////////
///添加写入多个字节命令
///注意：此时没有开始执行命令，只是放到队列中了，IIC_Queue_Del();IIC_Start_CMD();语句会执行队列中下一个命令
///////////////////////////////////
void I2C_AddCMD_Write_Bytes(u8 device_addr,u8 register_addr, u8* data_write, u8 num)//参数：设备地址，寄存器地址，读取的数据存放的地址，需要读取数据的个数
{
	elemtype IIC_CMD_Temp;
	u8 i;
	if(num>1)
	{
		IIC_CMD_Temp.cmdType=I2C_WRITE_BYTES;
	}
	else
		IIC_CMD_Temp.cmdType=I2C_WRITE_BYTE;
	IIC_CMD_Temp.inDataLen=0;
	IIC_CMD_Temp.outDataLen=num+1;
	IIC_CMD_Temp.pDataIn=0;
	IIC_CMD_Temp.DataOut[0]=register_addr;
	for(i=1;i<=num;++i)
	{
		IIC_CMD_Temp.DataOut[i]=(*data_write);
		++data_write;
	}
	IIC_CMD_Temp.slaveAddr=device_addr;
	Queue_En(&IIC_CMD_Queue,IIC_CMD_Temp);//新的命令入队  
}


///////////////////////////////////
///添加如指令命令 （无寄存器）
///注意：此时没有开始执行命令，只是放到队列中了，IIC_Queue_Del();IIC_Start_CMD();语句会执行队列中下一个命令
///////////////////////////////////
void I2C_AddCMD_Write_CMD_Bytes(u8 device_addr, u8* CMD_Write, u8 num)//参数：设备地址，命令，需要写入的命令的个数
{
	elemtype IIC_CMD_Temp;
	u8 i;
	IIC_CMD_Temp.cmdType=I2C_WRITE_CMD;
	IIC_CMD_Temp.inDataLen=0;
	IIC_CMD_Temp.outDataLen=num;
	IIC_CMD_Temp.pDataIn=0;
	for(i=0;i<num;++i)
		IIC_CMD_Temp.DataOut[i]=(*CMD_Write)++;
	IIC_CMD_Temp.slaveAddr=device_addr;
	Queue_En(&IIC_CMD_Queue,IIC_CMD_Temp);//新的命令入队  
}



///////////////////////////////////
///添加如指令命令 （无寄存器）
///注意：此时没有开始执行命令，只是放到队列中了，IIC_Queue_Del();IIC_Start_CMD();语句会执行队列中下一个命令
///////////////////////////////////
void I2C_AddCMD_Write_CMD_Byte(u8 device_addr, u8 CMD_Write)//参数：设备地址，命令
{
	elemtype IIC_CMD_Temp;
	IIC_CMD_Temp.cmdType=I2C_WRITE_CMD;
	IIC_CMD_Temp.inDataLen=0;
	IIC_CMD_Temp.outDataLen=1;
	IIC_CMD_Temp.pDataIn=0;
	IIC_CMD_Temp.DataOut[0]=CMD_Write;
	IIC_CMD_Temp.slaveAddr=device_addr;
	Queue_En(&IIC_CMD_Queue,IIC_CMD_Temp);//新的命令入队  
}




/////////////////////////////////////
///IIC中断事件处理函数
/////////////////////////////////////
void I2C_EV_IRQHandler(void)
{
	uint32_t I2C_Status = I2C_GetLastEvent(I2C);
	DMA_InitTypeDef DMA_InitStructure;
	#ifdef DEBUG
		printf("EV_IRQ\n");
	#endif
	switch(I2C_Status)//查询中断事件类型
	{
		case I2C_EVENT_MASTER_MODE_SELECT: //EV5   //SB、BUSY、MSL位置位	 
			if(IIC_CMD_Queue.State==STATE_SEND_ADW)//发送从机地址+写信号
			{	
				I2C->CR2 &= ~I2C_IT_BUF; //使用DMA,关闭BUF中断
				I2C->CR2 |= I2C_CR2_DMAEN; //使能IIC DMA传输
				
				I2C_Send7bitAddress(I2C,IIC_CMD_Current.slaveAddr,I2C_Direction_Transmitter);
				IIC_CMD_Queue.State = STATE_SEND_DATA;//状态设置为需要发送数据（发送寄存器地址）
					/* I2Cx Common Channel Configuration */
				
				if(IIC_CMD_Current.cmdType>=I2C_WRITE_BYTE)//写模式
				{
					DMA_InitStructure.DMA_BufferSize = IIC_CMD_Current.outDataLen;
				}
				else                                        //读模式 设置长度为1 因为发送一个字节后需要再次发送起始信号
				{
					DMA_InitStructure.DMA_BufferSize = 1;
				}
				
				DMA_InitStructure.DMA_PeripheralInc =  DMA_PeripheralInc_Disable;
				DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
				DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
				DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte ;
				DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
				DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
				DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
				DMA_InitStructure.DMA_MemoryBaseAddr=(uint32_t)&IIC_CMD_Current.DataOut[0];
				DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralDST;
				DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)&(I2C->DR);
				
				DMA_Init(I2C_DMA_TX_Channel,&DMA_InitStructure);

				DMA_Cmd(I2C_DMA_TX_Channel, ENABLE);
			}
			else if(IIC_CMD_Queue.State==STATE_SEND_ADR)//发送从机地址+读信号
			{
				
				I2C_Send7bitAddress(I2C,IIC_CMD_Current.slaveAddr,I2C_Direction_Receiver);
				IIC_CMD_Queue.State = STATE_REVEIVE_DATA;//状态设置为需要发送数据（发送寄存器地址）
				
				if( IIC_CMD_Current.cmdType==I2C_READ_BYTE)//接收一个字节，不能使用DMA，使用中断的方式
				{
					I2C->CR2 |= I2C_IT_BUF; //接收一个字节，打开 BUF中断，不使用DMA
					I2C->CR2 &= ~I2C_CR2_DMAEN; //失能IIC DMA传输
				}
				else //接收的字节数大于1，使用DMA
				{
					I2C->CR2|=I2C_CR2_LAST;//使能接收最后一位时发送NACK
					I2C->CR2 &= ~I2C_IT_BUF; //使用DMA,关闭BUF中断
					I2C->CR2 |= I2C_CR2_DMAEN; //使能IIC DMA传输
					
					DMA_InitStructure.DMA_BufferSize = IIC_CMD_Current.inDataLen;
					
					DMA_InitStructure.DMA_PeripheralInc =  DMA_PeripheralInc_Disable;
					DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
					DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
					DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte ;
					DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
					DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
					DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
					DMA_InitStructure.DMA_MemoryBaseAddr=(uint32_t)IIC_CMD_Current.pDataIn;
					DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralSRC;
					DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)&(I2C->DR);
					
					DMA_Init(I2C_DMA_RX_Channel,&DMA_InitStructure);
					DMA_Cmd(I2C_DMA_RX_Channel, ENABLE);
					
				}
			}
			break;
		
		
//		case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED://EV6	//已: I2C_Send7bitAddress(W)   应: I2C_SendData()
//			break;
		
		
		case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED://EV6			//已: I2C_Send7bitAddress(R)		
			if(IIC_CMD_Current.cmdType==I2C_READ_BYTE)//只接收一个字节
			{
				I2C_AcknowledgeConfig(I2C,DISABLE);//关应答
				I2C_GenerateSTOP(I2C,ENABLE);//产生停止信号
			}
			break;
		
		
		case I2C_EVENT_MASTER_BYTE_RECEIVED://EV7   //数据到达，接收数据
			if(IIC_CMD_Current.cmdType==I2C_READ_BYTE)//只接收一个字节
			{
				(*IIC_CMD_Current.pDataIn) = I2C_ReceiveData(I2C); //保存接收到的数据
			
				I2C->CR2 &= ~I2C_IT_BUF; //使用DMA,关闭BUF中断
			
				//检查队列是否为空
				if(IIC_CMD_Queue.MemLength>0)//队列不为空
				{
					IIC_CMD_Current=Queue_Del(&IIC_CMD_Queue);//下一个命令出队
					//都要先发送从机地址+写信号
					IIC_CMD_Queue.State = STATE_SEND_ADW;//标志设置为需要发送从机地址+写信号
					IIC_CMD_Queue.Index_Send = 0;//发送、接收计数下标清零
					I2C_AcknowledgeConfig(I2C,ENABLE);	//使能应答
					I2C_GenerateSTART(I2C,ENABLE);		//产生启动信号
				}
				else//队列为空，
				{
					IIC_CMD_Queue.State=STATE_READY;//将状态设置为 准备好发送 模式
				}
			}
			break;
		
		
		case I2C_EVENT_MASTER_BYTE_TRANSMITTED:	//EV8_2					//已: I2C_SendData() 
			if(0==DMA_GetCurrDataCounter(I2C_DMA_TX_Channel))//检查是否发送完毕
			{
				#ifdef DEBUG
					printf("\nDMA_translate_Complet\n");
				#endif
				DMA_Cmd(I2C_DMA_TX_Channel, DISABLE);//关DMA
				
				if(IIC_CMD_Current.cmdType >= I2C_WRITE_BYTE)//处于写模式 单字节或者多字节
					{
						//已经写完，发送停止信号
						I2C_GenerateSTOP(I2C,ENABLE);//产生停止信号
						//判断队列是否为空
						if(IIC_CMD_Queue.MemLength>0)//队列不为空
						{
							IIC_CMD_Current=Queue_Del(&IIC_CMD_Queue);//下一个命令出队
							
							 //都要先发送从机地址+写信号
						
							IIC_CMD_Queue.State = STATE_SEND_ADW;//标志设置为需要发送从机地址+写信号
							IIC_CMD_Queue.Index_Send = 0;//发送、接收计数下标清零
							I2C_AcknowledgeConfig(I2C,ENABLE);	//使能应答
							I2C_GenerateSTART(I2C,ENABLE);		//产生启动信号
						}
						else//队列为空，
						{
							IIC_CMD_Queue.State=STATE_READY;//将状态设置为 准备好发送 模式
						}
					}
					else//处于读模式
					{
						IIC_CMD_Queue.State = STATE_SEND_ADR;//状态设置为 发送从机地址+读 
						I2C_GenerateSTART(I2C,ENABLE);//起始信号
					}
			}
			break;
		
//		case I2C_EVENT_MASTER_BYTE_TRANSMITTING://EV8   /* TRA, BUSY, MSL, TXE flags */

//			break;
		
	}
	if((I2C_Status&0x00010000)==0)//从机模式
	{
		I2C_Soft_Reset();
		IIC_CMD_Queue.State = STATE_ERROR;//状态设置为错误状态
	}
}



#if defined(USE_ERROR_CALLBACK) 
/* Call Error UserCallback */  
void I2C_ERR_UserCallback(u16 Error)
{
		/* If Bus error occurred ---------------------------------------------------*/
	if((Error&I2C_ERR_BERR)!=0)
	{
		I2C_BERR_UserCallback();
	}
          
    /* If Arbitration Loss error occurred --------------------------------------*/
	if((Error&I2C_ERR_ARLO)!=0)
	{
		I2C_ARLO_UserCallback();
	}
    
    /* If Overrun error occurred -----------------------------------------------*/
	if((Error&I2C_ERR_OVR)!=0)
	{
		I2C_OVR_UserCallback();
    }
        
    /* If Acknowledge Failure error occurred -----------------------------------*/
	if((Error&I2C_ERR_AF)!=0)
    {
		I2C_AF_UserCallback();      
    } 
	
	if((Error&I2C_ERR_TIMEOUT)!=0)
	{
		I2C_TIME_OUT_UserCallback();
	}
}
#endif /* USE_SINGLE_ERROR_CALLBACK */



////////////////////////////////////////////////
///IIC错误中断处理函数
////////////////////////////////////////////////
uint32_t I2C_ER_IRQHandler(void)
{
	u16 Error=I2C->SR1 & ((uint16_t)0x0F00) ; /*!< I2C errors Mask  *///获取错误信息
	IIC_CMD_Queue.State = STATE_ERROR;//状态设置为错误状态
	I2C->SR1 = ~((uint16_t)0x0F00);//清除错误信息
	#ifdef DEBUG
			printf("ER_IRQ\n");
		#endif
	/* If Bus error occurred ---------------------------------------------------*/
	if((Error&I2C_ERR_BERR)!=0)
	{
		#ifdef DEBUG
			printf("BERR\n");
		#endif
			/* Generate I2C software reset in order to release SDA and SCL lines */
			I2C->CR1 |= I2C_CR1_SWRST; 
			I2C->CR1 &= ~I2C_CR1_SWRST;
	}
          
    /* If Arbitration Loss error occurred --------------------------------------*/
	if((Error&I2C_ERR_ARLO)!=0)
	{
		#ifdef DEBUG
			printf("ARLO\n");
		#endif
			/* Generate I2C software reset in order to release SDA and SCL lines */
			I2C->CR1 |= I2C_CR1_SWRST; 
			I2C->CR1 &= ~I2C_CR1_SWRST; 
	}
    
    /* If Overrun error occurred -----------------------------------------------*/
	if((Error&I2C_ERR_OVR)!=0)
	{
		#ifdef DEBUG
			printf("OVR\n");
		#endif
		  /* No I2C software reset is performed here in order to allow user to get back
		  the last data received correctly */	    
    }
        
    /* If Acknowledge Failure error occurred -----------------------------------*/
	if((Error&I2C_ERR_AF)!=0)
    {
		#ifdef DEBUG
			printf("AF\n");
		#endif
		  /* No I2C software reset is performed here in order to allow user to recover 
		  communication */ 
    }   
	    /* USE_SINGLE_ERROR_CALLBACK is defined in cpal_conf.h file */
	#if defined(USE_ERROR_CALLBACK)  
		/* Call Error UserCallback */  
		I2C_Error_UserCallback(Error);
		I2C_ERR_UserCallback(Error);
	#endif /* USE_SINGLE_ERROR_CALLBACK */
	
	
	
	I2C_Soft_Reset();//软件复位，清除I2C所有寄存器中的值
	
	
//	IIC_CMD_Queue.State = STATE_READY;//状态设置为正常
	return 0;
}
void I2C_DMA_TX_IRQHandler()
{
	if((uint32_t)( I2C_DMA->ISR & I2C_DMA_TX_TC_FLAG )!=0)//传输完成
	{
		
		
		
	}
	else if((uint32_t)( I2C_DMA->ISR & I2C_DMA_TX_HT_FLAG )!=0)//传输一半
	{
		
	}
	else if((uint32_t)( I2C_DMA->ISR & I2C_DMA_TX_TE_FLAG )!=0)//传输发生错误
	{
	}
	I2C_DMA->IFCR=I2C_DMA_TX_TC_FLAG | I2C_DMA_TX_HT_FLAG | I2C_DMA_TX_TE_FLAG;//清除中断标志
	
}
void I2C_DMA_RX_IRQHandler()
{
	if((uint32_t)( I2C_DMA->ISR & I2C_DMA_RX_TC_FLAG )!=0)//传输完成
	{
	//	if(!DMA_GetCurrDataCounter(I2C_DMA_RX_Channel))//检查是否接收完毕   //此处不用判断 ,判断会出错， 为查明情况
	//	printf("\n\n\n....%d...\n\n\n",DMA_GetCurrDataCounter(I2C_DMA_RX_Channel));
		{
		#ifdef DEBUG
			printf("\nDMA reveive Complet\n");
		#endif
			DMA_Cmd(I2C_DMA_RX_Channel, DISABLE);//关DMA
			I2C_GenerateSTOP(I2C,ENABLE);		//产生停止信号  
								//检查队列是否为空
			if(IIC_CMD_Queue.MemLength>0)//队列不为空
				{
					IIC_CMD_Current=Queue_Del(&IIC_CMD_Queue);//下一个命令出队
					//都要先发送从机地址+写信号
					
					IIC_CMD_Queue.State = STATE_SEND_ADW;//标志设置为需要发送从机地址+写信号
					IIC_CMD_Queue.Index_Send = 0;//发送、接收计数下标清零
					I2C_AcknowledgeConfig(I2C,ENABLE);	//使能应答
					I2C_GenerateSTART(I2C,ENABLE);		//产生启动信号	
				}
				else//队列为空，
				{
					IIC_CMD_Queue.State=STATE_READY;//将状态设置为 准备好发送 模式
				}
		}
	}
	else if((uint32_t)( I2C_DMA->ISR & I2C_DMA_RX_HT_FLAG )!=0)//传输一半
	{
		
	}
	else if((uint32_t)( I2C_DMA->ISR & I2C_DMA_RX_TE_FLAG )!=0)//传输发生错误
	{
		
	}
	I2C_DMA->IFCR=I2C_DMA_RX_TC_FLAG | I2C_DMA_RX_HT_FLAG | I2C_DMA_RX_TE_FLAG;//清除中断标志
}









//////////////////////////////////
/////Arbitration lost (master mode)仲裁丢失   IIC总线错误用户回调函数
/////出现此错误之后总线已经重置
////////////////////////////////////
//void I2C_ARLO_UserCallback(void)
//{
//	//仲裁丢失后会自动切回从模式，在这里进行重新初始化
//	printf("ARLO\n");
//}



/////////////////////////////////////
/////Overrun/Underrun 过载、欠载错误   IIC总线错误用户回调函数
/////出现此错误之后总线未重置
////////////////////////////////////
//void I2C_OVR_UserCallback(void)
//{
////	printf("OVR\n");
//}


////////////////////////////////////
/////(Bus error 总线出错         IIC总线错误用户回调函数
/////出现此错误之后总线已经重置
////////////////////////////////////
//void I2C_BERR_UserCallback(void)
//{
//	//硬件连接在读写过程中断开也会产生该错误
//	//在本函数中或者在无应答回调函数I2C_AF_UserCallback()中进行处理，防止继续进一步对已经失去连接了的设备进行读写
//	printf("BERR\n");
//}


/////////////////////////////////
/////Acknowledge failure User Callback Function 应答失败 IIC总线错误用户回调函数
/////出现此错误之后总线未重置
/////////////////////////////////
//void I2C_AF_UserCallback(void)
//{
//	//无应答时，应该重新发送数据或者停止再次访问设备的动作
//	printf("AF\n");
//}

////////////////////////////////////
/////(Time out error 超时         IIC超时错误用户回调函数
////////////////////////////////////
//void I2C_TIME_OUT_UserCallback(void)
//{
//	printf("Time_out");
//}

//void I2C_Error_UserCallback(u16 Error)
//{
//	
//}

