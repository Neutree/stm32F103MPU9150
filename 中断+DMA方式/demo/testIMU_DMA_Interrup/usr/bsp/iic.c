/**
* @file iic.c
* @author 1208077207@qq.com
* @version v0.1
* @date 2015-10-4
* @pre First initialize the SystemClock eg:72M 
* @brief  i2c HAL lib 
*Introduction:
*		中断方式操作MPU6050，操作命令都由一个循环队列管理，
*		IIC中断优先级最好是最高，可以在 iic.h 宏定义中设置
*		此处设置的中断优先级为：分组2 ，IIC事件中断抢占优先级为2，响应优先级为0， IIC错误中断抢占式优先级为0，响应优先级为0,DMA抢占优先级为1，相应优先级为0
*		
*		
*Change Log:
*		2015-10-4  7位地址主机读写基本实现
*		
*How To Use It：
*		1：工程中引入文件夹下的4个文件（iic.h iic.c queue.h queue.c ） IIC配置（在iic.h开头的I2C  HAL Configuration中注释掉不用的选项，给要使用的选项取消注释）
*		2：IIC硬件初始化      I2C_Init_Config();
*		3: 向队列中添加命令   具体函数见iic.h中的声明
*							  ... ....
*		4:在IIC状态为空闲的时候并且队列中的命令数量大于0的时候就可以开始执行队列中的命令了 (注意：一定要在IIC为空闲并且命令队列不为空的时候才能调用开始执行命令语句，否则可能产生时序错误)
*							  if ( (IIC_CMD_Queue.State==STATE_READY）  &&  (IIC_CMD_Queue.MemLength>0) )
*									IIC_Start_Next_CMD();
*									
*		
*
* @bug  Just as Master and 7bit adress Mode ,no slave mode   仅仅只考虑到7位地址的主机模式，没有从机模式
* @warning   
* @copyright 
* @attention 
*/


#include "iic.h"


#ifdef DEBUG
	#include "usart.h"
#endif
	

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

////////////////////////////////////
///IIC硬件初始化，使用IIC务必调用
////////////////////////////////////
void I2C_Init_Config()
{
	I2C_InitTypeDef I2C_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
////设置成默认值
	I2C->CR1 &= ~I2C_CR1_PE;//I2C失能  I2C_Cmd(I2C,DISABLE); 								//失能 I2C		
	I2C_GPIODeInit();//IO设置成默认值
	RCC_APB1PeriphResetCmd(I2C_CLK,ENABLE);//重置clk时钟 防止有错误标志
	RCC_APB1PeriphResetCmd(I2C_CLK,DISABLE);//关闭clk重置
	RCC_APB1PeriphClockCmd(I2C_CLK,DISABLE);//关闭CLK时钟
	
	//dma默认值
	DMA_DeInit(I2C_DMA_TX_Channel);
	DMA_DeInit(I2C_DMA_RX_Channel);
	
	//I2C CLK初始化
	RCC_APB1PeriphResetCmd(I2C_CLK,ENABLE);//重置clk时钟 防止有错误标志
	RCC_APB1PeriphResetCmd(I2C_CLK,DISABLE);//关闭clk重置
	RCC_APB1PeriphClockCmd(I2C_CLK,ENABLE);//开启I2C时钟
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
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
/////////////////////////////
void IIC_Start_Next_CMD(void)
{
	if(IIC_CMD_Queue.State==STATE_READY && IIC_CMD_Queue.MemLength>0 )//IIC状态为空闲，并且队列不为空，则发送起始信号
	{
		IIC_Queue_Del();
		IIC_Start_CMD();
	}
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
					if(IIC_CMD_Current.cmdType == I2C_READ_DIRECT)//直接读取模式，状态设置为读取
					{
						IIC_CMD_Queue.State = STATE_REVEIVE_DATA;//状态设置为接收数据模式
					}
					else //其它的，都要先发送从机地址+写信号
					{
						IIC_CMD_Queue.State = STATE_SEND_ADW;//标志设置为需要发送从机地址+写信号
						IIC_CMD_Queue.Index_Send = 0;//发送、接收计数下标清零
						I2C_AcknowledgeConfig(I2C,ENABLE);	//使能应答
						I2C_GenerateSTART(I2C,ENABLE);		//产生启动信号
					}
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
//		
		
//		case I2C_EVENT_MASTER_BYTE_TRANSMITTING://EV8   /* TRA, BUSY, MSL, TXE flags */

//			break;
	}
}



////////////////////////////////////////////////
///IIC错误中断处理函数
////////////////////////////////////////////////
uint32_t I2C_ER_IRQHandler(void)
{
	u16 Error=I2C->SR1 & ((uint16_t)0x0F00) ; /*!< I2C errors Mask  *///获取错误信息
	IIC_CMD_Queue.State = STATE_ERROR;//状态设置为错误状态
	I2C->SR1 = ~((uint16_t)0x0F00);//清除错误信息
	
	/* If Bus error occurred ---------------------------------------------------*/
	if((Error&I2C_ERR_BERR)!=0)
	{
		#ifdef DEBUG
			printf("BERR\n");
		#endif
			/* Generate I2C software reset in order to release SDA and SCL lines */
			I2C->CR1 |= I2C_CR1_SWRST; 
			I2C->CR1 &= ~I2C_CR1_SWRST;
//		#ifdef USE_MULTIPLE_ERROR_CALLBACK
//			/* Call Bus Error UserCallback */
//		I2C_BERR_UserCallback();
//		#endif /* USE_MULTIPLE_ERROR_CALLBACK */
		//以上条件在打开后会出错，暂时未找到问题所在
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
//		#ifdef USE_MULTIPLE_ERROR_CALLBACK    
//			/* Call Arbitration Lost UserCallback */ 
//			I2C_ARLO_UserCallback();  
//		#endif /* USE_MULTIPLE_ERROR_CALLBACK */  
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
		I2C_ERR_UserCallback(Error);
	#endif /* USE_SINGLE_ERROR_CALLBACK */
	
	IIC_CMD_Queue.State = STATE_READY;//状态设置为正常
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
}





////////////////////////////////
///Arbitration lost (master mode)仲裁丢失   IIC总线错误用户回调函数
//////////////////////////////////
void I2C_ARLO_UserCallback(void)
{
	
}



///////////////////////////////////
///Overrun/Underrun 过载、欠载错误   IIC总线错误用户回调函数
//////////////////////////////////
void I2C_OVR_UserCallback(void)
{
	
}


//////////////////////////////////
///(Bus error 总线出错         IIC总线错误用户回调函数
//////////////////////////////////
void I2C_BERR_UserCallback(void)
{
	//硬件连接在读写过程中断开也会产生该错误
	//在本函数中或者在无应答回调函数I2C_AF_UserCallback()中进行处理，防止继续进一步对已经失去连接了的设备进行读写
}


///////////////////////////////
///Acknowledge failure User Callback Function 应答失败 IIC总线错误用户回调函数
///////////////////////////////
void I2C_AF_UserCallback(void)
{
	//无应答时，应该重新发送数据或者停止再次访问设备的动作
	
	
}

