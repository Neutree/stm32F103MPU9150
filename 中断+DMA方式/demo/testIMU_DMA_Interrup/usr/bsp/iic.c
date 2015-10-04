/**
* @file iic.c
* @author 1208077207@qq.com
* @version v0.1
* @date 2015-10-4
* @pre First initialize the SystemClock eg:72M 
* @brief  i2c HAL lib 
*Introduction:
*		�жϷ�ʽ����MPU6050�����������һ��ѭ�����й���
*		IIC�ж����ȼ��������ߣ������� iic.h �궨��������
*		�˴����õ��ж����ȼ�Ϊ������2 ��IIC�¼��ж���ռ���ȼ�Ϊ2����Ӧ���ȼ�Ϊ0�� IIC�����ж���ռʽ���ȼ�Ϊ0����Ӧ���ȼ�Ϊ0,DMA��ռ���ȼ�Ϊ1����Ӧ���ȼ�Ϊ0
*		
*		
*Change Log:
*		2015-10-4  7λ��ַ������д����ʵ��
*		
*How To Use It��
*		1�������������ļ����µ�4���ļ���iic.h iic.c queue.h queue.c �� IIC���ã���iic.h��ͷ��I2C  HAL Configuration��ע�͵����õ�ѡ���Ҫʹ�õ�ѡ��ȡ��ע�ͣ�
*		2��IICӲ����ʼ��      I2C_Init_Config();
*		3: ��������������   ���庯����iic.h�е�����
*							  ... ....
*		4:��IIC״̬Ϊ���е�ʱ���Ҷ����е�������������0��ʱ��Ϳ��Կ�ʼִ�ж����е������� (ע�⣺һ��Ҫ��IICΪ���в���������в�Ϊ�յ�ʱ����ܵ��ÿ�ʼִ��������䣬������ܲ���ʱ�����)
*							  if ( (IIC_CMD_Queue.State==STATE_READY��  &&  (IIC_CMD_Queue.MemLength>0) )
*									IIC_Start_Next_CMD();
*									
*		
*
* @bug  Just as Master and 7bit adress Mode ,no slave mode   ����ֻ���ǵ�7λ��ַ������ģʽ��û�дӻ�ģʽ
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




void I2C_GPIODeInit(void)//IO���ó�Ĭ��ֵ
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
    
  RCC_APB2PeriphClockCmd(I2C_SCL_GPIO_CLK|I2C_SDA_GPIO_CLK,ENABLE);//����scl  sdaʱ��
  
	
	/* Enable Pin Remap if PB8 (SCL) and PB9 (SDA) is used for I2C1 ���ô� */
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
///IICӲ����ʼ����ʹ��IIC��ص���
////////////////////////////////////
void I2C_Init_Config()
{
	I2C_InitTypeDef I2C_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
////���ó�Ĭ��ֵ
	I2C->CR1 &= ~I2C_CR1_PE;//I2Cʧ��  I2C_Cmd(I2C,DISABLE); 								//ʧ�� I2C		
	I2C_GPIODeInit();//IO���ó�Ĭ��ֵ
	RCC_APB1PeriphResetCmd(I2C_CLK,ENABLE);//����clkʱ�� ��ֹ�д����־
	RCC_APB1PeriphResetCmd(I2C_CLK,DISABLE);//�ر�clk����
	RCC_APB1PeriphClockCmd(I2C_CLK,DISABLE);//�ر�CLKʱ��
	
	//dmaĬ��ֵ
	DMA_DeInit(I2C_DMA_TX_Channel);
	DMA_DeInit(I2C_DMA_RX_Channel);
	
	//I2C CLK��ʼ��
	RCC_APB1PeriphResetCmd(I2C_CLK,ENABLE);//����clkʱ�� ��ֹ�д����־
	RCC_APB1PeriphResetCmd(I2C_CLK,DISABLE);//�ر�clk����
	RCC_APB1PeriphClockCmd(I2C_CLK,ENABLE);//����I2Cʱ��
	//IO��ʼ��
	I2C_GPIOInit();
	
	//i2cʹ�� I2C_Cmd(I2C,ENABLE); 
	I2C->CR1 |= I2C_CR1_PE;
	
	//ʹ��DMA
	I2C->CR2 |= I2C_CR2_DMAEN;
	
	I2C_InitStructure.I2C_ClockSpeed          = I2C_SPEED;                        /* Initialize the I2C_ClockSpeed member */
	I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;                  /* Initialize the I2C_Mode member */
	I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;               /* Initialize the I2C_DutyCycle member */
	I2C_InitStructure.I2C_OwnAddress1         = 0;                             /* Initialize the I2C_OwnAddress1 member */
	I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;                /* Initialize the I2C_Ack member */
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  /* Initialize the I2C_AcknowledgedAddress member */
  
	I2C_Init(I2C,&I2C_InitStructure);//iic��ʼ��
	
	
	
	//DMA��ʼ��
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
	
	
	
	//�жϳ�ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	/* Enable the IRQ channel */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	/* Configure NVIC for I2Cx EVT Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = I2C_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_EVT_PREPRIO;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_EVT_SUBPRIO;
	NVIC_Init(&NVIC_InitStructure);
	I2C->CR2 |= I2C_CR2_ITEVTEN;//                            ����I2C�¼��ж�
 /* Configure NVIC for I2Cx ERR Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = I2C_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_ERR_PREPRIO;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_ERR_SUBPRIO;
    NVIC_Init(&NVIC_InitStructure);
	I2C->CR2 |= I2C_CR2_ITERREN;//                             ����I2C�����ж�
	
	I2C->CR2 &= ~I2C_IT_BUF; //ʹ��DMAģʽʱ��� BUF�ж�

/* Configure NVIC for DMA TX channel interrupt */
      NVIC_InitStructure.NVIC_IRQChannel = I2C_DMA_TX_IRQn ;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_DMATX_PREPRIO;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_DMATX_SUBPRIO;
      NVIC_Init(&NVIC_InitStructure);
	  /* Enable DMA TX Channel TCIT  */
	  I2C_DMA_TX_Channel->CCR |= DMA_IT_TC;  //�򿪷�������ж�
	   /* Enable DMA TX Channel TEIT  */
	  I2C_DMA_TX_Channel->CCR |= DMA_IT_TE; //�򿪴����ж�
	   /* Enable DMA TX Channel HTIT  */
	  //I2C_DMA_TX_Channel->CCR |= DMA_IT_HT;
/* Configure NVIC for DMA RX channel interrupt */
      NVIC_InitStructure.NVIC_IRQChannel = I2C_DMA_RX_IRQn ;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_DMARX_PREPRIO;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_DMARX_SUBPRIO;
      NVIC_Init(&NVIC_InitStructure);
	  /* Enable DMA TX Channel TCIT  */
	  I2C_DMA_RX_Channel->CCR |= DMA_IT_TC; //�򿪽�������ж�
	   /* Enable DMA TX Channel TEIT  */
	  I2C_DMA_RX_Channel->CCR |= DMA_IT_TE; //�򿪽��մ����ж�
	   /* Enable DMA TX Channel HTIT  */
	  //I2C_DMA_RX_Channel->CCR |= DMA_IT_HT;	  

	IIC_CMD_Queue.State=STATE_READY;//����״̬����Ϊ���Կ�ʼ��һ������
}


///////////////////////////////
///�µ�������ӣ�IIC_CMD_Queue����ǰ������IIC_CMD_Current��
//////////////////////////////
void IIC_Queue_Del()
{
	IIC_CMD_Current=Queue_Del(&IIC_CMD_Queue);//��һ���������
}




//////////////////////////////
///��ʼִ���Ѿ����ӵĵ�ǰ���������λ��IIC_CMD_Current��
/////////////////////////////
void IIC_Start_CMD()
{
	IIC_CMD_Queue.State = STATE_SEND_ADW;//��־����Ϊ��Ҫ���ʹӻ���ַ+д�ź�
	IIC_CMD_Queue.Index_Send = 0;//���͡����ռ����±�����
	I2C_AcknowledgeConfig(I2C,ENABLE);	//ʹ��Ӧ��
	I2C_GenerateSTART(I2C,ENABLE);		//���������ź�
}



//////////////////////////////
///��ʼд��������һ��һ������
/////////////////////////////
void IIC_Start_Next_CMD(void)
{
	if(IIC_CMD_Queue.State==STATE_READY && IIC_CMD_Queue.MemLength>0 )//IIC״̬Ϊ���У����Ҷ��в�Ϊ�գ�������ʼ�ź�
	{
		IIC_Queue_Del();
		IIC_Start_CMD();
	}
}




///////////////////////////////////
///��Ӷ�ȡ����ֽ�����
///ע�⣺��ʱû�п�ʼִ�����ֻ�Ƿŵ��������ˣ�IIC_Queue_Del();IIC_Start_Next_CMD();����ִ�ж�������һ������
///////////////////////////////////
void I2C_AddCMD_Read_Bytes(u8 device_addr,u8 register_addr, u8* data_read, u8 num)//�������豸��ַ���Ĵ�����ַ����ȡ�����ݴ�ŵĵ�ַ����Ҫ��ȡ���ݵĸ���
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
	Queue_En(&IIC_CMD_Queue,IIC_CMD_Temp);//�µ�������� 
}







///////////////////////////////////
///���д�����ֽ�����
///ע�⣺��ʱû�п�ʼִ�����ֻ�Ƿŵ��������ˣ�IIC_Queue_Del();IIC_Start_CMD();����ִ�ж�������һ������
///////////////////////////////////
void I2C_AddCMD_Write_Bytes(u8 device_addr,u8 register_addr, u8* data_write, u8 num)//�������豸��ַ���Ĵ�����ַ����ȡ�����ݴ�ŵĵ�ַ����Ҫ��ȡ���ݵĸ���
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
	Queue_En(&IIC_CMD_Queue,IIC_CMD_Temp);//�µ��������  
}


///////////////////////////////////
///�����ָ������ ���޼Ĵ�����
///ע�⣺��ʱû�п�ʼִ�����ֻ�Ƿŵ��������ˣ�IIC_Queue_Del();IIC_Start_CMD();����ִ�ж�������һ������
///////////////////////////////////
void I2C_AddCMD_Write_CMD_Bytes(u8 device_addr, u8* CMD_Write, u8 num)//�������豸��ַ�������Ҫд�������ĸ���
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
	Queue_En(&IIC_CMD_Queue,IIC_CMD_Temp);//�µ��������  
}



///////////////////////////////////
///�����ָ������ ���޼Ĵ�����
///ע�⣺��ʱû�п�ʼִ�����ֻ�Ƿŵ��������ˣ�IIC_Queue_Del();IIC_Start_CMD();����ִ�ж�������һ������
///////////////////////////////////
void I2C_AddCMD_Write_CMD_Byte(u8 device_addr, u8 CMD_Write)//�������豸��ַ������
{
	elemtype IIC_CMD_Temp;
	IIC_CMD_Temp.cmdType=I2C_WRITE_CMD;
	IIC_CMD_Temp.inDataLen=0;
	IIC_CMD_Temp.outDataLen=1;
	IIC_CMD_Temp.pDataIn=0;
	IIC_CMD_Temp.DataOut[0]=CMD_Write;
	IIC_CMD_Temp.slaveAddr=device_addr;
	Queue_En(&IIC_CMD_Queue,IIC_CMD_Temp);//�µ��������  
}


/////////////////////////////////////
///IIC�ж��¼�������
/////////////////////////////////////
void I2C_EV_IRQHandler(void)
{
	uint32_t I2C_Status = I2C_GetLastEvent(I2C);
	DMA_InitTypeDef DMA_InitStructure;
	#ifdef DEBUG
		printf("EV_IRQ\n");
	#endif
	switch(I2C_Status)//��ѯ�ж��¼�����
	{
		case I2C_EVENT_MASTER_MODE_SELECT: //EV5   //SB��BUSY��MSLλ��λ	 
			if(IIC_CMD_Queue.State==STATE_SEND_ADW)//���ʹӻ���ַ+д�ź�
			{	
				I2C->CR2 &= ~I2C_IT_BUF; //ʹ��DMA,�ر�BUF�ж�
				I2C->CR2 |= I2C_CR2_DMAEN; //ʹ��IIC DMA����
				
				I2C_Send7bitAddress(I2C,IIC_CMD_Current.slaveAddr,I2C_Direction_Transmitter);
				IIC_CMD_Queue.State = STATE_SEND_DATA;//״̬����Ϊ��Ҫ�������ݣ����ͼĴ�����ַ��
					/* I2Cx Common Channel Configuration */
				
				if(IIC_CMD_Current.cmdType>=I2C_WRITE_BYTE)//дģʽ
				{
					DMA_InitStructure.DMA_BufferSize = IIC_CMD_Current.outDataLen;
				}
				else                                        //��ģʽ ���ó���Ϊ1 ��Ϊ����һ���ֽں���Ҫ�ٴη�����ʼ�ź�
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
			else if(IIC_CMD_Queue.State==STATE_SEND_ADR)//���ʹӻ���ַ+���ź�
			{
				
				I2C_Send7bitAddress(I2C,IIC_CMD_Current.slaveAddr,I2C_Direction_Receiver);
				IIC_CMD_Queue.State = STATE_REVEIVE_DATA;//״̬����Ϊ��Ҫ�������ݣ����ͼĴ�����ַ��
				
				if( IIC_CMD_Current.cmdType==I2C_READ_BYTE)//����һ���ֽڣ�����ʹ��DMA��ʹ���жϵķ�ʽ
				{
					I2C->CR2 |= I2C_IT_BUF; //����һ���ֽڣ��� BUF�жϣ���ʹ��DMA
					I2C->CR2 &= ~I2C_CR2_DMAEN; //ʧ��IIC DMA����
				}
				else //���յ��ֽ�������1��ʹ��DMA
				{
					I2C->CR2|=I2C_CR2_LAST;//ʹ�ܽ������һλʱ����NACK
					I2C->CR2 &= ~I2C_IT_BUF; //ʹ��DMA,�ر�BUF�ж�
					I2C->CR2 |= I2C_CR2_DMAEN; //ʹ��IIC DMA����
					
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
		
		
//		case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED://EV6	//��: I2C_Send7bitAddress(W)   Ӧ: I2C_SendData()
//			break;
		
		
		case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED://EV6			//��: I2C_Send7bitAddress(R)		
			if(IIC_CMD_Current.cmdType==I2C_READ_BYTE)//ֻ����һ���ֽ�
			{
				I2C_AcknowledgeConfig(I2C,DISABLE);//��Ӧ��
				I2C_GenerateSTOP(I2C,ENABLE);//����ֹͣ�ź�
			}
			break;
		
		
		case I2C_EVENT_MASTER_BYTE_RECEIVED://EV7   //���ݵ����������
			if(IIC_CMD_Current.cmdType==I2C_READ_BYTE)//ֻ����һ���ֽ�
			{
				(*IIC_CMD_Current.pDataIn) = I2C_ReceiveData(I2C); //������յ�������
			
				I2C->CR2 &= ~I2C_IT_BUF; //ʹ��DMA,�ر�BUF�ж�
			
				//�������Ƿ�Ϊ��
				if(IIC_CMD_Queue.MemLength>0)//���в�Ϊ��
				{
					IIC_CMD_Current=Queue_Del(&IIC_CMD_Queue);//��һ���������
					if(IIC_CMD_Current.cmdType == I2C_READ_DIRECT)//ֱ�Ӷ�ȡģʽ��״̬����Ϊ��ȡ
					{
						IIC_CMD_Queue.State = STATE_REVEIVE_DATA;//״̬����Ϊ��������ģʽ
					}
					else //�����ģ���Ҫ�ȷ��ʹӻ���ַ+д�ź�
					{
						IIC_CMD_Queue.State = STATE_SEND_ADW;//��־����Ϊ��Ҫ���ʹӻ���ַ+д�ź�
						IIC_CMD_Queue.Index_Send = 0;//���͡����ռ����±�����
						I2C_AcknowledgeConfig(I2C,ENABLE);	//ʹ��Ӧ��
						I2C_GenerateSTART(I2C,ENABLE);		//���������ź�
					}
				}
				else//����Ϊ�գ�
				{
					IIC_CMD_Queue.State=STATE_READY;//��״̬����Ϊ ׼���÷��� ģʽ
				}
			}
			break;
		
		
		case I2C_EVENT_MASTER_BYTE_TRANSMITTED:	//EV8_2					//��: I2C_SendData() 
			if(0==DMA_GetCurrDataCounter(I2C_DMA_TX_Channel))//����Ƿ������
			{
				#ifdef DEBUG
					printf("\nDMA_translate_Complet\n");
				#endif
				DMA_Cmd(I2C_DMA_TX_Channel, DISABLE);//��DMA
				
				if(IIC_CMD_Current.cmdType >= I2C_WRITE_BYTE)//����дģʽ ���ֽڻ��߶��ֽ�
					{
						//�Ѿ�д�꣬����ֹͣ�ź�
						I2C_GenerateSTOP(I2C,ENABLE);//����ֹͣ�ź�
						//�ж϶����Ƿ�Ϊ��
						if(IIC_CMD_Queue.MemLength>0)//���в�Ϊ��
						{
							IIC_CMD_Current=Queue_Del(&IIC_CMD_Queue);//��һ���������
							
							 //��Ҫ�ȷ��ʹӻ���ַ+д�ź�
						
							IIC_CMD_Queue.State = STATE_SEND_ADW;//��־����Ϊ��Ҫ���ʹӻ���ַ+д�ź�
							IIC_CMD_Queue.Index_Send = 0;//���͡����ռ����±�����
							I2C_AcknowledgeConfig(I2C,ENABLE);	//ʹ��Ӧ��
							I2C_GenerateSTART(I2C,ENABLE);		//���������ź�
						}
						else//����Ϊ�գ�
						{
							IIC_CMD_Queue.State=STATE_READY;//��״̬����Ϊ ׼���÷��� ģʽ
						}
					}
					else//���ڶ�ģʽ
					{
						IIC_CMD_Queue.State = STATE_SEND_ADR;//״̬����Ϊ ���ʹӻ���ַ+�� 
						I2C_GenerateSTART(I2C,ENABLE);//��ʼ�ź�
					}
			}
			break;
//		
		
//		case I2C_EVENT_MASTER_BYTE_TRANSMITTING://EV8   /* TRA, BUSY, MSL, TXE flags */

//			break;
	}
}



////////////////////////////////////////////////
///IIC�����жϴ�����
////////////////////////////////////////////////
uint32_t I2C_ER_IRQHandler(void)
{
	u16 Error=I2C->SR1 & ((uint16_t)0x0F00) ; /*!< I2C errors Mask  *///��ȡ������Ϣ
	IIC_CMD_Queue.State = STATE_ERROR;//״̬����Ϊ����״̬
	I2C->SR1 = ~((uint16_t)0x0F00);//���������Ϣ
	
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
		//���������ڴ򿪺�������ʱδ�ҵ���������
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
	
	IIC_CMD_Queue.State = STATE_READY;//״̬����Ϊ����
	return 0;
}
void I2C_DMA_TX_IRQHandler()
{
	if((uint32_t)( I2C_DMA->ISR & I2C_DMA_TX_TC_FLAG )!=0)//�������
	{
		
		
		
	}
	else if((uint32_t)( I2C_DMA->ISR & I2C_DMA_TX_HT_FLAG )!=0)//����һ��
	{
		
	}
	else if((uint32_t)( I2C_DMA->ISR & I2C_DMA_TX_TE_FLAG )!=0)//���䷢������
	{
		
	}
	I2C_DMA->IFCR=I2C_DMA_TX_TC_FLAG | I2C_DMA_TX_HT_FLAG | I2C_DMA_TX_TE_FLAG;//����жϱ�־
	
}
void I2C_DMA_RX_IRQHandler()
{
	if((uint32_t)( I2C_DMA->ISR & I2C_DMA_RX_TC_FLAG )!=0)//�������
	{
	//	if(!DMA_GetCurrDataCounter(I2C_DMA_RX_Channel))//����Ƿ�������   //�˴������ж� ,�жϻ���� Ϊ�������
	//	printf("\n\n\n....%d...\n\n\n",DMA_GetCurrDataCounter(I2C_DMA_RX_Channel));
		{
		#ifdef DEBUG
			printf("\nDMA reveive Complet\n");
		#endif
			DMA_Cmd(I2C_DMA_RX_Channel, DISABLE);//��DMA
			I2C_GenerateSTOP(I2C,ENABLE);		//����ֹͣ�ź�  
								//�������Ƿ�Ϊ��
			if(IIC_CMD_Queue.MemLength>0)//���в�Ϊ��
				{
					IIC_CMD_Current=Queue_Del(&IIC_CMD_Queue);//��һ���������
					//��Ҫ�ȷ��ʹӻ���ַ+д�ź�
					
					IIC_CMD_Queue.State = STATE_SEND_ADW;//��־����Ϊ��Ҫ���ʹӻ���ַ+д�ź�
					IIC_CMD_Queue.Index_Send = 0;//���͡����ռ����±�����
					I2C_AcknowledgeConfig(I2C,ENABLE);	//ʹ��Ӧ��
					I2C_GenerateSTART(I2C,ENABLE);		//���������ź�	
				}
				else//����Ϊ�գ�
				{
					IIC_CMD_Queue.State=STATE_READY;//��״̬����Ϊ ׼���÷��� ģʽ
				}
		}
	}
	else if((uint32_t)( I2C_DMA->ISR & I2C_DMA_RX_HT_FLAG )!=0)//����һ��
	{
		
	}
	else if((uint32_t)( I2C_DMA->ISR & I2C_DMA_RX_TE_FLAG )!=0)//���䷢������
	{
		
	}
	I2C_DMA->IFCR=I2C_DMA_RX_TC_FLAG | I2C_DMA_RX_HT_FLAG | I2C_DMA_RX_TE_FLAG;//����жϱ�־
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
///Arbitration lost (master mode)�ٲö�ʧ   IIC���ߴ����û��ص�����
//////////////////////////////////
void I2C_ARLO_UserCallback(void)
{
	
}



///////////////////////////////////
///Overrun/Underrun ���ء�Ƿ�ش���   IIC���ߴ����û��ص�����
//////////////////////////////////
void I2C_OVR_UserCallback(void)
{
	
}


//////////////////////////////////
///(Bus error ���߳���         IIC���ߴ����û��ص�����
//////////////////////////////////
void I2C_BERR_UserCallback(void)
{
	//Ӳ�������ڶ�д�����жϿ�Ҳ������ô���
	//�ڱ������л�������Ӧ��ص�����I2C_AF_UserCallback()�н��д�����ֹ������һ�����Ѿ�ʧȥ�����˵��豸���ж�д
}


///////////////////////////////
///Acknowledge failure User Callback Function Ӧ��ʧ�� IIC���ߴ����û��ص�����
///////////////////////////////
void I2C_AF_UserCallback(void)
{
	//��Ӧ��ʱ��Ӧ�����·������ݻ���ֹͣ�ٴη����豸�Ķ���
	
	
}

