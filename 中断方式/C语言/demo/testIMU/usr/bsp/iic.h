#ifndef __IIC_H
#define __IIC_H

#ifdef __cplusplus
extern "C" {
#endif
	
/////////////////////////////////
///include file
////////////////////////////////
#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "queue.h"

	
/***********************************************************************************************************************************************************/
////////////////////////////////////																														
///I2C  HAL Configuration
////////////////////////////////////
	
				#define USE_I2C1                          //����ʹ��I2C1
//				#define I2C1_REMAP 						  //I2C�˿���ӳ��򿪣�ʹ��PB8��PB9    ȡ��ע����Ч
					
//				#define USE_I2C2                          //����ʹ��I2C2	ʹ��I2C2��ȡ��ע�Ͳ���USE_I2C1ע�͵�   ��û��ע��USE_I2C1 ��Ĭ��ʹ��I2C1
					
	
				#define I2C_IT_EVT_SUBPRIO              0   /* I2C EVT SUB-PRIORITY */ 
				#define I2C_IT_EVT_PREPRIO              2   /* I2C EVT PREEMPTION PRIORITY */ 
				#define I2C_IT_ERR_SUBPRIO              0   /* I2C ERR SUB-PRIORITY */
				#define I2C_IT_ERR_PREPRIO              0   /* I2C ERR PREEMPTION PRIORITY */
//				#define I2C_IT_DMATX_SUBPRIO            0   /* I2C DMA TX SUB-PRIORITY */
//				#define I2C_IT_DMATX_PREPRIO            1   /* I2C DMA TX PREEMPTION PRIORITY */
//				#define I2C_IT_DMARX_SUBPRIO            0   /* I2C DMA RX SUB-PRIORITY */
//				#define I2C_IT_DMARX_PREPRIO            1   /* I2C DMA RX PREEMPTION PRIORITY */

/***********************************************************************************************************************************************************/



	
///////////////////////////////////
///I2C definition
///////////////////////////////////


#ifdef USE_I2C1

	#define I2C        I2C1
	#ifdef I2C1_REMAP //ʹ�ö˿���ӳ��
		#define I2C_SCL_GPIO_PIN   		  GPIO_Pin_8
		#define I2C_SDA_GPIO_PIN          GPIO_Pin_9
	#else            //��ʹ�ö˿���ӳ��
		#define I2C_SCL_GPIO_PIN   		  GPIO_Pin_6
		#define I2C_SDA_GPIO_PIN          GPIO_Pin_7
	#endif
	
	#define I2C_CLK                   RCC_APB1Periph_I2C1
	#define I2C_EV_IRQn                I2C1_EV_IRQn
	#define I2C_ER_IRQn                I2C1_ER_IRQn
	#define I2C_EV_IRQHandler          I2C1_EV_IRQHandler
	#define I2C_ER_IRQHandler          I2C1_ER_IRQHandler
	
#else
	
	#define I2C        I2C2
	#define I2C_SCL_GPIO_PIN   		  GPIO_Pin_10
	#define I2C_SDA_GPIO_PIN          GPIO_Pin_11
	
	
	#define I2C_CLK                   RCC_APB1Periph_I2C2
	#define I2C_EV_IRQn                I2C2_EV_IRQn
	#define I2C_ER_IRQn                I2C2_ER_IRQn
	#define I2C_EV_IRQHandler          I2C2_EV_IRQHandler
	#define I2C_ER_IRQHandler          I2C2_ER_IRQHandler
	
#endif


#define I2C_SCL_GPIO_PORT   GPIOB       
#define I2C_SCL_GPIO_CLK   RCC_APB2Periph_GPIOB 

#define I2C_SDA_GPIO_PORT         GPIOB       
#define I2C_SDA_GPIO_CLK          RCC_APB2Periph_GPIOB 


//#define I2C_DMA                   DMA1
//#define I2C1_DMA_CLK               RCC_AHBPeriph_DMA1
//#define I2C_DMA_CLK               I2C1_DMA_CLK 
//#define I2C_DMA_TX_TC_FLAG        DMA1_FLAG_TC6
//#define I2C_DMA_TX_HT_FLAG        DMA1_FLAG_HT6
//#define I2C_DMA_TX_TE_FLAG        DMA1_FLAG_TE6
//  
//#define I2C_DMA_RX_TC_FLAG        DMA1_FLAG_TC7
//#define I2C_DMA_RX_HT_FLAG        DMA1_FLAG_HT7
//#define I2C_DMA_RX_TE_FLAG        DMA1_FLAG_TE7
///*----------- I2C1 Device -----------*/
//#define I2C1_DMA_TX_Channel        DMA1_Channel6
//#define I2C1_DMA_RX_Channel        DMA1_Channel7
//  
///*----------- I2C2 Device -----------*/
//#define I2C2_DMA_TX_Channel        DMA1_Channel4
//#define I2C2_DMA_RX_Channel        DMA1_Channel5

//#define I2C_DMA_TX_Channel         I2C1_DMA_TX_Channel
//#define I2C_DMA_RX_Channel         I2C1_DMA_RX_Channel


//#define I2C_DMA_TX_IRQn            DMA1_Channel6_IRQn
//#define I2C_DMA_RX_IRQn            DMA1_Channel7_IRQn


//#define I2C_DMA_TX_IRQHandler      DMA1_Channel6_IRQHandler
//#define I2C_DMA_RX_IRQHandler      DMA1_Channel7_IRQHandler




#define I2C_True    1
#define I2C_False   0
#define I2C_Bool    u8

 
extern Queue_Mem_Struct IIC_CMD_Queue;//�������Ķ���
extern elemtype IIC_CMD_Current;//��ŵ�ǰ����Ķ���

/* I2C Errors TypeDef */

typedef enum
{
  I2C_ERR_NONE      = 0x0000, /*!<No Error: This is the default state for an Idle peripheral */

  I2C_ERR_TIMEOUT   = 0x00FF, /*!<Timeout error: The specified timeout has been elapsed without 
                                         any response (expected flag or data didn't happen at expected time). */

  I2C_ERR_BERR      = 0x0100, /*!<Bus error: This error occurs when I2C peripheral detects an external
                                       Stop or Start condition during address or data transfer. In this case:
                                          - The BERR bit is set and an interrupt is generated if the ITERREN bit is set.
                                       In Slave mode: 
                                         data are discarded and the lines are released by hardware:
                                          - In case of a misplaced Start, the slave considers it is a restart and 
                                            waits for an address, or a Stop condition.
                                          - In case of a misplaced Stop, the slave behaves like for a Stop condition and 
                                           the lines are released by hardware.
                                       In Master mode: 
                                         the lines are not released and the state of the current transmission is not 
                                         affected. It is up to the software to abort or not the current transmission.
                                       
                                       Software Clearing sequence for the BERR bit:      
                                         1. Writing '0' to this bit  */
                                            
                                                      
  I2C_ERR_ARLO        = 0x0200, /*!<Arbitration Lost error: This error occurs when the I2C interface detects 
                                         an arbitration lost condition. In this case:
                                          - The ARLO bit is set by hardware (and an interrupt is generated if the 
                                            ITERREN bit is set).
                                         the I2C Interface goes automatically back to slave mode (the M/SL bit 
                                         is cleared). 
                                         When the I2C loses the arbitration, it is not able to acknowledge its slave
                                         address in the same transfer, but it can acknowledge it after a repeated 
                                         Start from the winning master.
                                         Lines are released by hardware.
                                              
                                         Software Clearing sequence for the BERR bit:      
                                          1. Writing '0' to this bit  */
                                                  
  I2C_ERR_AF          = 0x0400, /*!<Acknowledge Failure : This error occurs when the interface detects 
                                         a non-acknowledge bit. In this case:
                                          - The AF bit is set and an interrupt is generated if the ITERREN bit 
                                            is set.
                                         A transmitter which receives a NACK must reset the communication:
                                          - If Slave: lines are released by hardware.
                                          - If Master: a Stop or repeated Start condition must be generated 
                                            by software.
                                                 
                                         Software Clearing sequence for the ARLO bit:
                                         1. Writing '0' to this bit */                                        
                                                      
  I2C_ERR_OVR          = 0x0800, /*!<Overrun/Underrun error: An overrun error can occur in slave mode when clock 
                                          stretching is disabled and the I2C interface is receiving data. The interface has
                                          received a byte (RxNE=1) and the data in DR has not been read, before the next 
                                          byte is received by the interface. 
                                          In this case:
                                          The last received byte is lost.
                                           - In case of Overrun error, software should clear the RxNE bit and the transmitter 
                                             should retransmit the last received byte.
                                          
                                          Underrun error can occur in slave mode when clock stretching is disabled and the 
                                          I2C interface is transmitting data. The interface has not updated the DR with the 
                                          next byte(TxE=1), before the clock comes for the next byte. In this case:
                                           - The same byte in the DR register will be sent again.
                                           - The user should make sure that data received on the receiver side during an 
                                             underrun error are discarded and that the next bytes are written within the 
                                             clock low time specified in the I2C bus standard.
                                          For the first byte to be transmitted, the DR must be written after ADDR is 
                                          cleared and before the first SCL rising edge. If not possible, the receiver 
                                          must discard the first data.
                                      
                                       Software Clearing sequence for the ARLO bit:
                                        1. Writing '0' to this bit */
                                                  
 }I2CErrorTypeDef;

void I2C_Init_Config(void);
///////////////////////////////
///�µ��������
//////////////////////////////
void IIC_Queue_Del(void);
//////////////////////////////
///��ʼд/������
/////////////////////////////
void IIC_Start_CMD(void);
 
 
//////////////////////////////
///��ʼдһ������
/////////////////////////////
void IIC_Start_Next_CMD(void);
 
 
///////////////////////////////////
///��Ӷ�ȡ����ֽ�����
///ע�⣺��ʱû�п�ʼִ�����ֻ�Ƿŵ��������ˣ�IIC_Queue_Del();IIC_Start_Next_CMD();����ִ�ж�������һ������
///////////////////////////////////
void I2C_AddCMD_Read_Bytes(u8 device_addr,u8 register_addr, u8* data_read, u8 num);//�������豸��ַ���Ĵ�����ַ����ȡ�����ݴ�ŵĵ�ַ����Ҫ��ȡ���ݵĸ���

///////////////////////////////////
///���д�����ֽ�����
///ע�⣺��ʱû�п�ʼִ�����ֻ�Ƿŵ��������ˣ�IIC_Queue_Del();IIC_Start_CMD();����ִ�ж�������һ������
///////////////////////////////////
void I2C_AddCMD_Write_Bytes(u8 device_addr,u8 register_addr, u8* data_write, u8 num);//�������豸��ַ���Ĵ�����ַ����ȡ�����ݴ�ŵĵ�ַ����Ҫ��ȡ���ݵĸ���

#ifdef __cplusplus
}
#endif


#endif

