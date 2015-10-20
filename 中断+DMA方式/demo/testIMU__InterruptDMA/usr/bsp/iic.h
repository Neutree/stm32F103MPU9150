/**
* @file iic.h
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
#include "delay.h"
//#include "usart.h"	


	

	
/***********************************************************************************************************************************************************/
										////////////////////////////////////																														
										///I2C  HAL Configuration IIC配置
										////////////////////////////////////
	
//				#define USE_I2C1                          //定义使用I2C1
//				#define I2C1_REMAP 						  //I2C端口重映射打开，使用PB8和PB9    取消注释生效
					
				#define USE_I2C2                          //定义使用I2C2	使用I2C2则取消注释并把USE_I2C1注释掉   若没有注释USE_I2C1 则默认使用I2C1
					
				#define I2C_SPEED   200000                //I2C speed  Max:400000    
				
				#define I2C_NVIC_PriorityGroup          NVIC_PriorityGroup_3
				#define I2C_IT_EVT_SUBPRIO              0   /* I2C EVT SUB-PRIORITY */ 
				#define I2C_IT_EVT_PREPRIO              2   /* I2C EVT PREEMPTION PRIORITY */ 
				#define I2C_IT_ERR_SUBPRIO              0   /* I2C ERR SUB-PRIORITY */
				#define I2C_IT_ERR_PREPRIO              0   /* I2C ERR PREEMPTION PRIORITY */
				#define I2C_IT_DMATX_SUBPRIO            0   /* I2C DMA TX SUB-PRIORITY */
				#define I2C_IT_DMATX_PREPRIO            1   /* I2C DMA TX PREEMPTION PRIORITY */
				#define I2C_IT_DMARX_SUBPRIO            0   /* I2C DMA RX SUB-PRIORITY */
				#define I2C_IT_DMARX_PREPRIO            1   /* I2C DMA RX PREEMPTION PRIORITY */

				#define I2C_TIME_OUT_MAX_TIME           3   /* 在调用开始执行命令队列中的命令时会对当前的总线状态进行检查，
																若连续I2C_TIME_OUT_MAX_TIME次都是一样的状态（STATE_READY除外），则判定为超时 */

//				#define USE_ERROR_CALLBACK                 /*  使用错误回调函数  */

/***********************************************************************************************************************************************************/






/***********************************************************************************************************************************************************/
													////////////////////////////////////
													///接口函数
													///////////////////////////////////

///////////////////////////
///初始化IIC硬件
///////////////////////////
u8 I2C_Init_Config(void);

 
///////////////////////////
///获取I2C设备的通信健康状况
///@retval -0:健康 -1：出现错误
///////////////////////////
u8 I2C_Get_Health(void);
 
//////////////////////////////
///开始写队列中下一个一个命令
///如果队列中不存在命令或者硬件正在传送队列中的命令，则无需执行本函数，调用本函数也不会执行任何内容
///@retval -1:成功发送开始信号，可能会执行成功 -0:发送成功信号失败,I2C总线通信错误
/////////////////////////////
u8 IIC_Start_Next_CMD(void);

 
 
///////////////////////////////////
///添加读取多个字节命令
///@param device_addr -设备地址
///@param register_addr -设备寄存器地址
///@param data_read -读取的数据存放的地方
///@param num -读取的数量
///@attention 注意：此时没有开始执行命令，只是放到队列中了，IIC_Queue_Del();IIC_Start_Next_CMD();语句会执行队列中下一个命令
///////////////////////////////////
void I2C_AddCMD_Read_Bytes(u8 device_addr,u8 register_addr, u8* data_read, u8 num);//参数：设备地址，寄存器地址，读取的数据存放的地址，需要读取数据的个数




///////////////////////////////////
///添加写入多个字节命令
///@param device_addr -设备地址
///@param register_addr -设备寄存器地址
///@param data_write -写入的数据的来源地址
///@param num -写入数据的数量
///@attention 注意：此时没有开始执行命令，只是放到队列中了，IIC_Queue_Del();IIC_Start_CMD();语句会执行队列中下一个命令
///////////////////////////////////
void I2C_AddCMD_Write_Bytes(u8 device_addr,u8 register_addr, u8* data_write, u8 num);//参数：设备地址，寄存器地址，读取的数据存放的地址，需要读取数据的个数





	

///////////////////////////////////
///添加如指令命令 （无寄存器）
///@param device_addr -设备地址
///@param CMD_Write -要写入设备的命令
///@param num -写入命令的长度（单位：字节）
///@attention注意：此时没有开始执行命令，只是放到队列中了，IIC_Queue_Del();IIC_Start_CMD();语句会执行队列中下一个命令
///////////////////////////////////
void I2C_AddCMD_Write_CMD_Bytes(u8 device_addr, u8* CMD_Write, u8 num);//参数：设备地址，命令，需要写入的命令的个数
	





///////////////////////////////////
///添加如指令命令 （无寄存器）
///@param device_addr -设备地址
///@param CMD_Write -要写入设备的命令
///注意：此时没有开始执行命令，只是放到队列中了，IIC_Queue_Del();IIC_Start_CMD();语句会执行队列中下一个命令
///////////////////////////////////
void I2C_AddCMD_Write_CMD_Byte(u8 device_addr, u8 CMD_Write);//参数：设备地址，命令

/***********************************************************************************************************************************************************/






/***********************************************************************************************************************************************************/
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/*以下为错误回调函数，若要使用该回调函数，请将上面配置处的  #define USE_ERROR_CALLBACK 取消注释，并自行定义这些函数，否则会报错****************/
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(USE_ERROR_CALLBACK) 

///////////////////////////////////
///错误中断,任何错误情况都会触发回调，前提是开启对调函数
//////////////////////////////////
void I2C_Error_UserCallback(u16 Error);


///////////////////////////////
///Acknowledge failure User Callback Function 应答失败 IIC总线错误用户回调函数
///////////////////////////////
void I2C_AF_UserCallback(void);


////////////////////////////////
///Arbitration lost (master mode)仲裁丢失   IIC总线错误用户回调函数
//////////////////////////////////
void I2C_ARLO_UserCallback(void);



///////////////////////////////////
///Overrun/Underrun 过载、欠载错误   IIC总线错误用户回调函数、
//////////////////////////////////
void I2C_OVR_UserCallback(void);

//////////////////////////////////
///(Bus error 总线出错         IIC总线错误用户回调函数
//////////////////////////////////
void I2C_BERR_UserCallback(void);
//////////////////////////////////
///(Time out error 超时         IIC超时错误用户回调函数
//////////////////////////////////
void I2C_TIME_OUT_UserCallback(void);


#endif /* USE_SINGLE_ERROR_CALLBACK */
/***********************************************************************************************************************************************************/









/*******************************************************************************************************************************/
/***********************************************以下内容不建议修改**************************************************************/
/********************************************配置I2C和查看接口请看上面**************************************************************/
/*******************************************************************************************************************************/

///////////////////////////////////
///I2C definition
///////////////////////////////////



#ifdef USE_I2C1            //I2C1

	#define I2C        I2C1
	#ifdef I2C1_REMAP //使用端口重映射
		#define I2C_SCL_GPIO_PIN   		  GPIO_Pin_8
		#define I2C_SDA_GPIO_PIN          GPIO_Pin_9
	#else            //不使用端口重映射
		#define I2C_SCL_GPIO_PIN   		  GPIO_Pin_6
		#define I2C_SDA_GPIO_PIN          GPIO_Pin_7
	#endif
	
	#define I2C_CLK                   RCC_APB1Periph_I2C1
	#define I2C_EV_IRQn                I2C1_EV_IRQn
	#define I2C_ER_IRQn                I2C1_ER_IRQn
	#define I2C_EV_IRQHandler          I2C1_EV_IRQHandler
	#define I2C_ER_IRQHandler          I2C1_ER_IRQHandler
	
	#define I2C_DMA_CLK               I2C1_DMA_CLK
	#define I2C_DMA_TX_Channel         I2C1_DMA_TX_Channel
	#define I2C_DMA_RX_Channel         I2C1_DMA_RX_Channel
	#define I2C_DMA_TX_TC_FLAG        DMA1_FLAG_TC6
	#define I2C_DMA_TX_HT_FLAG        DMA1_FLAG_HT6
	#define I2C_DMA_TX_TE_FLAG        DMA1_FLAG_TE6
	  
	#define I2C_DMA_RX_TC_FLAG        DMA1_FLAG_TC7
	#define I2C_DMA_RX_HT_FLAG        DMA1_FLAG_HT7
	#define I2C_DMA_RX_TE_FLAG        DMA1_FLAG_TE7
	
	#define I2C_DMA_TX_IRQn            DMA1_Channel6_IRQn
	#define I2C_DMA_RX_IRQn            DMA1_Channel7_IRQn
	
	#define I2C_DMA_TX_IRQHandler      DMA1_Channel6_IRQHandler
	#define I2C_DMA_RX_IRQHandler      DMA1_Channel7_IRQHandler

#else             //I2C2
	
	#define I2C        I2C2
	#define I2C_SCL_GPIO_PIN   		  GPIO_Pin_10
	#define I2C_SDA_GPIO_PIN          GPIO_Pin_11
	
	
	#define I2C_CLK                   RCC_APB1Periph_I2C2
	#define I2C_EV_IRQn                I2C2_EV_IRQn
	#define I2C_ER_IRQn                I2C2_ER_IRQn
	#define I2C_EV_IRQHandler          I2C2_EV_IRQHandler
	#define I2C_ER_IRQHandler          I2C2_ER_IRQHandler
	
	#define I2C_DMA_CLK                I2C2_DMA_CLK
	#define I2C_DMA_TX_Channel         I2C2_DMA_TX_Channel
	#define I2C_DMA_RX_Channel         I2C2_DMA_RX_Channel
	#define I2C_DMA_TX_TC_FLAG         DMA1_FLAG_TC4
	#define I2C_DMA_TX_HT_FLAG         DMA1_FLAG_HT4
	#define I2C_DMA_TX_TE_FLAG         DMA1_FLAG_TE4
	  
	#define I2C_DMA_RX_TC_FLAG         DMA1_FLAG_TC5
	#define I2C_DMA_RX_HT_FLAG         DMA1_FLAG_HT5
	#define I2C_DMA_RX_TE_FLAG         DMA1_FLAG_TE5
	
	#define I2C_DMA_TX_IRQn            DMA1_Channel4_IRQn
	#define I2C_DMA_RX_IRQn            DMA1_Channel5_IRQn
	
	#define I2C_DMA_TX_IRQHandler      DMA1_Channel4_IRQHandler
	#define I2C_DMA_RX_IRQHandler      DMA1_Channel5_IRQHandler
	
#endif


#define I2C_SCL_GPIO_PORT   GPIOB       
#define I2C_SCL_GPIO_CLK   RCC_APB2Periph_GPIOB 

#define I2C_SDA_GPIO_PORT         GPIOB       
#define I2C_SDA_GPIO_CLK          RCC_APB2Periph_GPIOB 


#define I2C_DMA                   DMA1
#define I2C1_DMA_CLK               RCC_AHBPeriph_DMA1
#define I2C2_DMA_CLK               RCC_AHBPeriph_DMA1
 

/*----------- I2C1 Device -----------*/
#define I2C1_DMA_TX_Channel        DMA1_Channel6
#define I2C1_DMA_RX_Channel        DMA1_Channel7
  
/*----------- I2C2 Device -----------*/
#define I2C2_DMA_TX_Channel        DMA1_Channel4
#define I2C2_DMA_RX_Channel        DMA1_Channel5



#define I2C_True    1
#define I2C_False   0
#define I2C_Bool    u8

 
extern Queue_Mem_Struct IIC_CMD_Queue;//存放命令的队列
extern elemtype IIC_CMD_Current;//存放当前命令的队列


 







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

 

void I2C_EV_IRQHandler(void);
uint32_t I2C_ER_IRQHandler(void);
void I2C_DMA_TX_IRQHandler(void);
void I2C_DMA_RX_IRQHandler(void);



#ifdef __cplusplus
}
#endif


#endif

