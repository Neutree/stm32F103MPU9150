/**
* @file queue.h
* @author 1208077207@qq.com
* @version v0.1
* @date 2015-10-4
* @pre 
* @brief  Circular queue related operation , and few struct type about iic 
* @bug  
* @warning   
* @copyright 
* @attention 
*/


#ifndef __QUEUE_H
#define __QUEUE_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "stm32f10x.h"


#define MAX_QUEUE_LEN 51
#define elemtype I2C_Command_Struct
	

typedef enum
{
	STATE_DISABLED = 0x00,       
    
	STATE_READY    = 0x01,   

	STATE_BUSY     = 0x02, 	    

	STATE_ERROR    = 0x10,  

	STATE_SEND_ADW = 0x20,//需要发送从机地址+写信号
	
	STATE_SEND_ADR = 0x21,//需要发送从机地址+读信号
	
	STATE_SEND_DATA= 0x22, //需要发送数据（包括寄存器地址或者命令或者其它需要发送的数据）
	
	STATE_REVEIVE_DATA= 0x30 //接收数据
	
}Queue_State;

typedef enum
{
	I2C_READ_BYTE   = 0x01,	//从从机的指定寄存器读一个字节
	I2C_READ_BYTES  = 0x02,	//从从机的指定寄存器读多个字节
	I2C_WRITE_BYTE  = 0x04,	//向从机的指定寄存器写一个字节
	I2C_WRITE_BYTES	= 0x05,	//向从机的指定寄存器写多个字节
	I2C_WRITE_CMD   = 0x06,  //向从机写一个命令(无寄存器)
}IIC_CMD_Type;
typedef struct 
{ 
	__IO IIC_CMD_Type cmdType;			//命令类型
	u8 slaveAddr;		//从机地址
	u8 DataOut[5];	//输出的数据，仅限少量字节
	u8 outDataLen;	//输出数据字节数
	u8* pDataIn;		//读入数据的存放首地址
	u8 inDataLen;   //读入数据字节数	
}I2C_Command_Struct;

typedef struct//循环队列结构体
{
	__IO u16 MemFront;//头下标
	__IO u16 MemRear;//尾下标
	__IO u16 MemLength;//队列长度
	__IO Queue_State State;//状态备注
	__IO u16 Index_Send;//正在发送的数据的下标
	elemtype MemDataBuf[MAX_QUEUE_LEN];
}Queue_Mem_Struct, *Queue_Mem_Struct_p;

extern elemtype Queue_Init_Value;
//////////////////////
//队列操作函数声明
//////////////////////

//////////////////
///初始化队列
//////////////////
void Queue_Init(Queue_Mem_Struct_p sq);

/////////////////////////
///求队列长度
////////////////////////
u16 Queue_GetSize(Queue_Mem_Struct_p sq);

///////////////////////
///读队头元素
///////////////////////
elemtype Queue_Head(Queue_Mem_Struct_p sq);

//////////////////////
///出队
//////////////////////
elemtype Queue_Del(Queue_Mem_Struct_p sq);

//////////////////////
///入队
//////////////////////
void Queue_En(Queue_Mem_Struct_p sq,elemtype data);

///////////////////////
///入队溢出错误
///////////////////////
void Queue_ERROverflowCallback(Queue_Mem_Struct_p sq,elemtype data);

#ifdef __cplusplus
}
#endif

#endif
