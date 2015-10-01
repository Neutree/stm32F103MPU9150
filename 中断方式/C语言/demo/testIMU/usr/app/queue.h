/**
*@file /usr/app/queue.h
*@brief ѭ������
*
*
*
*
**/


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

	STATE_SEND_ADW = 0x20,//��Ҫ���ʹӻ���ַ+д�ź�
	
	STATE_SEND_ADR = 0x21,//��Ҫ���ʹӻ���ַ+���ź�
	
	STATE_SEND_DATA= 0x22, //��Ҫ�������ݣ������Ĵ�����ַ�����������������Ҫ���͵����ݣ�
	
	STATE_REVEIVE_DATA= 0x30 //��������
	
}Queue_State;

typedef enum
{
	I2C_READ_BYTE   = 0x01,	//�Ӵӻ���ָ���Ĵ�����һ���ֽ�
	I2C_READ_BYTES  = 0x02,	//�Ӵӻ���ָ���Ĵ���������ֽ�
	I2C_READ_DIRECT = 0x03,  //�Ӵӻ���ȡ����ֽ�(�޼Ĵ���)
	I2C_WRITE_BYTE  = 0x04,	//��ӻ���ָ���Ĵ���дһ���ֽ�
	I2C_WRITE_BYTES	= 0x05,	//��ӻ���ָ���Ĵ���д����ֽ�
	I2C_WRITE_CMD   = 0x06,  //��ӻ�дһ������(�޼Ĵ���)
}IIC_CMD_Type;
typedef struct 
{ 
	__IO IIC_CMD_Type cmdType;			//��������
	u8 slaveAddr;		//�ӻ���ַ
	u8 DataOut[5];	//��������ݣ����������ֽ�
	u8 outDataLen;	//��������ֽ���
	u8* pDataIn;		//�������ݵĴ���׵�ַ
	u8 inDataLen;   //���������ֽ���	
}I2C_Command_Struct;

typedef struct//ѭ�����нṹ��
{
	__IO u16 MemFront;//ͷ�±�
	__IO u16 MemRear;//β�±�
	__IO u16 MemLength;//���г���
	__IO Queue_State State;//״̬��ע
	__IO u16 Index_Send;//���ڷ��͵����ݵ��±�
	elemtype MemDataBuf[MAX_QUEUE_LEN];
}Queue_Mem_Struct, *Queue_Mem_Struct_p;

extern elemtype Queue_Init_Value;
//////////////////////
//���в�����������
//////////////////////

//��ʼ������
void Queue_Init(Queue_Mem_Struct_p sq);
//����г���
u16 Queue_GetSize(Queue_Mem_Struct_p sq);
//����ͷԪ��
elemtype Queue_Head(Queue_Mem_Struct_p sq);
//����
elemtype Queue_Del(Queue_Mem_Struct_p sq);
//���
void Queue_En(Queue_Mem_Struct_p sq,elemtype data);
//����������
void Queue_ERROverflowCallback(Queue_Mem_Struct_p sq,elemtype data);

#ifdef __cplusplus
}
#endif

#endif
