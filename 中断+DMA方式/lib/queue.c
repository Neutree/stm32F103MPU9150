/**
* @file queue.c
* @author 1208077207@qq.com
* @version v0.1
* @date 2015-10-4
* @pre 
* @brief Circular queue related operation
* @bug  
* @warning   
* @copyright 
* @attention 
*/


#include "queue.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////����///////////////////////////
///////////////ѭ�����У�ͷָ�벻�����������////////////////
///////////////////////////////////////////////////////////
//����������

elemtype Queue_Init_Value;


/////////////////////////
///��ʼ������
/////////////////////////
void Queue_Init(Queue_Mem_Struct_p sq)
{
	u16 i;
	sq->MemFront=0;
	sq->MemRear=0;
	sq->MemLength=0;
	for(i=0;i<MAX_QUEUE_LEN;++i)
		sq->MemDataBuf[i]=Queue_Init_Value;
}


////////////////////////
///����г���
///////////////////////
u16 Queue_GetSize(Queue_Mem_Struct_p sq)
{
	return (MAX_QUEUE_LEN+sq->MemRear-sq->MemFront)%MAX_QUEUE_LEN;
}


////////////////////////
///����ͷԪ��
////////////////////////
elemtype Queue_Head(Queue_Mem_Struct_p sq)
{
	if(sq->MemFront==sq->MemRear)
		return Queue_Init_Value;
	else
		return sq->MemDataBuf[(sq->MemFront+1)%MAX_QUEUE_LEN];
}	



//////////////////////////
///����
//////////////////////////
elemtype Queue_Del(Queue_Mem_Struct_p sq)
{
	if(sq->MemFront==sq->MemRear)
	{
		return Queue_Init_Value;
	}
	else
	{
		sq->MemFront=(sq->MemFront+1)%MAX_QUEUE_LEN;
		--sq->MemLength;
		return (sq->MemDataBuf[sq->MemFront]);
	}
}



///////////////////////////
//���
///////////////////////////
void Queue_En(Queue_Mem_Struct_p sq,elemtype data)
{
	if((sq->MemRear+1)%MAX_QUEUE_LEN==sq->MemFront)//���������
	{
		Queue_ERROverflowCallback(sq,data);
	}
	else{
		sq->MemRear=(sq->MemRear+1)%MAX_QUEUE_LEN;
		sq->MemDataBuf[sq->MemRear]=data;
		++sq->MemLength;
	}
}


/////////////////////////
///����������
/////////////////////////
void Queue_ERROverflowCallback(Queue_Mem_Struct_p sq,elemtype data)
{
	Queue_Del(sq);//���ӣ�����ǰ�������
	Queue_En(sq,data);//���������
}

