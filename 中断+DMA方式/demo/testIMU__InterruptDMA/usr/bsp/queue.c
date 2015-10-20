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
////////////////////////////队列///////////////////////////
///////////////循环队列，头指针不用来存放数据////////////////
///////////////////////////////////////////////////////////
//入队溢出错误

elemtype Queue_Init_Value;


/////////////////////////
///初始化队列
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
///求队列长度
///////////////////////
u16 Queue_GetSize(Queue_Mem_Struct_p sq)
{
	return (MAX_QUEUE_LEN+sq->MemRear-sq->MemFront)%MAX_QUEUE_LEN;
}


////////////////////////
///读队头元素
////////////////////////
elemtype Queue_Head(Queue_Mem_Struct_p sq)
{
	if(sq->MemFront==sq->MemRear)
		return Queue_Init_Value;
	else
		return sq->MemDataBuf[(sq->MemFront+1)%MAX_QUEUE_LEN];
}	



//////////////////////////
///出队
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
//入队
///////////////////////////
void Queue_En(Queue_Mem_Struct_p sq,elemtype data)
{
	if((sq->MemRear+1)%MAX_QUEUE_LEN==sq->MemFront)//队满，溢出
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
///入队溢出错误
/////////////////////////
void Queue_ERROverflowCallback(Queue_Mem_Struct_p sq,elemtype data)
{
	Queue_Del(sq);//出队，舍弃前面的数据
	Queue_En(sq,data);//新数据入队
}

