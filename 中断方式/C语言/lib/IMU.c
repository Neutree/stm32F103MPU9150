#include "IMU.h"


void IMU_Init()
{
	u8 IIC_Write_Temp;
	IIC_Write_Temp=0;
	I2C_AddCMD_Write_Bytes(MPU6050_ADDRESS,PWR_MGMT_1,&IIC_Write_Temp,1);
	IIC_Write_Temp=2;
	I2C_AddCMD_Write_Bytes(MPU6050_ADDRESS,INT_PIN_CFG,&IIC_Write_Temp,1);
	IIC_Write_Temp=7;
	I2C_AddCMD_Write_Bytes(MPU6050_ADDRESS,SMPLRT_DIV,&IIC_Write_Temp,1);
	IIC_Write_Temp=7;
	I2C_AddCMD_Write_Bytes(MPU6050_ADDRESS,USER_CTRL,&IIC_Write_Temp,1);
	IIC_Write_Temp=6;
	I2C_AddCMD_Write_Bytes(MPU6050_ADDRESS,CONFIG,&IIC_Write_Temp,1);
	IIC_Write_Temp=0x18;
	I2C_AddCMD_Write_Bytes(MPU6050_ADDRESS,GYRO_CONFIG,&IIC_Write_Temp,1);
	IIC_Write_Temp=1;
	I2C_AddCMD_Write_Bytes(MPU6050_ADDRESS,ACCEL_CONFIG,&IIC_Write_Temp,1);
}

/////////////////////////
///获取IMU的加速度、温度和角速度值到IMUData类型的参数中
///只是将读取的命令放入了队列中
////////////////////////
void IMU_Add_Read_ACC_TEMP_GYR(IMUData* IMU)
{
	I2C_AddCMD_Read_Bytes(MPU6050_ADDRESS,ACCEL_XOUT_H,&IMU->acc_XH,14);//获取IMU加速度、角速度、IMU温度计数值
}



//////////////////////////
///读取IMU某个寄存器的值
/////////////////////////
void IMU_Add_Read_Register(u8 device_addr,u8 register_addr, u8* data_read, u8 num)//参数：设备地址，寄存器地址，读取的数据存放的地址，需要读取数据的个数)
{
		
	I2C_AddCMD_Read_Bytes(device_addr,register_addr,data_read,num);//向队列添加新的读取一个字节的命令
}


/////////////////////////
///写IMU的某个寄存器
/////////////////////////
void IMU_Add_Write_Register(u8 device_addr,u8 register_addr, u8* data_write, u8 num)//参数：设备地址，寄存器地址，读取的数据存放的地址，需要读取数据的个数
{
	I2C_AddCMD_Write_Bytes(device_addr,register_addr,data_write,num);
}


//////////////////////////////////////
///检查标识与IMU设备通信是否处于空闲状态，即是否可以开始下一次读/写命令
//////////////////////////////////////
I2C_Bool IMU_Is_Device_Ready()
{
	if(IIC_CMD_Queue.State==STATE_READY)
		return I2C_True;
	else
		return I2C_False;
}


//////////////////////////////////
///获取填充命令的队列的长度
/////////////////////////////////
u16 IMU_Get_Queue_length(void)
{
	return IIC_CMD_Queue.MemLength;
}



//////////////////////////////////
///判断填充命令的队列的长度是否为空
/////////////////////////////////
I2C_Bool IMU_Is_Queue_Empty(void)
{
	if(IIC_CMD_Queue.MemLength>0)
		return I2C_False;
	else
		return I2C_True;
}


/////////////////////////////////
///开始执行队列里的命令
///@attention 在IMU_Is_Device_Ready()==I2C_True并且IMU_Is_Queue_Empty()==I2C_False时才能执行
////////////////////////////////
void IMU_Start_CMD_Queue()
{
	IIC_Start_Next_CMD(); //在本函数中有判断语句  判断队列是否为空和判断状态是否为STATE_READY，同时满足才开始执行队列中的命令
						 //if(IIC_CMD_Queue.State==STATE_READY && IIC_CMD_Queue.MemLength>0 )//IIC状态为空闲，并且队列不为空，则发送起始信号
}


