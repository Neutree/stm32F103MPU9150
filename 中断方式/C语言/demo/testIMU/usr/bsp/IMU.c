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
///��ȡIMU�ļ��ٶȡ��¶Ⱥͽ��ٶ�ֵ��IMUData���͵Ĳ�����
///ֻ�ǽ���ȡ����������˶�����
////////////////////////
void IMU_Add_Read_ACC_TEMP_GYR(IMUData* IMU)
{
	I2C_AddCMD_Read_Bytes(MPU6050_ADDRESS,ACCEL_XOUT_H,&IMU->acc_XH,14);//��ȡIMU���ٶȡ����ٶȡ�IMU�¶ȼ���ֵ
}



//////////////////////////
///��ȡIMUĳ���Ĵ�����ֵ
/////////////////////////
void IMU_Add_Read_Register(u8 device_addr,u8 register_addr, u8* data_read, u8 num)//�������豸��ַ���Ĵ�����ַ����ȡ�����ݴ�ŵĵ�ַ����Ҫ��ȡ���ݵĸ���)
{
		
	I2C_AddCMD_Read_Bytes(device_addr,register_addr,data_read,num);//���������µĶ�ȡһ���ֽڵ�����
}


/////////////////////////
///дIMU��ĳ���Ĵ���
/////////////////////////
void IMU_Add_Write_Register(u8 device_addr,u8 register_addr, u8* data_write, u8 num)//�������豸��ַ���Ĵ�����ַ����ȡ�����ݴ�ŵĵ�ַ����Ҫ��ȡ���ݵĸ���
{
	I2C_AddCMD_Write_Bytes(device_addr,register_addr,data_write,num);
}


//////////////////////////////////////
///����ʶ��IMU�豸ͨ���Ƿ��ڿ���״̬�����Ƿ���Կ�ʼ��һ�ζ�/д����
//////////////////////////////////////
I2C_Bool IMU_Is_Device_Ready()
{
	if(IIC_CMD_Queue.State==STATE_READY)
		return I2C_True;
	else
		return I2C_False;
}


//////////////////////////////////
///��ȡ�������Ķ��еĳ���
/////////////////////////////////
u16 IMU_Get_Queue_length(void)
{
	return IIC_CMD_Queue.MemLength;
}



//////////////////////////////////
///�ж��������Ķ��еĳ����Ƿ�Ϊ��
/////////////////////////////////
I2C_Bool IMU_Is_Queue_Empty(void)
{
	if(IIC_CMD_Queue.MemLength>0)
		return I2C_False;
	else
		return I2C_True;
}


/////////////////////////////////
///��ʼִ�ж����������
///@attention ��IMU_Is_Device_Ready()==I2C_True����IMU_Is_Queue_Empty()==I2C_Falseʱ����ִ��
////////////////////////////////
void IMU_Start_CMD_Queue()
{
	IIC_Start_Next_CMD(); //�ڱ����������ж����  �ж϶����Ƿ�Ϊ�պ��ж�״̬�Ƿ�ΪSTATE_READY��ͬʱ����ſ�ʼִ�ж����е�����
						 //if(IIC_CMD_Queue.State==STATE_READY && IIC_CMD_Queue.MemLength>0 )//IIC״̬Ϊ���У����Ҷ��в�Ϊ�գ�������ʼ�ź�
}


