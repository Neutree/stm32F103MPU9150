#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"
#include "iic.h"
#include "IMU.h"


//////////////////////////////////////
///variable
/////////////////////////////////////

void init(void)
{
	//SystemInit();  库中初始化代码中已经包含，若库版本太低则要加上
	Usart_Configuration();
	Usart_NVIC_Configuration();
//	printf("\r\nSystem Starting...\r\n");
	I2C_Init_Config();
//	printf("\r\nSystem Started\r\n");
	
	delay_s(1);//延时1秒，上电后至少ms后才能对mpu6050写命令
	IMU_Init();
	
}

u8 IIC_read_Temp;
IMUData IMU;

int main()
{
	init();

	IMU_Add_Read_ACC_TEMP_GYR(&IMU);//获取IMU加速度、角速度、mpu6050温度计数值
	IMU_Start_CMD_Queue();
	while(1)
	{
		if(IMU_Is_Queue_Empty())//命令队列空，出队
		{
			IMU_Add_Read_ACC_TEMP_GYR(&IMU);
		}
		if(IMU_Is_Device_Ready() && (!IMU_Is_Queue_Empty()) )//IIC状态为空闲，并且队列不为空，则发送起始信号
		{
			IMU_Start_CMD_Queue();
		}
		printf("\n%d\t%d\t%d\t%d\t%d\t%d\t%d\n",(s16)(IMU.imu_TH<<8|IMU.imu_TL),(s16)(IMU.acc_XH<<8|IMU.acc_XL ),(s16)(IMU.acc_YH<<8|IMU.acc_YL),(s16)(IMU.acc_ZH<<8|IMU.acc_ZL),(s16)(IMU.gyro_XH<<8|IMU.gyro_XL),(s16)(IMU.gyro_YH<<8|IMU.gyro_YL),(s16)(IMU.gyro_ZH<<8|IMU.gyro_ZL));

	}
}

