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
	//SystemInit();  ���г�ʼ���������Ѿ�����������汾̫����Ҫ����
	Usart_Configuration();
	Usart_NVIC_Configuration();
//	printf("\r\nSystem Starting...\r\n");
	I2C_Init_Config();
//	printf("\r\nSystem Started\r\n");
	
	delay_s(1);//��ʱ1�룬�ϵ������ms����ܶ�mpu6050д����
	IMU_Init();
	
}

u8 IIC_read_Temp;
IMUData IMU;

int main()
{
	init();

//	IMU_Add_Read_ACC_TEMP_GYR(&IMU);//��ȡIMU���ٶȡ����ٶȡ�mpu6050�¶ȼ���ֵ
//	IMU_Start_CMD_Queue();
	while(1)
	{
		if(IMU_Is_Queue_Empty())//������пգ�����
		{
			IMU_Add_Read_ACC_TEMP_GYR(&IMU);
			
		}
		if(IMU_Is_Device_Ready() && (!IMU_Is_Queue_Empty()) )//IIC״̬Ϊ���У����Ҷ��в�Ϊ�գ�������ʼ�ź�
		{
			IMU_Start_CMD_Queue();
			
		}
		printf("\n%d\t%d\t%d\t%d\t%d\t%d\t%d\n",(s16)(IMU.imu_TH<<8|IMU.imu_TL),(s16)(IMU.acc_XH<<8|IMU.acc_XL ),(s16)(IMU.acc_YH<<8|IMU.acc_YL),(s16)(IMU.acc_ZH<<8|IMU.acc_ZL),(s16)(IMU.gyro_XH<<8|IMU.gyro_XL),(s16)(IMU.gyro_YH<<8|IMU.gyro_YL),(s16)(IMU.gyro_ZH<<8|IMU.gyro_ZL));
		printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",(s16)((u16)(MS561101BA_Value.C1_H<<8)|MS561101BA_Value.C1_L),MS561101BA_Value.C2_H<<8|MS561101BA_Value.C2_L,MS561101BA_Value.C3_H<<8|MS561101BA_Value.C3_L,MS561101BA_Value.C4_H<<8|MS561101BA_Value.C4_L,MS561101BA_Value.C5_H<<8|MS561101BA_Value.C5_L,MS561101BA_Value.C6_H<<8|MS561101BA_Value.C6_L,MS561101BA_Value.Pressure,IMU_Add_MS561101BA_Get_Temperature());
		while(!IMU_Is_Device_Ready() || (!IMU_Is_Queue_Empty()) );//IIC״̬Ϊ��Ϊ���У����Ҷ��в�Ϊ�գ��ȴ�
		IMU_Add_MS561101BA_DoConversion_Temp(MS561101BA_OSR_4096);
		IMU_Start_CMD_Queue();
		while(!IMU_Is_Device_Ready() || (!IMU_Is_Queue_Empty()) );//IIC״̬Ϊ��Ϊ���У����Ҷ��в�Ϊ�գ��ȴ�
		delay_ms(10);//��ʱ1s
		IMU_Add_MS561101BA_Receive_Temp();
		IMU_Start_CMD_Queue();
		while(!IMU_Is_Device_Ready() || (!IMU_Is_Queue_Empty()) );//IIC״̬Ϊ��Ϊ���У����Ҷ��в�Ϊ�գ��ȴ�
		
		IMU_Add_MS561101BA_DoConversion_Pressure(MS561101BA_OSR_4096);
		IMU_Start_CMD_Queue();
		while(!IMU_Is_Device_Ready() || (!IMU_Is_Queue_Empty()) );//IIC״̬Ϊ��Ϊ���У����Ҷ��в�Ϊ�գ��ȴ�
		delay_s(1);//��ʱ1s
		IMU_Add_MS561101BA_Receive_Pressure();
		IMU_Start_CMD_Queue();
		while(!IMU_Is_Device_Ready() || !IMU_Is_Queue_Empty());//�ȴ���ʼ������ִ�����
		IMU_Add_MS561101BA_Get_Pressure();
	}
}

