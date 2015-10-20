
extern "C"{
	#include "stm32f10x.h"
	#include "delay.h"
	#include "usart.h"

}

#include "IMU.h"

IMU IMU_;

u8 init(void)
{
	//SystemInit();  库中初始化代码中已经包含，若库版本太低则要加上
	Usart_Configuration();
	Usart_NVIC_Configuration();
	
	IMU_.Use_Acc_Gyro=1;
	IMU_.Use_Baro=1;
	IMU_.Use_Mag=1;
	delay_ms(100);
	IMU_.Init();
	return 1;
}



int main()
{
	u16 count=0;
	init();
	while(1)
	{
		++count;
		if(!IMU_.Update_Data_ACC_GYR())//更新失败 没有开启传感器或者通信失败，初始化通信
		{
			IMU_.Init();
		}
		if(count%10==0)
		{	
			count=0;
			printf("%d\n",IMU_.Get_ACC_Raw().yaw);
			
			
		}
		else if(count==100)
		{
			count=0;
		}
		delay_ms(10);
	}
}

