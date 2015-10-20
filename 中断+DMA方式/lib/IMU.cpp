 #include"IMU.h"
 
 ////////////////////////////
 ///构造函数
 ////////////////////////////
 IMU::IMU()
{
	Data.Mag_Exist=0;
	Data.Acc_Exist=0;
	Data.Gyro_Exist=0;
	Data.Baro_Exist=0;
	
	
	Use_Acc_Gyro = 0;
	Use_Mag = 0;
	Use_Baro = 0;
} 



///////////////////////////
///析构函数
///////////////////////////
IMU::~IMU()
{
	
}



//////////////////////////
///IMU初始化函数
///@retval -1:检测到硬件 -2:检测到部分硬件 -3:正在检测 -0:未检测到硬件
//////////////////////////
u8 IMU::Init()
{
	return IMU_Init(&this->Data,Use_Acc_Gyro,Use_Mag,Use_Baro);//调用HAL初始化
}



////////////////////////////////
///获取加速度原始值
///@retval 返回三轴的加速度值
///////////////////////////////
Three_Axis_Aviation_Int IMU::Get_ACC_Raw()
{
	Three_Axis_Aviation_Int temp;
	temp.pitch=((int16_t)(Data.acc_XH<<8)) | Data.acc_XL;
	temp.roll=((int16_t)(Data.acc_YH<<8)) | Data.acc_YL;
	temp.yaw=((int16_t)(Data.acc_ZH<<8)) | Data.acc_ZL;
	return temp;
}


////////////////////////////////
///获取加速度计的温度原始值
///@retval 返回加速度计的温度值
///////////////////////////////
int IMU::Get_ACC_TEMP_Raw()
{
	return ((int16_t)(Data.imu_TH<<8))|Data.imu_TL;
}



////////////////////////////////
///获取角速度原始值
///@retval 返回三轴的角速度值 
///////////////////////////////
Three_Axis_Aviation_Int IMU::Get_GYR_Raw()
{
	Three_Axis_Aviation_Int temp;
	temp.pitch=((int16_t)(Data.gyro_XH<<8))|Data.gyro_XL;
	temp.roll=((int16_t)(Data.gyro_YH<<8))|Data.gyro_YL;
	temp.yaw=((int16_t)(Data.gyro_ZH<<8))|Data.gyro_ZL;
	return temp;
}



////////////////////////////////
///获取磁力计原始值
///@retval 返回三轴的磁力计原始值
///////////////////////////////
Three_Axis_Aviation_Int IMU::Get_Compass_Raw()
{
	Three_Axis_Aviation_Int temp;
	temp.pitch=((int16_t)(Data.mag_XH<<8))|Data.mag_XL;
	temp.roll=((int16_t)(Data.mag_YH<<8))|Data.mag_YL;
	temp.yaw=((int16_t)(Data.mag_ZH<<8))|Data.mag_ZL;
	return temp;
}



////////////////////////////////
///获取气压计原始值
///@retval 返回气压计原始值
///////////////////////////////
int32_t IMU::Get_Baro_Raw()
{
	IMU_Add_MS561101BA_Get_Pressure(&Data);
	return Data.MS561101BA_Value.Pressure;
}





//////////////////////////////
///从硬件更新加速度、角速度值到内存
///@return -0:发送更新数据失败，没有开启跟新该传感器的开关或者通讯错误 1:命令将会发送（具体的发送时间取决于队列中的排队的命令的数量）
//////////////////////////////
u8 IMU::Update_Data_ACC_GYR()
{
	
	if(Use_Acc_Gyro)
	{
		return HAL_Update_Data_ACC_GYR(&this->Data);
	}
	else
		return 0;
}
	


//////////////////////////////
///从硬件更新磁力计数据到内存
///@return -0:发送更新数据失败，没有开启跟新该传感器的开关 1:命令将会发送（具体的发送时间取决于队列中的排队的命令的数量）
///////////////////////////////
u8 IMU::Update_Data_Compass()
{
	if(Use_Mag)
	{
		if(!Data.Mag_Exist)//检测到的是硬件不存在，但是读取到了数据
		{
			static u8 flag=0;
			++flag;
			if(flag==1)
				Data.mag_ZH=0x7f;
			else
			{
				if(Data.mag_ZH!=0x7f)//读取到了数据
				{
					flag=0;
					Init();
	//				Data.Acc_Exist=1;//硬件存在
				}
			}
			
		}
		IMU_Add_Read_Compass(&this->Data);//从硬件更新磁力计值
		IMU_Start_CMD_Queue();//开始命令队列
	}
	else
		return 0;
	return 1;
}




/////////////////////////////
///从硬件更新IMU加速度、加速度计温度、角速度、磁力计数据
///@return -0:发送更新数据失败，没有开启跟新该传感器的开关 1:命令将会发送（具体的发送时间取决于队列中的排队的命令的数量）
////////////////////////////
u8 IMU::Update_Data_ACC_GYR_Compass()
{
	if(Use_Mag&&Use_Acc_Gyro)
	{
		if(!Data.Acc_Exist)//检测到的是硬件不存在，但是读取到了数据
		{
			static u8 flag=0;
			++flag;
			if(flag==1)
				Data.acc_ZH=0x7f;
			else
			{
				if(Data.acc_ZH!=0x7f)//读取到了数据
				{
					flag=0;
					Init();
	//				Data.Acc_Exist=1;//硬件存在
				}
			}
		}
		if(!Data.Gyro_Exist)//检测到的是硬件不存在，但是读取到了数据
		{
			static u8 flag2=0;
			++flag2;
			if(flag2==1)
				Data.gyro_ZH=0x7f;
			else
			{
				if(Data.gyro_ZH!=0x7f)//读取到了数据
				{
					flag2=0;
					Init();
	//				Data.Gyro_Exist=1;//硬件存在
				}
			}
		}
		if(!Data.Mag_Exist)//检测到的是硬件不存在，但是读取到了数据
		{
			static u8 flag=0;
			++flag;
			if(flag==1)
				Data.mag_ZH=0x7f;
			else
			{
				if(Data.mag_ZH!=0x7f)//读取到了数据
				{
					flag=0;
					Init();
	//				Data.Acc_Exist=1;//硬件存在
				}
			}
		}
		IMU_Add_Read_ACC_TEMP_GYR(&this->Data);//从硬件更新加速度、加速度计温度值、角速度值
		IMU_Add_Read_Compass(&this->Data);//从硬件更新磁力计值
		IMU_Start_CMD_Queue();//开始命令队列
	}
	else
		return 0;
	return 1;
}



//////////////////////////////
///从硬件更新气压值数据
///@attention 由于硬件限制，此函数需在大于10ms的间隔才能调用，在调用2次后获得一次有效数据
///@return -0:发送更新数据失败，没有开启跟新该传感器的开关 1:命令将会发送（具体的发送时间取决于队列中的排队的命令的数量）
/////////////////////////////
u8 IMU::Update_Data_Baro()//从硬件更新气压计值
{
	if(Use_Baro)
	{
		if(!Data.Baro_Exist)//检测到的是硬件不存在，但是读取到了数据
		{
			static u8 flag=0;
			++flag;
			if(flag==1)
				Data.Baro_Exist=0x7f;
			else
			{
				if(Data.Baro_Exist!=0x7f)//读取到了数据
				{
					flag=0;
					Init();
	//				Data.Acc_Exist=1;//硬件存在
				}
			}
		}
		static u8 baro_count=0;
		if(baro_count==0)
		{
			IMU_Add_MS561101BA_DoConversion_Temp(MS561101BA_OSR_4096);//启动气压计温度转换，必须的，计算气压需要
		}
		else if(baro_count==1)
		{
			IMU_Add_MS561101BA_Receive_Temp(&this->Data);//接收转换好的气压计温度值
			IMU_Add_MS561101BA_DoConversion_Pressure(MS561101BA_OSR_4096);//启动气压计的气压转换
		}
		else if(baro_count==2)
		{
			IMU_Add_MS561101BA_Receive_Pressure(&this->Data);//接收转换好的气压计的气压值
			IMU_Add_MS561101BA_DoConversion_Temp(MS561101BA_OSR_4096);//启动气压计温度转换，必须的，计算气压需要
			baro_count=0;
		}
		IMU_Start_CMD_Queue();//开始命令队列
		++baro_count;//计数，因为气压计的读取高度过程化，需要先转化，再读取
	}
	else
		return 0;
	return 1;
}

////////////////////////////////
///获取结算过后的三轴的角度值
///@retval 返回结算过后的三轴的角度值
///////////////////////////////
Three_Axis_Aviation_Double IMU::Get_Angle()
{
	Three_Axis_Aviation_Double temp;
	return temp;
}



////////////////////////////////
///获取经计算过后的高度值
///@retval 返回经计算过后的高度值*100
///////////////////////////////
int32_t IMU::Get_Altidude()
{
	return 44330.0*(1.0 - pow(Data.MS561101BA_Value.Pressure/101325.0,0.190295));
}

