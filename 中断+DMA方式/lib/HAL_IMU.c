/**
* @file 
* @author Chenzhangdi
* @version v1.0
* @date 2015-08-28
* @pre First initialize the SystemClock eg:72M 
* @brief  
*
* @bug
* @warning   
* @copyright 
* @attention 
*/

#include "HAL_IMU.h"
#include "usart.h"


/*****************************************************************************************************************************/
										///////////////////////
										///IMU初始化，包括所有传感器的初始化
										//////////////////////
/*****************************************************************************************************************************/
////////////////////////////////
///加速度计、角速度计初始化
////////////////////////////////
void Acc_Gyro_Init(IMUData* IMU)
{
	u8 IIC_Write_Temp;
	u16 time_out=0;
	IMU->Acc_Exist=2;//设置标志位，表明正在初始化
	IMU->Gyro_Exist=2;
	
	//向命令队列里添加命令
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
	I2C_AddCMD_Write_Bytes(MPU6050_ADDRESS,GYRO_CONFIG,&IIC_Write_Temp,1);//+-2000 °/s
	IIC_Write_Temp=9;
	I2C_AddCMD_Write_Bytes(MPU6050_ADDRESS,ACCEL_CONFIG,&IIC_Write_Temp,1);//+-4g   5Hz
	//开始执行命令
	IMU_Start_CMD_Queue();
	//出错检测
	while(!IMU_Is_Device_Ready() || !IMU_Is_Queue_Empty())//等待初始化命令执行完毕
	{
		if(IIC_CMD_Queue.State==STATE_ERROR)//总线出错了//应答失败，硬件可能未连接正确
		{	
			IMU->Acc_Exist=0;//标志硬件不存在
			IMU->Gyro_Exist=0;//标志硬件不存在
			IIC_CMD_Queue.State=STATE_READY;//清除错误标志
			I2C_Init_Config();
			while(!IMU_Is_Queue_Empty())//因为没有应答，说明硬件不存在，可以将剩下在队列里的命令丢弃
			{
				Queue_Del(&IIC_CMD_Queue);
			}
			break;//超时，返回错误信息
			
		}
		++time_out;
		if(time_out>=65534)
		{	
			IMU->Acc_Exist=0;//标志硬件不存在
			IMU->Gyro_Exist=0;//标志硬件不存在
			IIC_CMD_Queue.State=STATE_READY;//清除错误标志
			I2C_Init_Config();
			while(!IMU_Is_Queue_Empty())//因为没有应答，说明硬件不存在，可以将剩下在队列里的命令丢弃
			{
				Queue_Del(&IIC_CMD_Queue);
			}
			break;//超时，返回错误信息
			
		}
	}
	if(IMU->Acc_Exist==2)
		IMU->Acc_Exist=1;
	if(IMU->Gyro_Exist==2)
		IMU->Gyro_Exist=1;
}

////////////////////////////////
///磁力计初始化
////////////////////////////////
void Compass_Init(IMUData* IMU)
{
	u8 IIC_Write_Temp;
	u16 time_out=0;
	IMU->Mag_Exist=2;//设置标志位，表明正在初始化
	
	//向命令队列里添加命令
	IIC_Write_Temp=0x0D;
	I2C_AddCMD_Write_Bytes(MPU6050_ADDRESS,I2C_MST_CTRL,&IIC_Write_Temp,1);// config the mpu6050  master bus rate as 400kHz
	IIC_Write_Temp=0x00;
	I2C_AddCMD_Write_Bytes(MPU6050_ADDRESS,USER_CTRL,&IIC_Write_Temp,1);//Disable mpu6050 Master mode
	IIC_Write_Temp=0x02;
	I2C_AddCMD_Write_Bytes(MPU6050_ADDRESS,INT_PIN_CFG,&IIC_Write_Temp,1); //为了直接接到辅助IIC接口    Pass-Through Mode：I2C_MST_EN=0(USER_CTRL bit5)  I2C_BYPASS_EN =1(INT_PIN_CFG bit1)即可直接接到旁路IIC，注意修改IIC读/写的从机地址
																		//Enable bypass mode
	IIC_Write_Temp=0x74;   //01110100B
	I2C_AddCMD_Write_Bytes(HMC5883_ADDRESS,HMC5883_Config_RA,&IIC_Write_Temp,1); //Config Register A  :number of samples averaged->8  Data Output rate->30Hz
	IIC_Write_Temp=0x20;   //00100000B
	I2C_AddCMD_Write_Bytes(HMC5883_ADDRESS,HMC5883_Config_RB,&IIC_Write_Temp,1);//Config Register B:  Gain Configuration as : Sensor Field Range->(+-)1.3Ga ; Gain->1090LSB/Gauss; Output Range->0xF800-0x07ff(-2048~2047)
	IIC_Write_Temp=0x00;   //00000000B
	I2C_AddCMD_Write_Bytes(HMC5883_ADDRESS,HMC5883_Mode,&IIC_Write_Temp,1);//Config Mode as: Continous Measurement Mode
	
	
	//开始执行命令
	IMU_Start_CMD_Queue();
	//出错检测
	while(!IMU_Is_Device_Ready() || !IMU_Is_Queue_Empty())//等待初始化命令执行完毕
	{
		if(IIC_CMD_Queue.State==STATE_ERROR)//总线出错了//应答失败，硬件可能未连接正确
		{	
			IMU->Mag_Exist=0;//标志硬件不存在
			IIC_CMD_Queue.State=STATE_READY;//清除错误标志
			I2C_Init_Config();
			while(!IMU_Is_Queue_Empty())//因为没有应答，说明硬件不存在，可以将剩下在队列里的命令丢弃
			{
				Queue_Del(&IIC_CMD_Queue);
			}
			break;//超时，返回错误信息
			
		}
		++time_out;
		if(time_out>=65534)
		{	
			IMU->Mag_Exist=0;//标志硬件不存在
			IIC_CMD_Queue.State=STATE_READY;//清除错误标志
			I2C_Init_Config();
			while(!IMU_Is_Queue_Empty())//因为没有应答，说明硬件不存在，可以将剩下在队列里的命令丢弃
			{
				Queue_Del(&IIC_CMD_Queue);
			}
			break;//超时，返回错误信息
			
		}
	}
	if(IMU->Mag_Exist==2)
		IMU->Mag_Exist=1;

}

////////////////////////////////
///气压计初始化
////////////////////////////////
void Baro_Init(IMUData* IMU)
{
	u16 time_out=0;
	IMU->Baro_Exist=2;//标志设置，表示现在正在初始化气压计中
	IMU_Add_MS561101BA_Reset();
	IMU_Start_CMD_Queue();
	while(!IMU_Is_Device_Ready() || !IMU_Is_Queue_Empty())//等待初始化命令执行完毕
	{
		if(IIC_CMD_Queue.State==STATE_ERROR)//总线出错了//应答失败，硬件可能未连接正确
		{	
			IIC_CMD_Queue.State=STATE_READY;//清除错误标志
			IMU->Baro_Exist=0;//标志硬件不存在
			I2C_Init_Config();
			while(!IMU_Is_Queue_Empty())//因为没有应答，说明硬件不存在，可以将剩下在队列里的命令丢弃
			{
				Queue_Del(&IIC_CMD_Queue);
			}
			break;//超时
			
		}
		++time_out;
		if(time_out>=65534)
		{	
			IMU->Baro_Exist=0;//标志硬件不存在
			IIC_CMD_Queue.State=STATE_READY;//清除错误标志
			I2C_Init_Config();
			while(!IMU_Is_Queue_Empty())//因为没有应答，说明硬件不存在，可以将剩下在队列里的命令丢弃
			{
				Queue_Del(&IIC_CMD_Queue);
			}
			break;//超时
			
		}
	}
	if(IMU->Baro_Exist==2)
		IMU->Baro_Exist=1;
	delay_us(1900);
	if(IMU->Baro_Exist==1)	//气压计存在
	{
		
		IMU->Baro_Exist=2;//设置标志位，表明正在初始化
		
		IMU_Add_MS561101BA_ReadPROM(IMU);//获取出厂校验值*/
		IMU_Start_CMD_Queue();
		
		while(!IMU_Is_Device_Ready() || !IMU_Is_Queue_Empty())//等待初始化命令执行完毕
		{
			if(IIC_CMD_Queue.State==STATE_ERROR)//总线出错了//应答失败，硬件可能未连接正确
			{	
				IMU->Baro_Exist=0;//标志硬件不存在
				IIC_CMD_Queue.State=STATE_READY;//清除错误标志
				I2C_Init_Config();
				while(!IMU_Is_Queue_Empty())//因为没有应答，说明硬件不存在，可以将剩下在队列里的命令丢弃
				{
					Queue_Del(&IIC_CMD_Queue);
				}
				break;//超时，返回错误信息
				
			}
			++time_out;
			if(time_out>=65534)
			{	
				IMU->Baro_Exist=0;//标志硬件不存在
				IIC_CMD_Queue.State=STATE_READY;//清除错误标志
				I2C_Init_Config();
				while(!IMU_Is_Queue_Empty())//因为没有应答，说明硬件不存在，可以将剩下在队列里的命令丢弃
				{
					Queue_Del(&IIC_CMD_Queue);
				}
				break;//超时，返回错误信息
				
			}
		}
		if(IMU->Baro_Exist==2)
			IMU->Baro_Exist=1;
	}
}

///////////////////////////
///IMU初始化，初始化之前需要等待一段时间，等IMU上电启动。
///@retval -1:初始化成功 -2:部分成功 -3:正在初始化  -0:初始化失败
///@param 是否使用响应传感器  加速度角速度  磁力计  气压计  0：不使用 1：使用
///////////////////////////
u8 IMU_Init(IMUData* IMU,u8 useAccGyro,u8 useMag,u8 useBaro)
{

	u8 exist_flag=0;
	
	if(IMU->Mag_Exist==2||IMU->Acc_Exist==2||IMU->Gyro_Exist==2|| IMU->Baro_Exist==2)//如果是正在初始化并且还没有初始化完成
	{
			return 3;
	}
	
	
	I2C_Init_Config();//IIC相关硬件初始化
	
//	delay_ms(10);//延时10毫秒，可以适当增加，上电后至少ms后才能对mpu6050写命令
	
	///////////////////////////////////////
	///MS561101BA气压计初始化--1 
	///////////////////////////////////////
	if(useBaro)
	{
		Baro_Init(IMU);
	}
	
	
	//////////////////////////////////////////
	///mpu9150/mpu6050加速度角速度初始化
	//////////////////////////////////////////
	if(useAccGyro)
	{
		Acc_Gyro_Init(IMU);
	}
	
	
	
	////////////////////////////////////////
	///mpu9150 Slave磁力计 初始化
	///////////////////////////////////////
	if(useMag)
	{
		Compass_Init(IMU);
	}
		
	
	
//	printf("%d\t%d\t%d\n",IMU->Acc_Exist,IMU->Mag_Exist,IMU->Baro_Exist);
//	printf("\r\nIMU Init ok\r\n");
	
	if(useAccGyro&&IMU->Acc_Exist)//使用该传感器并且检测到了
		exist_flag|=0x01;
	else if(!useAccGyro)//不使用
		exist_flag|=0x01;
	else                //使用，但是不存在
		exist_flag&=0xfe;
	
	if(useMag&&IMU->Mag_Exist)//使用该传感器并且检测到了
		exist_flag|=0x02;
	else if(!useMag)//不使用
		exist_flag|=0x02;
	else                //使用，但是不存在
		exist_flag&=0xfd;
	
	if(useBaro&&IMU->Baro_Exist)//使用该传感器并且检测到了
		exist_flag|=0x04;
	else if(!useBaro)//不使用
		exist_flag|=0x04;
	else                //使用，但是不存在
		exist_flag&=0xfb;
	
	if((exist_flag&0x07)==0x07)//都符合情况
		return 1;
	else if((exist_flag&0x07)==0x00)//都不符合情况
		return 0;
	else                             //部分符合
		return 2;
}



/*****************************************************************************************************************************/
									///////////////////////////////////////
									///加速度、角速度相关
									//////////////////////////////////////
/*****************************************************************************************************************************/
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

//////////////////////////////
///从硬件更新加速度、角速度值到内存
///@return -0:发送更新数据失败 1:命令将会发送（具体的发送时间取决于队列中的排队的命令的数量）
//////////////////////////////
u8 HAL_Update_Data_ACC_GYR(IMUData*  IMU)
{
	static u8 flag=0;
//	static u8 flag2=0;
	static u8 temp=0;
//	static u8 temp2;
	u8 status=0;
	if(!IMU->Acc_Exist)//检测到的是硬件不存在，但是I2C总线完好，即实际是存在的，并读取到了数据
	{
		++flag;
		if(flag==1)
		{
			IMU->acc_ZL=IMU->acc_ZL+1;
			temp=IMU->acc_ZL;
		}
		else
		{
			if(IMU->acc_ZL!=temp)//读取到了数据,说明硬件存在，只是传感器未进行初始化
			{
				flag=0;
				Acc_Gyro_Init(IMU);//可以正常通讯，不需要复位iic硬件，只需对传感器发送初始化命令 
			}
		}
	}
	//这里加速度计和角速度计是一个模块，所以一个就行了
//	if(!IMU->Gyro_Exist)//检测到的是硬件不存在，但是读取到了数据
//	{
//		
//		++flag2;
//		if(flag2==1)
//		{
//			IMU->gyro_ZH+=1;
//			temp2=IMU->gyro_ZH;
//		}
//		else
//		{
//			if(IMU->gyro_ZH!=temp2)//读取到了数据,说明硬件存在，只是传感器未进行初始化
//			{
//				flag2=0;
//				Acc_Gyro_Init(IMU);//可以正常通讯，不需要复位iic硬件，只需对传感器发送初始化命令 
//			}
//		}
//	}
		
	IMU_Add_Read_ACC_TEMP_GYR(IMU);//从硬件更新加速度、加速度计温度值、角速度值
	status = IMU_Start_CMD_Queue();//开始命令队列
	if(status==0)//失败
	{
		flag=0;
//		flag2=0;
	}
	return status;
}


/*****************************************************************************************************************************/
									///////////////////////////
									///磁力计相关
									///////////////////////////
/*****************************************************************************************************************************/

////////////////////////////////
///gain the tree aixs compass data
////////////////////////////////
void IMU_Add_Read_Compass(IMUData* IMU)
{
	I2C_AddCMD_Read_Bytes(HMC5883_ADDRESS,HMC5883_XOUT_M,&(IMU->mag_XH),6);//向队列添加新的读取字节的命令
}

///////////////////////////////////
///Calculate the Heading of Compass 
///@return Heading Defrees
//////////////////////////////////
double calculateHeading(IMUData* IMU)
{
	double headingDegrees,headingRadians;
	int x,y/*,z*/;
	x=(s16)(IMU->mag_XH<<8|IMU->mag_XL);
	y=(s16)(IMU->mag_YH<<8|IMU->mag_YL);
//	z=(int)(IMU->mag_ZH<<8|IMU->mag_ZL);
	
  headingRadians = atan2((double)((y)/*-offsetY*/),(double)((x)/*-offsetX*/));
  // Correct for when signs are reversed.
  if(headingRadians < 0)
    headingRadians += 2*PI;
 
  headingDegrees = headingRadians * 180/M_PI;
  headingDegrees += MagnetcDeclination; //the magnetc-declination angle 
 
  // Check for wrap due to addition of declination.
  if(headingDegrees > 360)
    headingDegrees -= 360;
 
  return headingDegrees;
}




/*****************************************************************************************************************************/
									///////////////////////////
									///气压计相关
									///////////////////////////
/*****************************************************************************************************************************/



////////////////////////////
///气压计复位
///////////////////////////
void IMU_Add_MS561101BA_Reset()
{
	I2C_AddCMD_Write_CMD_Byte(MS561101BA_ADDR,MS561101BA_RESET);//气压计上电复位
}



/////////////////////////
///获取气压计出厂校准值
///上电复位后应该获取
////////////////////////
void IMU_Add_MS561101BA_ReadPROM(IMUData* IMU)
{
	int i;
	for(i=0;i<MS561101BA_PROM_REG_COUNT;++i)
	{
		I2C_AddCMD_Read_Bytes(MS561101BA_ADDR,MS561101BA_PROM_BASE_ADDR+i*MS561101BA_PROM_REG_SIZE,(&IMU->MS561101BA_Value.C1_H)+i*MS561101BA_PROM_REG_SIZE,2);
	}
}


//////////////////////////////////
///启动气压计温度转换
///@attention 注意:气压计的数据读取相当于串行的，开启读取温度后最大9.04ms(时间取决于OSR，精度越高需要时间越多)后才可获取气压计温度值
//////////////////////////////////
void IMU_Add_MS561101BA_DoConversion_Temp(u8 OSR)
{
	I2C_AddCMD_Write_CMD_Byte(MS561101BA_ADDR,MS561101BA_D2+OSR);
}




//////////////////////////////////
///启动气压计气压值转换
///@attention 注意:气压计的数据读取相当于串行的，开启读取气压后最大9.04ms(时间取决于OSR，精度越高需要时间越多)后才可获取气压计气压值
//////////////////////////////////
void IMU_Add_MS561101BA_DoConversion_Pressure(u8 OSR)
{
	I2C_AddCMD_Write_CMD_Byte(MS561101BA_ADDR,MS561101BA_D1+OSR);
}





////////////////////////////////////
///获取气压计的温度值
///@pre 已经启动转换并且转换完毕（距离开始转换的时间大于转换需要的时间） 
////////////////////////////////////
void IMU_Add_MS561101BA_Receive_Temp(IMUData* IMU)
{
	I2C_AddCMD_Read_Bytes(MS561101BA_ADDR,0,&IMU->MS561101BA_Value.D2_2,3);
}





////////////////////////////////////
///获取气压计的气压值
///@pre 已经启动转换并且转换完毕（距离开始转换的时间大于转换需要的时间） 
////////////////////////////////////
void IMU_Add_MS561101BA_Receive_Pressure(IMUData* IMU)
{
	I2C_AddCMD_Read_Bytes(MS561101BA_ADDR,0,&IMU->MS561101BA_Value.D1_2,3);
}





//////////////////////////////////
///获取气压计的温度值
///@pre 温度转换、温度原始值获取完毕
//////////////////////////////////
u32 IMU_Add_MS561101BA_Get_Temperature(IMUData* IMU)
{
	int64_t deltaTemp = ( (u32)(IMU->MS561101BA_Value.D2_2<<16)|(u32)(IMU->MS561101BA_Value.D2_1<<8)|IMU->MS561101BA_Value.D2_2 )   -   ( (int32_t)(IMU->MS561101BA_Value.C5_H<<16) |(int32_t)(IMU->MS561101BA_Value.C5_L<<8) ) ;
	int32_t TEMP = 2000 + ((deltaTemp*(IMU->MS561101BA_Value.C6_H<<8|IMU->MS561101BA_Value.C6_L))>>23);
	if(TEMP<2000)
	 {
		 TEMP = TEMP - ((deltaTemp*deltaTemp)>>31);
	 }
	 return TEMP;
}





//////////////////////////////////
///获取气压计的气压值
///@pre 温度转换、温度原始值获取完毕 并且 气压转换、气压原始值获取完毕
//////////////////////////////////
int32_t IMU_Add_MS561101BA_Get_Pressure(IMUData* IMU)
{
	int64_t OFF2=0;
	int64_t SENS2=0;
	int64_t dT=( (u32)(IMU->MS561101BA_Value.D2_2<<16)|(u32)(IMU->MS561101BA_Value.D2_1<<8)|IMU->MS561101BA_Value.D2_2 )   -   ( (int32_t)(IMU->MS561101BA_Value.C5_H<<16) |(int32_t)(IMU->MS561101BA_Value.C5_L<<8) ) ;
	int64_t OFF=(((int64_t)(IMU->MS561101BA_Value.C2_H<<8|IMU->MS561101BA_Value.C2_L))<<16) + ( ((int64_t)( (IMU->MS561101BA_Value.C4_H<<8|IMU->MS561101BA_Value.C4_L) * dT)) >>7 );
	int64_t SENS=((int64_t)(IMU->MS561101BA_Value.C1_H<<8|IMU->MS561101BA_Value.C1_L)<<15) + ( ((int64_t)(IMU->MS561101BA_Value.C3_H<<8|IMU->MS561101BA_Value.C3_L)*dT) >>8 );
	int32_t TEMP=2000 + ((dT*(IMU->MS561101BA_Value.C6_H<<8|IMU->MS561101BA_Value.C6_L))>>23);
	
	//SECOND ORDER TEMPERATURE COMPENSATION
	if(TEMP<2000)
	{
		OFF2=(5*(TEMP-2000)*(TEMP-2000))>>1;
		SENS2=(5*(TEMP-2000)*(TEMP-2000))>>2;
		if(TEMP<-1500)
		{
			OFF2 = OFF2 + 7*(TEMP+1500)*(TEMP+1500);
			SENS2 = SENS2 + ((11*(TEMP+1500)*(TEMP+1500))>>1);
		}
	}
	OFF-=OFF2;
	SENS-=SENS2;
	IMU->MS561101BA_Value.Pressure = ( ( ((IMU->MS561101BA_Value.D1_2<<16|IMU->MS561101BA_Value.D1_1<<8|IMU->MS561101BA_Value.D1_0) *SENS) >>21) - OFF)>>15;
	return IMU->MS561101BA_Value.Pressure;
}









/*****************************************************************************************************************************/
									/////////////////////////////////////////////////
									///命令队列操作查看、IMU总线状态检查等其它
									///////////////////////////////////////////////////
/*****************************************************************************************************************************/
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


///开始执行队列里的命令
///@retval -1:成功发送开始信号，可能会执行成功 -2:队列为空，无需开始 -0:发送成功信号失败,I2C总线通信错误
////////////////////////////////
u8 IMU_Start_CMD_Queue()
{
	return IIC_Start_Next_CMD(); //在本函数中有判断语句  判断队列是否为空和判断状态是否为STATE_READY，同时满足才开始执行队列中的命令
						 //if(IIC_CMD_Queue.State==STATE_READY && IIC_CMD_Queue.MemLength>0 )//IIC状态为空闲，并且队列不为空，则发送起始信号
}


