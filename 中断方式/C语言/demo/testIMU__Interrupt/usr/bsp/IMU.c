#include "IMU.h"



/*****************************************************************************************************************************/
										///////////////////////
										///IMU初始化，包括所有传感器的初始化
										//////////////////////
/*****************************************************************************************************************************/

void IMU_Init(IMUData* IMU)
{
	u8 IIC_Write_Temp;
	
	
	
	///////////////////////////////////////
	///MS561101BA气压计初始化--1 
	///////////////////////////////////////
	IMU_Add_MS561101BA_Reset();
	IMU_Start_CMD_Queue();
	while(!IMU_Is_Device_Ready() || !IMU_Is_Queue_Empty());//等待初始化命令执行完毕
//	printf("\r\nreset ok\r\n");
	delay_ms(50);//wait for the completion of MS561101BA Reset
	
	
	
	//////////////////////////////////////////
	///mpu9150/mpu6050加速度角速度初始化
	//////////////////////////////////////////
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
	
	
	
	////////////////////////////////////////
	///mpu9150 Slave磁力计 初始化
	///////////////////////////////////////
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
	
	
	///////////////////////////////////////
	///MS561101BA气压计初始化--2
	///////////////////////////////////////
	
	IMU_Add_MS561101BA_ReadPROM(IMU);//获取出厂校验值
	IMU_Start_CMD_Queue();
	while(!IMU_Is_Device_Ready() || !IMU_Is_Queue_Empty());//等待初始化命令执行完毕
//	printf("\r\nIMU Init ok\r\n");
	
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
	printf("\nx:%d\ty:%d\n",x,y);
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


/////////////////////////////////
///开始执行队列里的命令
///@attention 在IMU_Is_Device_Ready()==I2C_True并且IMU_Is_Queue_Empty()==I2C_False时才能执行
////////////////////////////////
void IMU_Start_CMD_Queue()
{
	IIC_Start_Next_CMD(); //在本函数中有判断语句  判断队列是否为空和判断状态是否为STATE_READY，同时满足才开始执行队列中的命令
						 //if(IIC_CMD_Queue.State==STATE_READY && IIC_CMD_Queue.MemLength>0 )//IIC状态为空闲，并且队列不为空，则发送起始信号
}


