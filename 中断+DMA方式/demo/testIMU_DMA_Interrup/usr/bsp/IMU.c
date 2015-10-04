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

#include "IMU.h"



/*****************************************************************************************************************************/
										///////////////////////
										///IMU��ʼ�����������д������ĳ�ʼ��
										//////////////////////
/*****************************************************************************************************************************/

void IMU_Init(IMUData* IMU)
{
	u8 IIC_Write_Temp;
	
	
	///////////////////////////////////////
	///MS561101BA��ѹ�Ƴ�ʼ��--1 
	///////////////////////////////////////
	IMU_Add_MS561101BA_Reset();
	IMU_Start_CMD_Queue();
	while(!IMU_Is_Device_Ready() || !IMU_Is_Queue_Empty());//�ȴ���ʼ������ִ�����
//	printf("\r\nreset ok\r\n");
	delay_ms(50);//wait for the completion of MS561101BA Reset
	
	
	
	//////////////////////////////////////////
	///mpu9150/mpu6050���ٶȽ��ٶȳ�ʼ��
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
	///mpu9150 Slave������ ��ʼ��
	///////////////////////////////////////
	IIC_Write_Temp=0x0D;
	I2C_AddCMD_Write_Bytes(MPU6050_ADDRESS,I2C_MST_CTRL,&IIC_Write_Temp,1);// config the mpu6050  master bus rate as 400kHz
	IIC_Write_Temp=0x00;
	I2C_AddCMD_Write_Bytes(MPU6050_ADDRESS,USER_CTRL,&IIC_Write_Temp,1);//Disable mpu6050 Master mode
	IIC_Write_Temp=0x02;
	I2C_AddCMD_Write_Bytes(MPU6050_ADDRESS,INT_PIN_CFG,&IIC_Write_Temp,1); //Ϊ��ֱ�ӽӵ�����IIC�ӿ�    Pass-Through Mode��I2C_MST_EN=0(USER_CTRL bit5)  I2C_BYPASS_EN =1(INT_PIN_CFG bit1)����ֱ�ӽӵ���·IIC��ע���޸�IIC��/д�Ĵӻ���ַ
                                                                        //Enable bypass mode
	IIC_Write_Temp=0x74;   //01110100B
	I2C_AddCMD_Write_Bytes(HMC5883_ADDRESS,HMC5883_Config_RA,&IIC_Write_Temp,1); //Config Register A  :number of samples averaged->8  Data Output rate->30Hz
	IIC_Write_Temp=0x20;   //00100000B
	I2C_AddCMD_Write_Bytes(HMC5883_ADDRESS,HMC5883_Config_RB,&IIC_Write_Temp,1);//Config Register B:  Gain Configuration as : Sensor Field Range->(+-)1.3Ga ; Gain->1090LSB/Gauss; Output Range->0xF800-0x07ff(-2048~2047)
	IIC_Write_Temp=0x00;   //00000000B
	I2C_AddCMD_Write_Bytes(HMC5883_ADDRESS,HMC5883_Mode,&IIC_Write_Temp,1);//Config Mode as: Continous Measurement Mode
	
	
	///////////////////////////////////////
	///MS561101BA��ѹ�Ƴ�ʼ��--2
	///////////////////////////////////////
	
	IMU_Add_MS561101BA_ReadPROM(IMU);//��ȡ����У��ֵ
	IMU_Start_CMD_Queue();
	while(!IMU_Is_Device_Ready() || !IMU_Is_Queue_Empty());//�ȴ���ʼ������ִ�����
//	printf("\r\nIMU Init ok\r\n");
	

}



/*****************************************************************************************************************************/
									///////////////////////////////////////
									///���ٶȡ����ٶ����
									//////////////////////////////////////
/*****************************************************************************************************************************/
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





/*****************************************************************************************************************************/
									///////////////////////////
									///���������
									///////////////////////////
/*****************************************************************************************************************************/

////////////////////////////////
///gain the tree aixs compass data
////////////////////////////////
void IMU_Add_Read_Compass(IMUData* IMU)
{
	I2C_AddCMD_Read_Bytes(HMC5883_ADDRESS,HMC5883_XOUT_M,&(IMU->mag_XH),6);//���������µĶ�ȡ�ֽڵ�����
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
									///��ѹ�����
									///////////////////////////
/*****************************************************************************************************************************/



////////////////////////////
///��ѹ�Ƹ�λ
///////////////////////////
void IMU_Add_MS561101BA_Reset()
{
	I2C_AddCMD_Write_CMD_Byte(MS561101BA_ADDR,MS561101BA_RESET);//��ѹ���ϵ縴λ
}



/////////////////////////
///��ȡ��ѹ�Ƴ���У׼ֵ
///�ϵ縴λ��Ӧ�û�ȡ
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
///������ѹ���¶�ת��
///@attention ע��:��ѹ�Ƶ����ݶ�ȡ�൱�ڴ��еģ�������ȡ�¶Ⱥ����9.04ms(ʱ��ȡ����OSR������Խ����Ҫʱ��Խ��)��ſɻ�ȡ��ѹ���¶�ֵ
//////////////////////////////////
void IMU_Add_MS561101BA_DoConversion_Temp(u8 OSR)
{
	I2C_AddCMD_Write_CMD_Byte(MS561101BA_ADDR,MS561101BA_D2+OSR);
}




//////////////////////////////////
///������ѹ����ѹֵת��
///@attention ע��:��ѹ�Ƶ����ݶ�ȡ�൱�ڴ��еģ�������ȡ��ѹ�����9.04ms(ʱ��ȡ����OSR������Խ����Ҫʱ��Խ��)��ſɻ�ȡ��ѹ����ѹֵ
//////////////////////////////////
void IMU_Add_MS561101BA_DoConversion_Pressure(u8 OSR)
{
	I2C_AddCMD_Write_CMD_Byte(MS561101BA_ADDR,MS561101BA_D1+OSR);
}





////////////////////////////////////
///��ȡ��ѹ�Ƶ��¶�ֵ
///@pre �Ѿ�����ת������ת����ϣ����뿪ʼת����ʱ�����ת����Ҫ��ʱ�䣩 
////////////////////////////////////
void IMU_Add_MS561101BA_Receive_Temp(IMUData* IMU)
{
	I2C_AddCMD_Read_Bytes(MS561101BA_ADDR,0,&IMU->MS561101BA_Value.D2_2,3);
}





////////////////////////////////////
///��ȡ��ѹ�Ƶ���ѹֵ
///@pre �Ѿ�����ת������ת����ϣ����뿪ʼת����ʱ�����ת����Ҫ��ʱ�䣩 
////////////////////////////////////
void IMU_Add_MS561101BA_Receive_Pressure(IMUData* IMU)
{
	I2C_AddCMD_Read_Bytes(MS561101BA_ADDR,0,&IMU->MS561101BA_Value.D1_2,3);
}





//////////////////////////////////
///��ȡ��ѹ�Ƶ��¶�ֵ
///@pre �¶�ת�����¶�ԭʼֵ��ȡ���
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
///��ȡ��ѹ�Ƶ���ѹֵ
///@pre �¶�ת�����¶�ԭʼֵ��ȡ��� ���� ��ѹת������ѹԭʼֵ��ȡ���
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
									///������в����鿴��IMU����״̬��������
									///////////////////////////////////////////////////
/*****************************************************************************************************************************/
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


