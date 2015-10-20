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




#ifndef __HAL_IMU_H
#define	__HAL_IMU_H


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"
#include "iic.h"
//#include "delay.h"
#include "math.h"
/**************************************************************************************************************************************************/
/////////////////////////////////////////////////////////////////////
///IMU Configuration 
////////////////////////////////////////////////////////////////////


//	# define MS561101BA_Pin_CSB_HIGH  //如果气压计CSB引脚为低电平，请注释掉该段宏定义  高电平则取消注释


/**************************************************************************************************************************************************/


/* MPU6050 Register Address ------------------------------------------------------------*/
#define	SMPLRT_DIV					0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG							0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		 			0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	 			0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	 			0x3B
#define	ACCEL_XOUT_L 				0x3C
#define	ACCEL_YOUT_H				0x3D
#define	ACCEL_YOUT_L				0x3E
#define	ACCEL_ZOUT_H				0x3F
#define	ACCEL_ZOUT_L	 			0x40
#define	TEMP_OUT_H					0x41
#define	TEMP_OUT_L					0x42
#define	GYRO_XOUT_H		 			0x43
#define	GYRO_XOUT_L		 			0x44	
#define	GYRO_YOUT_H		 			0x45
#define	GYRO_YOUT_L		 			0x46
#define	GYRO_ZOUT_H		 			0x47
#define	GYRO_ZOUT_L					0x48
#define	PWR_MGMT_1		 			0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I			 			0x75	//IIC地址寄存器(默认数值0x68，只读)

#define MPU6050_ADDRESS   MPU6050_DEFAULT_ADDRESS
#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     0xD0   //MPU6050_ADDRESS_AD0_LOW  ((MPU6050_ADDRESS_AD0_LOW<<1)&0xFE) or  ((MPU6050_ADDRESS_AD0_HIGH<<1)&0xFE)

#define I2C_MST_CTRL        0x24  
#define I2C_SLV0_ADDR       0x25  //指定从机的IIC地址
#define I2C_SLV0_REG        0x26 	//指定从机的寄存器地址 
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV0_DO         0x63   //该寄存器的内容会写入到从机设备中
#define USER_CTRL           0x6A    //用户使能FIFO缓存区    I2C主机模式和主要I2C接口

#define INT_PIN_CFG         0x37  
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54




//---------------HMC5883L Register Address ---------------------------
#define HMC5883_ADDRESS     			0x3C  //       HMC     7-bit 地址:  0x1E    ADRESS+WRITE->0X3C  ADRESS+READ->0X3D
#define HMC5883_Config_RA      			0x00  //Configuration Register A   R/W
#define HMC5883_Config_RB      			0x01  //配置寄存器B                 R/W
#define HMC5883_Mode  			0x02  //模式寄存器                          R/W
#define HMC5883_XOUT_M 			0x03  //Data Output X MSB Register   Only Read
#define HMC5883_XOUT_L 			0x04  //Data Output X LSB Register   Only Read
#define HMC5883_YOUT_M 			0x05  //Data Output Y MSB Register   Only Read
#define HMC5883_YOUT_L 			0x06  //Data Output Y LSB Register   Only Read
#define HMC5883_ZOUT_M 			0x07  //Data Output Z MSB Register   Only Read
#define HMC5883_ZOUT_L 			0x08  //Data Output Z LSB Register   Only Read
#define HMC5883_Status             0x09   //状态寄存器                Only Read
#define HMC5883_Identification_A   0x0A   //识别寄存器                Only Read
#define HMC5883_Identification_B   0x0B   //                         Only Read
#define HMC5883_Identification_C   0x0C   //                         Only Read

#define MagnetcDeclination  1.0 //重庆
#define CalThreshold        0
#define PI                  3.141592653
#define M_PI                3.14159265358979323846


//----------------MS561101BA-----------------------------------------------------------

//地址，111011Cx，C为CSB引脚的反码  // addresses of the device  The MS5611-01BA address is 111011Cx, where C is the complementary value of the pin CSB. 
#ifdef MS561101BA_Pin_CSB_HIGH
	#define MS561101BA_ADDR 0xEC
#else
	#define MS561101BA_ADDR 0xEE
#endif


// registers of the device
#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
#define MS561101BA_RESET 0x1E
#define MS561101BA_D1D2_SIZE 3	// D1 and D2 result size (bytes)

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define MS561101BA_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values. 
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry.





/////////////////////////////////////
///定义变量保存气压计的值
////////////////////////////////////
typedef struct
{
	u8 C1_H;
	u8 C1_L;
	u8 C2_H;
	u8 C2_L;
	u8 C3_H;
	u8 C3_L;
	u8 C4_H;
	u8 C4_L;
	u8 C5_H;
	u8 C5_L;
	u8 C6_H;
	u8 C6_L;
	u8 D1_2;//气压值原始数据最高位
	u8 D1_1;//气压值原始数据
	u8 D1_0;//气压值原始数据最低位
	u8 D2_2;//温度值原始数据最高位
	u8 D2_1;//温度值原始数据
	u8 D2_0;//温度值原始数据最低位
	int32_t Pressure;//经过计算后的气压值
}MS561101BA_Value_Typedef;


//typedef union
//{
//	s16 Int;
//	u8  Byte[2];
//}Int_Bytes_TypeDef;//keil使用的是小端，即Int的低字节保存在Byte[0],Int类型的高字节保存在Byte[1]   ,但是MPU6050使用的是大端，即数据的高字节放在低地址，低字节放在高地址
//					//所以下面的IMU数据结构体不使用union了

typedef struct
{	
	//加速度
	u8 acc_XH;			//加速度 x轴 高字节
	u8 acc_XL;			//加速度 x轴 低字节
	u8 acc_YH;			//加速度 y轴 高字节
	u8 acc_YL;			//加速度 y轴 低字节
	u8 acc_ZH;			//加速度 z轴 高字节
	u8 acc_ZL;			//加速度 z轴 低字节	
		//温度
	u8 imu_TH;			//MPU6050温度 高字节
	u8 imu_TL;			//MPU6050温度 低字节
	//陀螺仪
	u8 gyro_XH;			//陀螺仪 x轴 高字节
	u8 gyro_XL;			//陀螺仪 x轴 低字节
	u8 gyro_YH;			//陀螺仪 y轴 高字节
	u8 gyro_YL;			//陀螺仪 y轴 低字节
	u8 gyro_ZH;			//陀螺仪 z轴 高字节
	u8 gyro_ZL;			//陀螺仪 z轴 低字节		
	//地磁
	u8 mag_XH;			//地磁计 x轴 高字节
	u8 mag_XL;			//地磁计 x轴 低字节
	u8 mag_ZH;			//地磁计 y轴 高字节
	u8 mag_ZL;			//地磁计 y轴 低字节
	u8 mag_YH;			//地磁计 z轴 高字节
	u8 mag_YL;			//地磁计 z轴 低字节	
	//气压
	MS561101BA_Value_Typedef MS561101BA_Value;
	u8 Acc_Exist;//加速度计存在表示      0：不存在  1：存在 2：正在检测是否存在
	u8 Gyro_Exist;//角速度计存在表示 0：不存在  1：存在 2：正在检测是否存在
	u8 Mag_Exist;//磁力计存在表示    0：不存在  1：存在 2：正在检测是否存在
	u8 Baro_Exist;//气压计存在表示   0：不存在  1：存在 2：正在检测是否存在
}IMUData;




/*****************************************************************************************************************************/
										///////////////////////
										///IMU初始化，包括所有传感器的初始化
										//////////////////////
/*****************************************************************************************************************************/

///////////////////////////
///IMU初始化，初始化之前需要等待一段时间，等IMU上电启动。
///@retval -1:初始化成功 -2:部分成功 -3:正在初始化  -0:初始化失败
///@param 是否使用响应传感器  加速度角速度  磁力计  气压计  0：不使用 1：使用
///////////////////////////
u8 IMU_Init(IMUData* IMU,u8 useAccGyro,u8 useMag,u8 useBaro);





/*****************************************************************************************************************************/
									///////////////////////////////////////
									///加速度、角速度相关
									//////////////////////////////////////
/*****************************************************************************************************************************/

/////////////////////////
///获取IMU的加速度、温度和角速度值到IMUData类型的参数中
///只是将读取的命令放入了队列中
////////////////////////
void IMU_Add_Read_ACC_TEMP_GYR(IMUData* );


//////////////////////////
///读取IMU某个寄存器的值
/////////////////////////
void IMU_Add_Read_Register(u8 device_addr,u8 register_addr, u8* data_read, u8 num);//参数：设备地址，寄存器地址，读取的数据存放的地址，需要读取数据的个数)



/////////////////////////
///写IMU的某个寄存器
/////////////////////////
void IMU_Add_Write_Register(u8 device_addr,u8 register_addr, u8* data_write, u8 num);//参数：设备地址，寄存器地址，读取的数据存放的地址，需要读取数据的个数


//////////////////////////////
///从硬件更新加速度、角速度值到内存
///@return -0:发送更新数据失败 1:命令将会发送（具体的发送时间取决于队列中的排队的命令的数量）
///@param IMU data adress
//////////////////////////////
u8 HAL_Update_Data_ACC_GYR(IMUData*  IMU);


/*****************************************************************************************************************************/
									///////////////////////////
									///磁力计相关
									///////////////////////////
/*****************************************************************************************************************************/

////////////////////////////////
///gain the tree aixs compass data
////////////////////////////////
void IMU_Add_Read_Compass(IMUData* IMU);

///////////////////////////////////
///Calculate the Heading of Compass 
///@return Heading Defrees
//////////////////////////////////
double calculateHeading(IMUData* IMU);



/*****************************************************************************************************************************/
									///////////////////////////
									///气压计相关
									///////////////////////////
/*****************************************************************************************************************************/

//****************************//
///*********************注意：读取气压计时，IIC速度最好不要设置400k，有时会跟不上（经测试，几率很大），设置小于400k的比较好********************
//****************************//





////////////////////////////
///气压计复位
///////////////////////////
void IMU_Add_MS561101BA_Reset(void);




/////////////////////////
///获取气压计出厂校准值
///上电复位后应该获取
////////////////////////
void IMU_Add_MS561101BA_ReadPROM(IMUData* );



//////////////////////////////////
///启动气压计温度转换
///@attention 注意:气压计的数据读取相当于串行的，开启读取温度后最大9.04ms(时间取决于OSR，精度越高需要时间越多)后才可获取气压计温度值
//////////////////////////////////
void IMU_Add_MS561101BA_DoConversion_Temp(u8 OSR);




//////////////////////////////////
///启动气压计气压值转换
///@attention 注意:气压计的数据读取相当于串行的，开启读取气压后最大9.04ms(时间取决于OSR，精度越高需要时间越多)后才可获取气压计气压值
//////////////////////////////////
void IMU_Add_MS561101BA_DoConversion_Pressure(u8 OSR);




////////////////////////////////////
///获取气压计的温度值
///@pre 已经启动转换并且转换完毕（距离开始转换的时间大于转换需要的时间） 
////////////////////////////////////
void IMU_Add_MS561101BA_Receive_Temp(IMUData* );





////////////////////////////////////
///获取气压计的气压值
///@pre 已经启动转换并且转换完毕（距离开始转换的时间大于转换需要的时间） 
////////////////////////////////////
void IMU_Add_MS561101BA_Receive_Pressure(IMUData* );





//////////////////////////////////
///获取气压计的温度值
///@pre 温度转换、温度原始值获取完毕
//////////////////////////////////
u32 IMU_Add_MS561101BA_Get_Temperature(IMUData* );




//////////////////////////////////
///获取气压计的气压值
///@pre 温度转换、温度原始值获取完毕 并且 气压转换、气压原始值获取完毕
//////////////////////////////////
int32_t IMU_Add_MS561101BA_Get_Pressure(IMUData* );









/*****************************************************************************************************************************/
									/////////////////////////////////////////////////
									///命令队列操作查看、IMU总线状态检查等其它
									///////////////////////////////////////////////////
/*****************************************************************************************************************************/

//////////////////////////////////////
///检查标识与IMU设备通信是否处于空闲状态，即是否可以开始下一次读/写命令
//////////////////////////////////////
I2C_Bool IMU_Is_Device_Ready(void);



//////////////////////////////////
///获取填充命令的队列的长度
/////////////////////////////////
u16 IMU_Get_Queue_length(void);



//////////////////////////////////
///判断填充命令的队列的长度是否为空
/////////////////////////////////
u8 IMU_Is_Queue_Empty(void);



///开始执行队列里的命令
///@retval -1:成功发送开始信号，可能会执行成功 -2:队列为空，无需开始 -0:发送成功信号失败,I2C总线通信错误
////////////////////////////////
u8 IMU_Start_CMD_Queue(void);
	


#ifdef __cplusplus
}
#endif

#endif 

