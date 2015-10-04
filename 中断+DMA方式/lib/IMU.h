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




#ifndef __IMU_H
#define	__IMU_H


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"
#include "iic.h"
#include "delay.h"
#include "usart.h"
#include "math.h"
/**************************************************************************************************************************************************/
/////////////////////////////////////////////////////////////////////
///IMU Configuration 
////////////////////////////////////////////////////////////////////


//	# define MS561101BA_Pin_CSB_HIGH  //�����ѹ��CSB����Ϊ�͵�ƽ����ע�͵��öκ궨��  �ߵ�ƽ��ȡ��ע��
	


/**************************************************************************************************************************************************/


/* MPU6050 Register Address ------------------------------------------------------------*/
#define	SMPLRT_DIV					0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG							0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		 			0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	 			0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
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
#define	PWR_MGMT_1		 			0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I			 			0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)

#define MPU6050_ADDRESS   MPU6050_DEFAULT_ADDRESS
#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     0xD0   //MPU6050_ADDRESS_AD0_LOW  ((MPU6050_ADDRESS_AD0_LOW<<1)&0xFE) or  ((MPU6050_ADDRESS_AD0_HIGH<<1)&0xFE)

#define I2C_MST_CTRL        0x24  
#define I2C_SLV0_ADDR       0x25  //ָ���ӻ���IIC��ַ
#define I2C_SLV0_REG        0x26 	//ָ���ӻ��ļĴ�����ַ 
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV0_DO         0x63   //�üĴ��������ݻ�д�뵽�ӻ��豸��
#define USER_CTRL           0x6A    //�û�ʹ��FIFO������    I2C����ģʽ����ҪI2C�ӿ�

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
#define HMC5883_ADDRESS     			0x3C  //       HMC     7-bit ��ַ:  0x1E    ADRESS+WRITE->0X3C  ADRESS+READ->0X3D
#define HMC5883_Config_RA      			0x00  //Configuration Register A   R/W
#define HMC5883_Config_RB      			0x01  //���üĴ���B                 R/W
#define HMC5883_Mode  			0x02  //ģʽ�Ĵ���                          R/W
#define HMC5883_XOUT_M 			0x03  //Data Output X MSB Register   Only Read
#define HMC5883_XOUT_L 			0x04  //Data Output X LSB Register   Only Read
#define HMC5883_YOUT_M 			0x05  //Data Output Y MSB Register   Only Read
#define HMC5883_YOUT_L 			0x06  //Data Output Y LSB Register   Only Read
#define HMC5883_ZOUT_M 			0x07  //Data Output Z MSB Register   Only Read
#define HMC5883_ZOUT_L 			0x08  //Data Output Z LSB Register   Only Read
#define HMC5883_Status             0x09   //״̬�Ĵ���                Only Read
#define HMC5883_Identification_A   0x0A   //ʶ��Ĵ���                Only Read
#define HMC5883_Identification_B   0x0B   //                         Only Read
#define HMC5883_Identification_C   0x0C   //                         Only Read

#define MagnetcDeclination  1.0 //����
#define CalThreshold        0
#define PI                  3.141592653
#define M_PI                3.14159265358979323846


//----------------MS561101BA-----------------------------------------------------------

//��ַ��111011Cx��CΪCSB���ŵķ���  // addresses of the device  The MS5611-01BA address is 111011Cx, where C is the complementary value of the pin CSB. 
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
///�������������ѹ�Ƶ�ֵ
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
	u8 D1_2;//��ѹֵԭʼ�������λ
	u8 D1_1;//��ѹֵԭʼ����
	u8 D1_0;//��ѹֵԭʼ�������λ
	u8 D2_2;//�¶�ֵԭʼ�������λ
	u8 D2_1;//�¶�ֵԭʼ����
	u8 D2_0;//�¶�ֵԭʼ�������λ
	int32_t Pressure;//������������ѹֵ
}MS561101BA_Value_Typedef;


//typedef union
//{
//	s16 Int;
//	u8  Byte[2];
//}Int_Bytes_TypeDef;//keilʹ�õ���С�ˣ���Int�ĵ��ֽڱ�����Byte[0],Int���͵ĸ��ֽڱ�����Byte[1]   ,����MPU6050ʹ�õ��Ǵ�ˣ������ݵĸ��ֽڷ��ڵ͵�ַ�����ֽڷ��ڸߵ�ַ
//					//���������IMU���ݽṹ�岻ʹ��union��

typedef struct
{	
	//���ٶ�
	u8 acc_XH;			//���ٶ� x�� ���ֽ�
	u8 acc_XL;			//���ٶ� x�� ���ֽ�
	u8 acc_YH;			//���ٶ� y�� ���ֽ�
	u8 acc_YL;			//���ٶ� y�� ���ֽ�
	u8 acc_ZH;			//���ٶ� z�� ���ֽ�
	u8 acc_ZL;			//���ٶ� z�� ���ֽ�	
		//�¶�
	u8 imu_TH;			//MPU6050�¶� ���ֽ�
	u8 imu_TL;			//MPU6050�¶� ���ֽ�
	//������
	u8 gyro_XH;			//������ x�� ���ֽ�
	u8 gyro_XL;			//������ x�� ���ֽ�
	u8 gyro_YH;			//������ y�� ���ֽ�
	u8 gyro_YL;			//������ y�� ���ֽ�
	u8 gyro_ZH;			//������ z�� ���ֽ�
	u8 gyro_ZL;			//������ z�� ���ֽ�		
	//�ش�
	u8 mag_XH;			//�شż� x�� ���ֽ�
	u8 mag_XL;			//�شż� x�� ���ֽ�
	u8 mag_ZH;			//�شż� y�� ���ֽ�
	u8 mag_ZL;			//�شż� y�� ���ֽ�
	u8 mag_YH;			//�شż� z�� ���ֽ�
	u8 mag_YL;			//�شż� z�� ���ֽ�	
	//��ѹ
	MS561101BA_Value_Typedef MS561101BA_Value;
//	u16 powerVoltage;
//	u8 sum;
}IMUData;



/*****************************************************************************************************************************/
										///////////////////////
										///IMU��ʼ�����������д������ĳ�ʼ��
										//////////////////////
/*****************************************************************************************************************************/

///////////////////////////
///IMU��ʼ������ʼ��֮ǰ��Ҫ�ȴ�һ��ʱ�䣬��IMU�ϵ�������
///////////////////////////
void IMU_Init(IMUData* IMU);





/*****************************************************************************************************************************/
									///////////////////////////////////////
									///���ٶȡ����ٶ����
									//////////////////////////////////////
/*****************************************************************************************************************************/

/////////////////////////
///��ȡIMU�ļ��ٶȡ��¶Ⱥͽ��ٶ�ֵ��IMUData���͵Ĳ�����
///ֻ�ǽ���ȡ����������˶�����
////////////////////////
void IMU_Add_Read_ACC_TEMP_GYR(IMUData* );


//////////////////////////
///��ȡIMUĳ���Ĵ�����ֵ
/////////////////////////
void IMU_Add_Read_Register(u8 device_addr,u8 register_addr, u8* data_read, u8 num);//�������豸��ַ���Ĵ�����ַ����ȡ�����ݴ�ŵĵ�ַ����Ҫ��ȡ���ݵĸ���)



/////////////////////////
///дIMU��ĳ���Ĵ���
/////////////////////////
void IMU_Add_Write_Register(u8 device_addr,u8 register_addr, u8* data_write, u8 num);//�������豸��ַ���Ĵ�����ַ����ȡ�����ݴ�ŵĵ�ַ����Ҫ��ȡ���ݵĸ���






/*****************************************************************************************************************************/
									///////////////////////////
									///���������
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
									///��ѹ�����
									///////////////////////////
/*****************************************************************************************************************************/

//****************************//
///*********************ע�⣺��ȡ��ѹ��ʱ��IIC�ٶ���ò�Ҫ����400k����ʱ������ϣ������ԣ����ʺܴ󣩣�����С��400k�ıȽϺ�********************
//****************************//





////////////////////////////
///��ѹ�Ƹ�λ
///////////////////////////
void IMU_Add_MS561101BA_Reset(void);




/////////////////////////
///��ȡ��ѹ�Ƴ���У׼ֵ
///�ϵ縴λ��Ӧ�û�ȡ
////////////////////////
void IMU_Add_MS561101BA_ReadPROM(IMUData* );



//////////////////////////////////
///������ѹ���¶�ת��
///@attention ע��:��ѹ�Ƶ����ݶ�ȡ�൱�ڴ��еģ�������ȡ�¶Ⱥ����9.04ms(ʱ��ȡ����OSR������Խ����Ҫʱ��Խ��)��ſɻ�ȡ��ѹ���¶�ֵ
//////////////////////////////////
void IMU_Add_MS561101BA_DoConversion_Temp(u8 OSR);




//////////////////////////////////
///������ѹ����ѹֵת��
///@attention ע��:��ѹ�Ƶ����ݶ�ȡ�൱�ڴ��еģ�������ȡ��ѹ�����9.04ms(ʱ��ȡ����OSR������Խ����Ҫʱ��Խ��)��ſɻ�ȡ��ѹ����ѹֵ
//////////////////////////////////
void IMU_Add_MS561101BA_DoConversion_Pressure(u8 OSR);




////////////////////////////////////
///��ȡ��ѹ�Ƶ��¶�ֵ
///@pre �Ѿ�����ת������ת����ϣ����뿪ʼת����ʱ�����ת����Ҫ��ʱ�䣩 
////////////////////////////////////
void IMU_Add_MS561101BA_Receive_Temp(IMUData* );





////////////////////////////////////
///��ȡ��ѹ�Ƶ���ѹֵ
///@pre �Ѿ�����ת������ת����ϣ����뿪ʼת����ʱ�����ת����Ҫ��ʱ�䣩 
////////////////////////////////////
void IMU_Add_MS561101BA_Receive_Pressure(IMUData* );





//////////////////////////////////
///��ȡ��ѹ�Ƶ��¶�ֵ
///@pre �¶�ת�����¶�ԭʼֵ��ȡ���
//////////////////////////////////
u32 IMU_Add_MS561101BA_Get_Temperature(IMUData* );




//////////////////////////////////
///��ȡ��ѹ�Ƶ���ѹֵ
///@pre �¶�ת�����¶�ԭʼֵ��ȡ��� ���� ��ѹת������ѹԭʼֵ��ȡ���
//////////////////////////////////
int32_t IMU_Add_MS561101BA_Get_Pressure(IMUData* );









/*****************************************************************************************************************************/
									/////////////////////////////////////////////////
									///������в����鿴��IMU����״̬��������
									///////////////////////////////////////////////////
/*****************************************************************************************************************************/

//////////////////////////////////////
///����ʶ��IMU�豸ͨ���Ƿ��ڿ���״̬�����Ƿ���Կ�ʼ��һ�ζ�/д����
//////////////////////////////////////
I2C_Bool IMU_Is_Device_Ready(void);



//////////////////////////////////
///��ȡ�������Ķ��еĳ���
/////////////////////////////////
u16 IMU_Get_Queue_length(void);



//////////////////////////////////
///�ж��������Ķ��еĳ����Ƿ�Ϊ��
/////////////////////////////////
u8 IMU_Is_Queue_Empty(void);



/////////////////////////////////
///��ʼִ�ж����������
///@attention һ��Ҫ��IMU_Is_Device_Ready()==I2C_True����IMU_Is_Queue_Empty()==I2C_Falseʱ����ִ��
////////////////////////////////
void IMU_Start_CMD_Queue(void);
	



#ifdef __cplusplus
}
#endif

#endif 

