Version:0.1

Authors: 2015-10-01 ChenZhangdi

Introduction:
		�жϷ�ʽ����MPU6050�����������һ��ѭ�����й���
		IIC�ж����ȼ��������ߣ������� iic.h �궨��������
		�˴����õ��ж����ȼ�Ϊ������2 ��IIC�¼��ж���ռ���ȼ�Ϊ2����Ӧ���ȼ�Ϊ0�� IIC�����ж���ռʽ���ȼ�Ϊ0����Ӧ���ȼ�Ϊ0
		
		
Change Log:
		2015-10-01 ����ʵ���жϷ�ʽ��ȡMPU6050
		
How To Use It��
		1�������������ļ����µ�6���ļ� IIC���ã���iic.h��ͷ��/*I2C  HAL Configuration*/��ע�͵����õ�ѡ���Ҫʹ�õ�ѡ��ȡ��ע�ͣ�
		2��IICӲ����ʼ��      I2C_Init_Config();
		3: IMU��ʼ��          IMU_Init();
		4: ��������������   void IMU_Add_Read_ACC_TEMP_GYR(IMUData*);  //���ٶȵ����ݱ�����IMUData�͵Ľṹ����
							  void IMU_Add_Read_Register(u8 device_addr,u8 register_addr, u8* data_read, u8 num);//�������豸��ַ���Ĵ�����ַ����ȡ�����ݴ�ŵĵ�ַ����Ҫ��ȡ���ݵĸ���)
							  void IMU_Add_Write_Register(u8 device_addr,u8 register_addr, u8* data_write, u8 num);//�������豸��ַ���Ĵ�����ַ����ȡ�����ݴ�ŵĵ�ַ����Ҫ��ȡ���ݵĸ���
							  
		5:��IIC״̬Ϊ���е�ʱ���Ҷ����е�������������0��ʱ��Ϳ��Կ�ʼִ�ж����е������� (ע�⣺һ��Ҫ��IICΪ���в���������в�Ϊ�յ�ʱ����ܵ��ÿ�ʼִ��������䣬������ܲ���ʱ�����)
							  if ( IMU_Is_Device_Ready()  &&  (!IMU_Is_Queue_Empty()) )
									IMU_Start_CMD_Queue();