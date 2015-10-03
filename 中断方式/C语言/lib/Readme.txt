Version:0.1

Authors: 2015-10-01 ChenZhangdi

Introduction:
		中断方式操作MPU6050，操作命令都由一个循环队列管理，
		IIC中断优先级最好是最高，可以在 iic.h 宏定义中设置
		此处设置的中断优先级为：分组2 ，IIC事件中断抢占优先级为2，响应优先级为0， IIC错误中断抢占式优先级为0，响应优先级为0
		
		
Change Log:
		2015-10-01 基本实现中断方式读取MPU6050
		2015-10-03 MPU6050 HMC5583L  MS561101BA related basic operation is ok
		
How To Use It：
		1：工程中引入文件夹下的6个文件 IIC配置（在iic.h开头的/*I2C  HAL Configuration*/中注释掉不用的选项，给要使用的选项取消注释）
		2：IIC硬件初始化      I2C_Init_Config();
		3: IMU初始化          IMU_Init();
		4: 向队列中添加命令   void IMU_Add_Read_ACC_TEMP_GYR(IMUData*);  //加速度的数据保存在IMUData型的结构体中
							  void IMU_Add_Read_Register(u8 device_addr,u8 register_addr, u8* data_read, u8 num);//参数：设备地址，寄存器地址，读取的数据存放的地址，需要读取数据的个数)
							  void IMU_Add_Write_Register(u8 device_addr,u8 register_addr, u8* data_write, u8 num);//参数：设备地址，寄存器地址，读取的数据存放的地址，需要读取数据的个数
							  ... ....
		5:在IIC状态为空闲的时候并且队列中的命令数量大于0的时候就可以开始执行队列中的命令了 (注意：一定要在IIC为空闲并且命令队列不为空的时候才能调用开始执行命令语句，否则可能产生时序错误)
							  if ( IMU_Is_Device_Ready()  &&  (!IMU_Is_Queue_Empty()) )
									IMU_Start_CMD_Queue();
									
		6:pay attention to the operation of MS561101BA , temperature and pressure Conversion can last a few time,data can be read only after conversion complete;