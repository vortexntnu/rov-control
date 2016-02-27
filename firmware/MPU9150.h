// IMU.h

#ifndef _IMU_h
#define _IMU_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
//        #include "ros.h"
#else
	#include "WProgram.h"
#endif

#include <string.h>
#include <geometry_msgs/Vector3.h>



class IMU
{
 private:
	int compas[3];
	int acceleration[3];
	int gyro[3];
	double temp;
	int I2C_address;

 public:
	void CompasSetup();
	void setI2C_address(int address);
	void init();
	int read(int address);
	int read(int addressL , int addressH);
	int write(int address , int data);
	String readAll();
	IMU();
	~IMU();

	double readTemperature();
	void readAcceleration(geometry_msgs::Vector3 *retur);
	void readGyro(geometry_msgs::Vector3 *retur);
	void readCompas(geometry_msgs::Vector3 *retur);
};

//extern IMU IMU;


// definitions
enum MPU9159Commands {
	MPU9150_SELF_TEST_X       = 0x0D,   // R/W
	MPU9150_SELF_TEST_Y       = 0x0E,   // R/W
	MPU9150_SELF_TEST_z       = 0x0F,   // R/W
	MPU9150_SELF_TEST_A       = 0x10,   // R/W
	MPU9150_SMPLRT_DIV        = 0x19,   // R/W
	MPU9150_CONFIG            = 0x1A,   // R/W
	MPU9150_GYRO_CONFIG       = 0x1B,   // R/W
	MPU9150_ACCEL_CONFIG      = 0x1C,   // R/W
	MPU9150_FF_THR            = 0x1D,   // R/W
	MPU9150_FF_DUR            = 0x1E,   // R/W
	MPU9150_MOT_THR           = 0x1F,   // R/W
	MPU9150_MOT_DUR           = 0x20,   // R/W
	MPU9150_ZRMOT_THR         = 0x21,   // R/W
	MPU9150_ZRMOT_DUR         = 0x22,   // R/W
	MPU9150_FIFO_EN           = 0x23,   // R/W
	MPU9150_I2C_MST_CTRL      = 0x24,   // R/W
	MPU9150_I2C_SLV0_ADDR     = 0x25,   // R/W
	MPU9150_I2C_SLV0_REG      = 0x26,   // R/W
	MPU9150_I2C_SLV0_CTRL     = 0x27,   // R/W
	MPU9150_I2C_SLV1_ADDR     = 0x28,   // R/W
	MPU9150_I2C_SLV1_REG      = 0x29,   // R/W
	MPU9150_I2C_SLV1_CTRL     = 0x2A,   // R/W
	MPU9150_I2C_SLV2_ADDR     = 0x2B,   // R/W
	MPU9150_I2C_SLV2_REG      = 0x2C,   // R/W
	MPU9150_I2C_SLV2_CTRL     = 0x2D,   // R/W
	MPU9150_I2C_SLV3_ADDR     = 0x2E,   // R/W
	MPU9150_I2C_SLV3_REG      = 0x2F,   // R/W
	MPU9150_I2C_SLV3_CTRL     = 0x30,   // R/W
	MPU9150_I2C_SLV4_ADDR     = 0x31,   // R/W
	MPU9150_I2C_SLV4_REG      = 0x32,   // R/W
	MPU9150_I2C_SLV4_DO       = 0x33,   // R/W
	MPU9150_I2C_SLV4_CTRL     = 0x34,   // R/W
	MPU9150_I2C_SLV4_DI       = 0x35,   // R
	MPU9150_I2C_MST_STATUS    = 0x36,   // R
	MPU9150_INT_PIN_CFG       = 0x37,   // R/W
	MPU9150_INT_ENABLE        = 0x38,   // R/W
	MPU9150_INT_STATUS        = 0x3A,   // R
	MPU9150_ACCEL_XOUT_H      = 0x3B,   // R
	MPU9150_ACCEL_XOUT_L      = 0x3C,   // R
	MPU9150_ACCEL_YOUT_H      = 0x3D,   // R
	MPU9150_ACCEL_YOUT_L      = 0x3E,   // R
	MPU9150_ACCEL_ZOUT_H      = 0x3F,   // R
	MPU9150_ACCEL_ZOUT_L      = 0x40,   // R
	MPU9150_TEMP_OUT_H        = 0x41,   // R
	MPU9150_TEMP_OUT_L        = 0x42,   // R
	MPU9150_GYRO_XOUT_H       = 0x43,   // R
	MPU9150_GYRO_XOUT_L       = 0x44,   // R
	MPU9150_GYRO_YOUT_H       = 0x45,   // R
	MPU9150_GYRO_YOUT_L       = 0x46,   // R
	MPU9150_GYRO_ZOUT_H       = 0x47,   // R
	MPU9150_GYRO_ZOUT_L       = 0x48,   // R
	MPU9150_EXT_SENS_DATA_00  = 0x49,   // R
	MPU9150_EXT_SENS_DATA_01  = 0x4A,   // R
	MPU9150_EXT_SENS_DATA_02  = 0x4B,   // R
	MPU9150_EXT_SENS_DATA_03  = 0x4C,   // R
	MPU9150_EXT_SENS_DATA_04  = 0x4D,   // R
	MPU9150_EXT_SENS_DATA_05  = 0x4E,   // R
	MPU9150_EXT_SENS_DATA_06  = 0x4F,   // R
	MPU9150_EXT_SENS_DATA_07  = 0x50,   // R
	MPU9150_EXT_SENS_DATA_08  = 0x51,   // R
	MPU9150_EXT_SENS_DATA_09  = 0x52,   // R
	MPU9150_EXT_SENS_DATA_10  = 0x53,   // R
	MPU9150_EXT_SENS_DATA_11  = 0x54,   // R
	MPU9150_EXT_SENS_DATA_12  = 0x55,   // R
	MPU9150_EXT_SENS_DATA_13  = 0x56,   // R
	MPU9150_EXT_SENS_DATA_14  = 0x57,   // R
	MPU9150_EXT_SENS_DATA_15  = 0x58,   // R
	MPU9150_EXT_SENS_DATA_16  = 0x59,   // R
	MPU9150_EXT_SENS_DATA_17  = 0x5A,   // R
	MPU9150_EXT_SENS_DATA_18  = 0x5B,   // R
	MPU9150_EXT_SENS_DATA_19  = 0x5C,   // R
	MPU9150_EXT_SENS_DATA_20  = 0x5D,   // R
	MPU9150_EXT_SENS_DATA_21  = 0x5E,   // R
	MPU9150_EXT_SENS_DATA_22  = 0x5F,   // R
	MPU9150_EXT_SENS_DATA_23  = 0x60,   // R
	MPU9150_MOT_DETECT_STATUS = 0x61,   // R
	MPU9150_I2C_SLV0_DO       = 0x63,   // R/W
	MPU9150_I2C_SLV1_DO       = 0x64,   // R/W
	MPU9150_I2C_SLV2_DO       = 0x65,   // R/W
	MPU9150_I2C_SLV3_DO       = 0x66,   // R/W
	MPU9150_I2C_MST_DELAY_CTRL= 0x67,   // R/W
	MPU9150_SIGNAL_PATH_RESET = 0x68,   // R/W
	MPU9150_MOT_DETECT_CTRL   = 0x69,   // R/W
	MPU9150_USER_CTRL         = 0x6A,   // R/W
	MPU9150_PWR_MGMT_1        = 0x6B,   // R/W
	MPU9150_PWR_MGMT_2        = 0x6C,   // R/W
	MPU9150_FIFO_COUNTH       = 0x72,   // R/W
	MPU9150_FIFO_COUNTL       = 0x73,   // R/W
	MPU9150_FIFO_R_W          = 0x74,   // R/W
	MPU9150_WHO_AM_I          = 0x75,   // R
	
	//MPU9150 Compass	
	MPU9150_CMPS_XOUT_L        = 0x4A,   // R
	MPU9150_CMPS_XOUT_H        = 0x4B,   // R
	MPU9150_CMPS_YOUT_L        = 0x4C,   // R
	MPU9150_CMPS_YOUT_H        = 0x4D,   // R
	MPU9150_CMPS_ZOUT_L        = 0x4E,   // R
	MPU9150_CMPS_ZOUT_H        = 0x4F,   // R
};



// I2C address 0x69 could be 0x68 depends on your wiring.
enum MPU9150I2CAddress{
	MPU9150_I2C_ADDRESS		= 0x69,
	COMPAS_I2C_ADDRESS		= 0x0C
};

#endif
