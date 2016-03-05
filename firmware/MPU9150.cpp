// 
// 
// 

#include "MPU9150.h"
#include <Wire.h>

void IMU::CompasSetup(){

  IMU::setI2C_address(COMPAS_I2C_ADDRESS);      //change Address to Compass
  Serial.print(this->I2C_address);
  IMU::write(0x0A, 0x00); //PowerDownMode
  IMU::write(0x0A, 0x0F); //SelfTest
  IMU::write(0x0A, 0x00); //PowerDownMode
  
  IMU::setI2C_address(MPU9150_I2C_ADDRESS);      //change Address to MPU
 
  IMU::write(0x24, 0x40); //Wait for Data at Slave0
  IMU::write(0x25, 0x8C); //Set i2c address at slave0 at 0x0C
  IMU::write(0x26, 0x02); //Set where reading at slave 0 starts
  IMU::write(0x27, 0x88); //set offset at start reading and enable
  IMU::write(0x28, 0x0C); //set i2c address at slv1 at 0x0C
  IMU::write(0x29, 0x0A); //Set where reading at slave 1 starts
  IMU::write(0x2A, 0x81); //Enable at set length to 1
  IMU::write(0x64, 0x01); //override register
  IMU::write(0x67, 0x03); //set delay rate
  IMU::write(0x01, 0x80);
  
  IMU::write(0x34, 0x04); //set i2c slv4 delay
  IMU::write(0x64, 0x00); //override register
  IMU::write(0x6A, 0x00); //clear user setting
  IMU::write(0x64, 0x01); //override register
  IMU::write(0x6A, 0x20); //enable master i2c mode
  IMU::write(0x34, 0x13); //disable slv4
  
}

void IMU::setI2C_address(int address){
  this->I2C_address = address;
}


int IMU::read(int address){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(address);
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  
  return Wire.read();
}

int IMU::read(int addressL , int addressH){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addressL);
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  byte L = Wire.read();

  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addressH);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  byte H = Wire.read();

  return (int16_t)((H<<8)+L);
}

void IMU::readAcceleration( geometry_msgs::Vector3 *retur ) {
  retur->x = (float) read(MPU9150_ACCEL_XOUT_L, MPU9150_ACCEL_XOUT_H);
  retur->y = (float) read(MPU9150_ACCEL_YOUT_L, MPU9150_ACCEL_YOUT_H);
  retur->z = (float) read(MPU9150_ACCEL_ZOUT_L, MPU9150_ACCEL_ZOUT_H);
}


void IMU::readGyro( geometry_msgs::Vector3 *retur ) {
  retur->x = read(MPU9150_GYRO_XOUT_L, MPU9150_GYRO_XOUT_H);
  retur->y = read(MPU9150_GYRO_YOUT_L, MPU9150_GYRO_YOUT_H);
  retur->z = read(MPU9150_GYRO_ZOUT_L, MPU9150_GYRO_ZOUT_H);
}

void IMU::readCompas( geometry_msgs::Vector3 *retur ) {
  retur->x = read(MPU9150_CMPS_XOUT_L, MPU9150_CMPS_XOUT_H);
  retur->y = read(MPU9150_CMPS_YOUT_L, MPU9150_CMPS_YOUT_H);
  retur->z = read(MPU9150_CMPS_ZOUT_L, MPU9150_CMPS_ZOUT_H);
}


double IMU::readTemperature() {
  return  ( (double) read(MPU9150_TEMP_OUT_L,MPU9150_TEMP_OUT_H ) + 12412.0) / 340.0;
}


/*

String IMU::readAll(){


	this->temp = ( (double) this->read(MPU9150_TEMP_OUT_L,MPU9150_TEMP_OUT_H) + 12412.0) / 340.0;
	this->compas[0] = (int) this->read(MPU9150_CMPS_XOUT_L,MPU9150_CMPS_XOUT_H);
	this->compas[1] = this->read(MPU9150_CMPS_YOUT_L,MPU9150_CMPS_YOUT_H);
	this->compas[2] = this->read(MPU9150_CMPS_ZOUT_L,MPU9150_CMPS_ZOUT_H);
	this->gyro[0] = this->read(MPU9150_GYRO_XOUT_L,MPU9150_GYRO_XOUT_H);
	this->gyro[1] = this->read(MPU9150_GYRO_YOUT_L,MPU9150_GYRO_YOUT_H);
	this->gyro[2] = this->read(MPU9150_GYRO_ZOUT_L,MPU9150_GYRO_ZOUT_H);
	this->acceleration[0] = this->read(MPU9150_ACCEL_XOUT_L,MPU9150_ACCEL_XOUT_H);
	this->acceleration[1] = this->read(MPU9150_ACCEL_YOUT_L,MPU9150_ACCEL_YOUT_H);
	this->acceleration[2] = this->read(MPU9150_ACCEL_ZOUT_L,MPU9150_ACCEL_ZOUT_H);

	
	String str = ("Tp:" + static_cast<String> (temp) + ";" + "Cx:" + static_cast<int> (compas[0]) + ";" + "Cy:" + compas[1] + ";" + "Cz:" + compas[2] + ";" + "Gx:" + gyro[0] + ";" + "Gy:" + gyro[1] + ";" + "Gz:" + gyro[2] + ";" + "Ax:" + acceleration[0] + ";" + "Ax:" + acceleration[1] + ";" + "Ax:" + acceleration[2] + ";" ) ; 
	
	return str;
}

*/

int IMU::write(int address , int data){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission(true);

  return 1;
}


IMU::IMU(){
  //Serial.print("start setup: \n");
  //write(MPU9150_PWR_MGMT_1, 0);
  //CompasSetup();
  int compas[3] = { 0 , 0 , 0 };
  int acceleration[3] = { 0 , 0 , 0 };
  int gyro[3] = { 0 , 0 , 0 };
  double temp = 0;
  int I2C_address = MPU9150_I2C_ADDRESS;
  //Serial.print("end setup \n");
  
}



IMU::~IMU(){}

//IMU IMU;
