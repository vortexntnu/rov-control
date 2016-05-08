#include "ros/ros.h"

#include <bcm2835.h>
#include "I2Cdev/I2Cdev.h"
#include "MPU6050/MPU6050.h"
#include "MS5803.h"


#include <geometry_msgs/Vector3.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/FluidPressure.h"

#define MPU9150_I2C_ADDR 0x69

#define STANDARD_GRAVITY 9.08665      // [m/s^2]
#define RAD_PER_DEG 0.01745329252     // [1/deg]
#define PASCAL_PER_MILLIBAR 0.01      // [Pa/mbar]
#define MICROTESLA_PER_TESLA 0.000001 // [uT/T]

sensor_msgs::Imu imu_raw_msg;
sensor_msgs::MagneticField compass_msg;
sensor_msgs::Temperature temperature_msg;
sensor_msgs::FluidPressure pressure_msg;

double GyroLsbSens, AccelLsbSens;
void getFsRangeAndSetLsbSensisivity(MPU6050 *accelgyro) {
  
  uint8_t GyroFsRange, AccelFsRange;
  
  GyroFsRange = accelgyro->getFullScaleGyroRange();
  /* Gyro
   * FS_SEL | Full Scale Range   | LSB Sensitivity
   * -------+--------------------+----------------
   * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
   * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
   * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
   * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
   */
  switch(GyroFsRange) {
  case 0:
    GyroLsbSens = 131.0;
    break;
  case 1:
    GyroLsbSens = 65.5;
    break;
  case 2:
    GyroLsbSens = 32.8;
    break;
  case 3:
    GyroLsbSens = 16.4;
    break;  
  };

  
  AccelFsRange = accelgyro->getFullScaleAccelRange();
  /*Accelerometer
   * AFS_SEL | Full Scale Range | LSB Sensitivity
   * --------+------------------+----------------
   * 0       | +/- 2g           | 16384 LSB/g
   * 1       | +/- 4g           | 8192 LSB/g
   * 2       | +/- 8g           | 4096 LSB/g
   * 3       | +/- 16g          | 2048 LSB/g
   */
  switch(AccelFsRange) {
  case 0:
    AccelLsbSens = 16384.0;
    break;
  case 1:
    AccelLsbSens = 8192.0;
    break;
  case 2:
    AccelLsbSens = 4096.0;
    break;
  case 3:
    AccelLsbSens = 2048.0;
    break;  
  };

}

void lesSensorer(MPU6050 *accelgyro, MS5803 *presstemp) {

  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t mx, my, mz;

  accelgyro->getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  //std::cout << ax << " " << ay << " " << az << " "  << mx  << " " << my << " " << mz << std::endl; 


  //Accelerometerdata enhet: [m/s^2]
  imu_raw_msg.linear_acceleration.x = (ax * STANDARD_GRAVITY) / AccelLsbSens; // OBS! MÅ VÆRE m/s^2!
  imu_raw_msg.linear_acceleration.y = (ay * STANDARD_GRAVITY) / AccelLsbSens; // OBS! MÅ VÆRE m/s^2!
  imu_raw_msg.linear_acceleration.z = (az * STANDARD_GRAVITY) / AccelLsbSens; // OBS! MÅ VÆRE m/s^2!

  //Gyrodata: enhet [rad/s]
  imu_raw_msg.angular_velocity.x = (gx * RAD_PER_DEG) / GyroLsbSens; // OBS! MÅ VÆRE RAD/SEC
  imu_raw_msg.angular_velocity.y = (gy * RAD_PER_DEG) / GyroLsbSens; // OBS! MÅ VÆRE RAD/SEC
  imu_raw_msg.angular_velocity.z = (gz * RAD_PER_DEG) / GyroLsbSens; // OBS! MÅ VÆRE RAD/SEC

  // Kompass, enhet [T]
  // TODO finn ut om 0.3 er riktig skaleringsfaktor
  compass_msg.magnetic_field.x = mx * 0.3 * MICROTESLA_PER_TESLA; // OBS! MÅ VÆRE TESLA! (IKKE MILLI/MICRO)
  compass_msg.magnetic_field.y = my * 0.3 * MICROTESLA_PER_TESLA; // OBS! MÅ VÆRE TESLA! (IKKE MILLI/MICRO)
  compass_msg.magnetic_field.z = mz * 0.3 * MICROTESLA_PER_TESLA; // OBS! MÅ VÆRE TESLA! (IKKE MILLI/MICRO)

  temperature_msg.temperature = ( (double)accelgyro->getTemperature() + 12412.0) / 340.0;

  
  presstemp->calcMeasurements(ADC_4096);
  pressure_msg.fluid_pressure = presstemp->getPress_mBar() * PASCAL_PER_MILLIBAR; // OBS! MÅ VÆRE I PASCAL
  

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "IMUNode");
  ros::NodeHandle nh;
  ROS_INFO("Launching IMUNode.");
  
  ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);
  ros::Publisher pub_mag = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 1000);
  ros::Publisher pub_pressure = nh.advertise<sensor_msgs::FluidPressure>("imu/pressure", 1000);
  ros::Publisher pub_temperature = nh.advertise<sensor_msgs::Temperature>("imu/temperature", 1000);
  
  I2Cdev::initialize();

  
  ROS_INFO("Initializing MS5803-14BA");
  MS5803 presstemp(0x76);
  presstemp.initialize(14); // MS5803-14BA
  ROS_INFO((presstemp.testConnection() ? "We are communicating with MS5803 via I2C." : "I2C Communications with MS5803 failed."));
  presstemp.setDebug(false);
  

  ROS_INFO("Initializing MPU9150");
  MPU6050 accelgyro(MPU9150_I2C_ADDR) ;
  accelgyro.initialize();
  getFsRangeAndSetLsbSensisivity(&accelgyro);
  //Sett lavpassfilter se http://www.i2cdevlib.com/docs/html/class_m_p_u6050.html#a9f2737fe22955fd85b2575ba8da874c6
  accelgyro.setDHPFMode(3); //acc 44Hz 4.9ms, gyro 42Hz 4.8ms
  ROS_INFO((accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed"));

  ROS_INFO("Running");
  
  ros::Rate rate(24.);
  while( ros::ok() ) {

    lesSensorer(&accelgyro, &presstemp);
    
    pub_imu.publish(imu_raw_msg);
    pub_mag.publish(compass_msg);
    pub_pressure.publish(pressure_msg);  
    pub_temperature.publish(temperature_msg);
    
    ros::spinOnce();
    
    rate.sleep();
  }
  
  return 0;

}
