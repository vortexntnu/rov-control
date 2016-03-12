#include <string.h>

#include <ros.h>

#include <std_msgs/String.h>
#include <joystick/PwmRequests.h>


//incleder for IMU og trykksensor
#include "MPU6050/MPU6050.h"
#include "MS5803_14BA.h"
#include <Wire.h>
#include <ros_arduino/SensorRaw.h>
#include <geometry_msgs/Vector3.h>

#define MPU9150_I2C_ADDR 0x69
MPU6050 accelgyro(MPU9150_I2C_ADDR);

MS5803_14BA depthSensor;

ros::NodeHandle nh;

ros_arduino::SensorRaw sensor_raw_msg;
ros::Publisher pub_imu( "SensorRaw", &sensor_raw_msg);

//PWM-variable:
int dbg_count = 0;
const int pwm_count = 6;
int internal_pwm_out[pwm_count];
const int pwm_pins[] = { 3, 5, 6, 9, 10, 11 };


joystick::PwmRequests  pwm_status_msg;
ros::Publisher pwm_status_pub("PwmStatus", &pwm_status_msg);


std_msgs::String arduino_dbg_msg;
ros::Publisher arduino_dbg_pub("ArduinoDbg", &arduino_dbg_msg);



void pwm_update( const joystick::PwmRequests& pwm_input ){


    internal_pwm_out[0] = (pwm_input.pwm1 >> 8) + 128;
    internal_pwm_out[1] = (pwm_input.pwm2 >> 8) + 128;
    internal_pwm_out[2] = (pwm_input.pwm3 >> 8) + 128;
    internal_pwm_out[3] = (pwm_input.pwm4 >> 8) + 128;
    internal_pwm_out[4] = (pwm_input.pwm5 >> 8) + 128;
    internal_pwm_out[5] = (pwm_input.pwm6 >> 8) + 128;
    

    //Send PWM-verdiene tilbake som debugoutput
    pwm_status_msg.pwm1 = internal_pwm_out[0];
    pwm_status_msg.pwm2 = internal_pwm_out[1];
    pwm_status_msg.pwm3 = internal_pwm_out[2];
    pwm_status_msg.pwm4 = internal_pwm_out[3];
    pwm_status_msg.pwm5 = internal_pwm_out[4];
    pwm_status_msg.pwm6 = internal_pwm_out[5];

    dbg_count = 0;

    pwm_status_pub.publish( &pwm_status_msg );


    
}

//TODO: fiks navn på topic "pwm_requests"
ros::Subscriber<joystick::PwmRequests> pwm_input_sub("pwm_requests", &pwm_update );


double GyroLsbSens, AccelLsbSens;
void getFsRangeAndSetLsbSensisivity() {
  
  uint8_t GyroFsRange, AccelFsRange;
  
  GyroFsRange = accelgyro.getFullScaleGyroRange();
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

  
  AccelFsRange = accelgyro.getFullScaleAccelRange();
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

void setup() {
  //start ROS-node
  nh.initNode();

  nh.advertise(pwm_status_pub);
  nh.advertise(arduino_dbg_pub);
  
  nh.advertise(pub_imu);  
  nh.subscribe(pwm_input_sub);
  nh.spinOnce();
  
  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();
  accelgyro.initialize();
  getFsRangeAndSetLsbSensisivity();

  depthSensor.initialize(false);


  nh.spinOnce();
  String dbg_msg =  String("init:\n") + 
    (accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  arduino_dbg_msg.data = dbg_msg.c_str();
  arduino_dbg_pub.publish( &arduino_dbg_msg );

  nh.spinOnce();

}


void lesSensorer() {

  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t mx, my, mz;

  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  //Accelerometerdata enhet: [g]
  sensor_raw_msg.acceleration.x = ax / AccelLsbSens;
  sensor_raw_msg.acceleration.y = ay / AccelLsbSens;
  sensor_raw_msg.acceleration.z = az / AccelLsbSens;

  //Gyrodata: enhet [deg/s]
  sensor_raw_msg.gyro.x = gx / GyroLsbSens;
  sensor_raw_msg.gyro.y = gy / GyroLsbSens;
  sensor_raw_msg.gyro.z = gz / GyroLsbSens;

  //Kompass enhent [µT]
  //TODO fin ut om 0.3 er riktig skalerinsfaktor
  sensor_raw_msg.compass.x = mx * 0.3;
  sensor_raw_msg.compass.y = my * 0.3;
  sensor_raw_msg.compass.z = mz * 0.3;

  sensor_raw_msg.temperature = ( (double)accelgyro.getTemperature() + 12412.0) / 340.0;

  depthSensor.read();
  sensor_raw_msg.pressure = depthSensor.getPreassure();

  
}

void loop(){


  nh.spinOnce();
  lesSensorer();
  nh.spinOnce();
  pub_imu.publish(&sensor_raw_msg);
  
  nh.spinOnce();
  dbg_count++;
  nh.spinOnce();

  
  /*
  //String dbg_msg = "pwm updated after " + String(dbg_count) + " cycles";
  //String dbg_msg = "accelX = " ;//+ String(imu.read(MPU9150_ACCEL_XOUT_L, MPU9150_ACCEL_XOUT_H));
  String dbg_msg = String("GyroFsRange = ") + String(GyroFsRange) +
    String("\nAccelFsRange = ") + String(AccelFsRange);
  arduino_dbg_msg.data = dbg_msg.c_str();
  arduino_dbg_pub.publish( &arduino_dbg_msg );  
  nh.spinOnce();
  */
  
  
  //oppateringsfekvens på 100Hz
  delay(10);
  
  

  //delay(500);


  for(int i = 0; i < pwm_count; i++) {
    nh.spinOnce();
    analogWrite(pwm_pins[i], internal_pwm_out[i]);
  }


  
}
