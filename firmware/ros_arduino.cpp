#include <ros.h>
#include <string.h>

#include "std_msgs/String.h"
#include "maelstrom_msgs/PwmRequests.h"
#include "maelstrom_msgs/ThrusterForces.h"
#include "maelstrom_msgs/LightInput.h"

#include "ForceToPwmLookup.h"

//includer for IMU og trykksensor
#include "MPU6050/MPU6050.h"
#include "MS5803_14BA.h"
#include <Wire.h>
#include <geometry_msgs/Vector3.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/FluidPressure.h"

#define STANDARD_GRAVITY 9.08665      // [m/s^2]
#define RAD_PER_DEG 0.01745329252     // [1/deg]
#define PASCAL_PER_MILLIBAR 0.01      // [Pa/mbar]
#define MICROTESLA_PER_TESLA 0.000001 // [uT/T]

#define MPU9150_I2C_ADDR 0x69
MPU6050 accelgyro(MPU9150_I2C_ADDR);

MS5803_14BA depthSensor;

ros::NodeHandle nh;

sensor_msgs::Imu imu_raw_msg;
sensor_msgs::MagneticField compass_msg;
sensor_msgs::Temperature temperature_msg;
sensor_msgs::FluidPressure pressure_msg;

ros::Publisher pub_imu("imu/data_raw", &imu_raw_msg);
ros::Publisher pub_mag("imu/mag", &compass_msg);
ros::Publisher pub_pressure("imu/pressure", &pressure_msg);
ros::Publisher pub_temperature("imu/temperature", &temperature_msg);

const int SensorReadDelay = 83;
unsigned long PrevoiusSensorReadMillis = 0;

int dbg_count = 0;

//Hold orden på pwm-pins til motor og lys
const int PwmCount = 7;
// const int PwmPins[PwmCount] = { 7, 8, 12, 13, 44, 45,      10};
const int PwmPins[PwmCount] = {13, 45, 44,  7,  8, 12, 10};
const int LigthPwmPin = PwmPins[PwmCount-1]; //pin 10 er på Timer / Counter 2
int PwmValue[PwmCount];


//Sett opp debugmeldinger
maelstrom_msgs::PwmRequests  pwm_status_msg;
std_msgs::String arduino_dbg_msg;

ros::Publisher pwm_status_pub("pwm_status", &pwm_status_msg);
ros::Publisher arduino_dbg_pub("arduino_dbg", &arduino_dbg_msg);


void WritePwm(int pin, uint8_t value) {
  switch(pin) {
    //LYS
  case 10:
    analogWrite(pin, value);
    break;

    //MOTOR
  case 7:
    OCR4BH = 0;
    OCR4BL = value;
    break;
  case 8:
    OCR4CH = 0;
    OCR4CL = value;
    break;
  case 12:
    OCR1BH = 0;
    OCR1BL = value;
    break;
  case 13:
    OCR1CH = 0;
    OCR1CL = value;
    break;
  case 44:
    OCR5BH = 0;
    OCR5BL = value;
    break;
  case 45:
    OCR5CH = 0;
    OCR5CL = value;
    break;
  };
}

void InitPwm() {

  //sett alle pwmpinnene som utputt
  for(int i = 0; i < PwmCount; i++)
    pinMode(PwmPins[i], OUTPUT);

  //initialiser Timer/Counter 1, 4 og 5
  TCCR1A |= (1<<COM1B1) | (1<<COM1C1)  | (1<<WGM11) | (1<<WGM10);
  TCCR4A |= (1<<COM4B1) | (1<<COM4C1)  | (1<<WGM41) | (1<<WGM40);
  TCCR5A |= (1<<COM5B1) | (1<<COM5C1)  | (1<<WGM51) | (1<<WGM50);

  //sett Clock select
  //på Timer/Counter 1
  TCCR1B |= (1<<CS11) | (1<<CS10);
  TCCR1B &= ~(1<<CS12);
  //På Timer/Counter 4
  TCCR4B |= (1<<CS41) | (1<<CS40);
  TCCR4B &= ~(1<<CS42);
  //på Timer/Counter 5
  TCCR5B |= (1<<CS51) | (1<<CS50);
  TCCR5B &= ~(1<<CS52);


  //sett alle motorer til 0 Newton
  for(int i = 0; i < PwmCount; i++) {
    PwmValue[i] = ForceToPwm(0);
    WritePwm(PwmPins[i], PwmValue[i]);
  }
  //og slå av lys
  PwmValue[LigthPwmPin] = 0;
  WritePwm(LigthPwmPin, 0);
  
}

void publishPwmStatus() {
  //Send PWM-verdiene tilbake som debugoutput
  pwm_status_msg.pwm1 = PwmValue[0];
  pwm_status_msg.pwm2 = PwmValue[1];
  pwm_status_msg.pwm3 = PwmValue[2];
  pwm_status_msg.pwm4 = PwmValue[3];
  pwm_status_msg.pwm5 = PwmValue[4];
  pwm_status_msg.pwm6 = PwmValue[5];
  pwm_status_msg.pwm7 = PwmValue[6];

  pwm_status_pub.publish( &pwm_status_msg );
}

void pwm_update( const maelstrom_msgs::ThrusterForces& force_input ){
  
  PwmValue[0] = ForceToPwm(force_input.F1);
  PwmValue[1] = ForceToPwm(force_input.F2);
  PwmValue[2] = ForceToPwm(force_input.F3);
  PwmValue[3] = ForceToPwm(force_input.F4);
  PwmValue[4] = ForceToPwm(force_input.F5);
  PwmValue[5] = ForceToPwm(force_input.F6);

  for(int i = 0; i < PwmCount; i++) {
    WritePwm(PwmPins[i], PwmValue[i]);
  }
    
  dbg_count = 0;

  publishPwmStatus();
}

void LightPwmUpdate( const maelstrom_msgs::LightInput &light_msg) {
  PwmValue[6] = light_msg.light_intensity;

  WritePwm(LigthPwmPin, PwmValue[6]);

  publishPwmStatus();
}

ros::Subscriber<maelstrom_msgs::ThrusterForces> pwm_input_sub("thruster_forces", &pwm_update );
ros::Subscriber<maelstrom_msgs::LightInput> light_pwm_input_sub("LightPwm", &LightPwmUpdate );


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
  
  InitPwm();

  //start ROS-node
  nh.initNode();

  nh.advertise(pwm_status_pub);
  nh.advertise(arduino_dbg_pub);
  
  nh.advertise(pub_imu);
  nh.advertise(pub_mag);
  nh.advertise(pub_temperature);
  nh.advertise(pub_pressure);
  
  nh.subscribe(pwm_input_sub);
  nh.subscribe(light_pwm_input_sub);
  
  nh.spinOnce();
  
  // Initialize the 'Wire' class for the I2C-bus.
  // I2C pins: 20 (SDA) og 21 (SCL) (Mega2560)
  Wire.begin();
  accelgyro.initialize();
  getFsRangeAndSetLsbSensisivity();
  //Sett lavpassfilter se http://www.i2cdevlib.com/docs/html/class_m_p_u6050.html#a9f2737fe22955fd85b2575ba8da874c6
  accelgyro.setDHPFMode(3); //acc 44Hz 4.9ms, gyro 42Hz 4.8ms

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

  temperature_msg.temperature = ( (double)accelgyro.getTemperature() + 12412.0) / 340.0;

  depthSensor.read();
  pressure_msg.fluid_pressure = depthSensor.getPreassure() * PASCAL_PER_MILLIBAR; // OBS! MÅ VÆRE I PASCAL

}

void loop(){

  nh.spinOnce();

  if( millis() - PrevoiusSensorReadMillis >= SensorReadDelay ) {
    PrevoiusSensorReadMillis = millis();

    lesSensorer();
    //nh.spinOnce();
    pub_imu.publish(&imu_raw_msg);
    pub_mag.publish(&compass_msg);
    pub_pressure.publish(&pressure_msg);  
    pub_temperature.publish(&temperature_msg);
  }
  
  
  
  /*
  //String dbg_msg = "pwm updated after " + String(dbg_count) + " cycles";
  //String dbg_msg = "accelX = " ;//+ String(imu.read(MPU9150_ACCEL_XOUT_L, MPU9150_ACCEL_XOUT_H));
  String dbg_msg = String("GyroFsRange = ") + String(GyroFsRange) +
  String("\nAccelFsRange = ") + String(AccelFsRange);
  arduino_dbg_msg.data = dbg_msg.c_str();
  arduino_dbg_pub.publish( &arduino_dbg_msg );  
  nh.spinOnce();
  */


}
