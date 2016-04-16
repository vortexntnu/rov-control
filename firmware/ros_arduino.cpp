#include <string.h>

#include <ros.h>

#include <std_msgs/String.h>
#include <joystick/PwmRequests.h>
#include <uranus_dp/ThrusterForces.h>

#include "ForceToPwmLookup.h"


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

const int SensorReadDelay = 83;
unsigned long PrevoiusSensorReadMillis = 0;

int dbg_count = 0;

//Hold orden på pwm-pins og hvilke rigeistre om må settes for å endre on-time
const int PwmCount = 6;
const int PwmPins[PwmCount] = { 2, 3, 7, 8, 12, 13 };
int PwmValue[PwmCount];


void WritePwm(int pin, uint8_t value) {
  switch(pin) {
  case 2:
    OCR3BH = 0;
    OCR3BL = value;
    break;
  case 3:
    OCR3CH = 0;
    OCR3CL = value;
    break;
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
  };
}


void InitPwm() {

  //sett alle pwmpinnene som utputt
  for(int i = 0; i < PwmCount; i++)
    pinMode(PwmPins[i], OUTPUT);

  //initialiser Timer/Counter 1, 3 og 4
  TCCR4A |= (1<<COM4B1) | (1<<COM4C1)  | (1<<WGM41) | (1<<WGM40);
  TCCR3A |= (1<<COM3B1) | (1<<COM3C1)  | (1<<WGM31) | (1<<WGM30);
  TCCR1A |= (1<<COM1B1) | (1<<COM1C1)  | (1<<WGM11) | (1<<WGM10);

  //sett Clock select
  //På Timer/Counter 4
  TCCR4B |= (1<<CS41) | (1<<CS40);
  TCCR4B &= ~(1<<CS42);
  //på Timer/Counter 3
  TCCR3B |= (1<<CS31) | (1<<CS30);
  TCCR3B &= ~(1<<CS32);
  //på Timer/Counter 1
  TCCR1B |= (1<<CS11) | (1<<CS10);
  TCCR1B &= ~(1<<CS12);


  //sett alle motorer til 0 Newton
  for(int i = 0; i < PwmCount; i++) {
    PwmValue[i] = ForceToPwm(0);
    WritePwm(PwmPins[i], PwmValue[i]);
  }
  
  
}


//const int PwmTotalTime = 3600;
//int PwmPinState[PwmCount] = { LOW, LOW, LOW, LOW, LOW, LOW };
//unsigned long PreviousToggleMicros[PwmCount] = { 0, 0, 0, 0, 0, 0 };



joystick::PwmRequests  pwm_status_msg;
ros::Publisher pwm_status_pub("PwmStatus", &pwm_status_msg);


std_msgs::String arduino_dbg_msg;
ros::Publisher arduino_dbg_pub("ArduinoDbg", &arduino_dbg_msg);



void pwm_update( const uranus_dp::ThrusterForces& force_input ){
  
  PwmValue[0] = ForceToPwm(force_input.F1);
  PwmValue[1] = ForceToPwm(force_input.F2);
  PwmValue[2] = ForceToPwm(force_input.F3);
  PwmValue[3] = ForceToPwm(force_input.F4);
  PwmValue[4] = ForceToPwm(force_input.F5);
  PwmValue[5] = ForceToPwm(force_input.F6);

  for(int i = 0; i < PwmCount; i++) {
    WritePwm(PwmPins[i], PwmValue[i]);
  }
  
  
  //Send PWM-verdiene tilbake som debugoutput
  pwm_status_msg.pwm1 = PwmValue[0];
  pwm_status_msg.pwm2 = PwmValue[1];
  pwm_status_msg.pwm3 = PwmValue[2];
  pwm_status_msg.pwm4 = PwmValue[3];
  pwm_status_msg.pwm5 = PwmValue[4];
  pwm_status_msg.pwm6 = PwmValue[5];
  
  dbg_count = 0;
  
  pwm_status_pub.publish( &pwm_status_msg );
  
  
  
}

<<<<<<< HEAD
ros::Subscriber<uranus_dp::ThrusterForces> pwm_input_sub("thruster_forces", &pwm_update );
=======
ros::Subscriber<uranus_dp::ThrusterForces> pwm_input_sub("control_inputs", &pwm_update );
>>>>>>> cb65ecb0801c3b26d3877fc7748033a45a4bf3e6


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
  /*
  //still ned hovedklokkefrekvensen
  CLKPR = 0b10000000;
  CLKPR = 0x3; // del på 8 (2^3)
  delay(1);
  */
  
  InitPwm();

  
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

  
  if( millis() - PrevoiusSensorReadMillis >= SensorReadDelay ) {
    PrevoiusSensorReadMillis = millis();
  
    lesSensorer();
    //nh.spinOnce();
    pub_imu.publish(&sensor_raw_msg);
    
    //nh.spinOnce();
    //dbg_count++;
    //nh.spinOnce();

    
    
    
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
