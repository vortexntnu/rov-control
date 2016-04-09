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

//PWM-variable:
int dbg_count = 0;
const int PwmCount = 6;
int PwmOnTime[PwmCount];
const int PwmPins[PwmCount] = { 3, 5, 6, 9, 10, 11 };
const int PwmTotalTime = 3600;
int PwmPinState[PwmCount] = { LOW, LOW, LOW, LOW, LOW, LOW };
unsigned long PreviousToggleMicros[PwmCount] = { 0, 0, 0, 0, 0, 0 };



joystick::PwmRequests  pwm_status_msg;
ros::Publisher pwm_status_pub("PwmStatus", &pwm_status_msg);


std_msgs::String arduino_dbg_msg;
ros::Publisher arduino_dbg_pub("ArduinoDbg", &arduino_dbg_msg);



void pwm_update( const uranus_dp::ThrusterForces& force_input ){
  
  PwmOnTime[0] = ForceToPwm(force_input.F1);
  PwmOnTime[1] = ForceToPwm(force_input.F2);
  PwmOnTime[2] = ForceToPwm(force_input.F3);
  PwmOnTime[3] = ForceToPwm(force_input.F4);
  PwmOnTime[4] = ForceToPwm(force_input.F5);
  PwmOnTime[5] = ForceToPwm(force_input.F6);
  
  
  //Send PWM-verdiene tilbake som debugoutput
  pwm_status_msg.pwm1 = PwmOnTime[0];
  pwm_status_msg.pwm2 = PwmOnTime[1];
  pwm_status_msg.pwm3 = PwmOnTime[2];
  pwm_status_msg.pwm4 = PwmOnTime[3];
  pwm_status_msg.pwm5 = PwmOnTime[4];
  pwm_status_msg.pwm6 = PwmOnTime[5];
  
  dbg_count = 0;
  
  pwm_status_pub.publish( &pwm_status_msg );
  
  
  
}

ros::Subscriber<uranus_dp::ThrusterForces> pwm_input_sub("control_inputs", &pwm_update );


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


  //sett alle motorer til 0 Newton
  for(int i = 0; i < PwmCount; i++) {
    PwmOnTime[i] = ForceToPwm(0);
  }
  

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

  /*
    //komenterte vekk IMU for å kunne bruke software-PWM uten forstyrrelser

  if( millis() - PrevoiusSensorReadMillis >= SensorReadDelay ) {
    PrevoiusSensorReadMillis = millis();
  
    lesSensorer();
    //nh.spinOnce();
    pub_imu.publish(&sensor_raw_msg);
    
    //nh.spinOnce();
    //dbg_count++;
    //nh.spinOnce();

    
    
    
  }
  */
  
  
  /*
  //String dbg_msg = "pwm updated after " + String(dbg_count) + " cycles";
  //String dbg_msg = "accelX = " ;//+ String(imu.read(MPU9150_ACCEL_XOUT_L, MPU9150_ACCEL_XOUT_H));
  String dbg_msg = String("GyroFsRange = ") + String(GyroFsRange) +
  String("\nAccelFsRange = ") + String(AccelFsRange);
  arduino_dbg_msg.data = dbg_msg.c_str();
  arduino_dbg_pub.publish( &arduino_dbg_msg );  
  nh.spinOnce();
  */


  //Software-PWM
  for(int i = 0; i < PwmCount; i++) {
     nh.spinOnce();
    
    unsigned long CurrentMicros = micros();
    if(PwmPinState[i] == HIGH) {
      if(CurrentMicros - PreviousToggleMicros[i] >= PwmOnTime[i]) {
	PwmPinState[i] = LOW;
	digitalWrite(PwmPins[i], PwmPinState[i]);
	PreviousToggleMicros[i] = CurrentMicros;

	
      }
    } else { // (PwmPinState[i] == LOW)
      if(CurrentMicros - PreviousToggleMicros[i] >= PwmTotalTime - PwmOnTime[i] ) {
	PwmPinState[i] = HIGH;
	digitalWrite(PwmPins[i], PwmPinState[i]);
	PreviousToggleMicros[i] = CurrentMicros;
      }
      
    }

    
    //analogWrite(pwm_pins[i], PwmOnTime[i]);
    
  }


  
}
