#include <string.h>

#include <ros.h>

#include <std_msgs/String.h>
#include <joystick/pwm_requests.h>


//incleder for IMUen
#include "MPU9150.h"
#include <Wire.h>
#include <ros_arduino/imu_raw.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle nh;

IMU imu;
ros_arduino::imu_raw imu_msg;
ros::Publisher pub_imu( "imu_raw", &imu_msg);

//PWM-variable:
int dbg_count = 0;
const int pwm_count = 6;
int internal_pwm_out[pwm_count];
const int pwm_pins[] = { 3, 5, 6, 9, 10, 11 };


joystick::pwm_requests  pwm_status_msg;
ros::Publisher pwm_status_pub("pwm_status", &pwm_status_msg);


std_msgs::String arduino_dbg_msg;
ros::Publisher arduino_dbg_pub("arduino_dbg", &arduino_dbg_msg);

/*

void pwm_update( const joystick::pwm_requests& pwm_input ){


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

ros::Subscriber<joystick::pwm_requests> pwm_input_sub("pwm_requests", &pwm_update );

*/

void setup() {
  //start ROS-node
  nh.initNode();


  //start wire for IMUen
  Wire.begin();

  
  nh.advertise(pwm_status_pub);
  nh.advertise(arduino_dbg_pub);
  
  nh.advertise(pub_imu);
  
  //nh.subscribe(pwm_input_sub);

}


void lesIMU() {
  
  //imu.readAcceleration( &imu_msg.acceleration  );
  //imu.readGyro( &imu_msg.gyro );
  //imu.readCompas( &imu_msg.compas );
  
}

void loop(){

  nh.spinOnce();
  lesIMU();
  nh.spinOnce();
  //pub_imu.publish(&imu_msg);
  
  nh.spinOnce();
  dbg_count++;
  nh.spinOnce();

  
  /*  
  String dbg_msg = "pwm updated after " + String(dbg_count) + " cycles";
  nh.spinOnce();
  arduino_dbg_msg.data = dbg_msg.c_str();
  nh.spinOnce();
*/
  
  arduino_dbg_pub.publish( &arduino_dbg_msg );
  nh.spinOnce();
  
  
  
  //delay(1000/60);

  delay(1000);
  
  for(int i = 0; i < pwm_count; i++) {
    nh.spinOnce();
    analogWrite(pwm_pins[i], internal_pwm_out[i]);
  }

  
}
