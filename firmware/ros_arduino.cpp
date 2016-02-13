#include <string.h>

#include <ros.h>

#include <std_msgs/String.h>
#include <joystick/pwm_requests.h>


ros::NodeHandle nh;

int dbg_count = 0;

const int pwm_count = 6;
int internal_pwm_out[pwm_count];



joystick::pwm_requests  pwm_status_msg;
ros::Publisher pwm_status_pub("pwm_status", &pwm_status_msg);

std_msgs::String arduino_dbg_msg;
ros::Publisher arduino_dbg_pub("arduino_dbg", &arduino_dbg_msg);



void pwm_update( const joystick::pwm_requests& pwm_input ){
  
    internal_pwm_out[0] = (pwm_input.pwm1 >> 8) + 128;
    internal_pwm_out[1] = (pwm_input.pwm2 >> 8) + 128;
    internal_pwm_out[2] = (pwm_input.pwm3 >> 8) + 128;
    internal_pwm_out[3] = (pwm_input.pwm4 >> 8) + 128;
    internal_pwm_out[4] = (pwm_input.pwm5 >> 8) + 128;
    internal_pwm_out[5] = (pwm_input.pwm6 >> 8) + 128;
 
    

    pwm_status_msg.pwm1 = internal_pwm_out[0];
    pwm_status_msg.pwm2 = internal_pwm_out[1];
    pwm_status_msg.pwm3 = internal_pwm_out[2];
    pwm_status_msg.pwm4 = internal_pwm_out[3];
    pwm_status_msg.pwm5 = internal_pwm_out[4];
    pwm_status_msg.pwm6 = internal_pwm_out[5];


    pwm_status_pub.publish( &pwm_status_msg );


    
}

ros::Subscriber<joystick::pwm_requests> pwm_input_sub("pwm_signal_input", &pwm_update );



void setup() {
    nh.initNode();
    
    nh.advertise(pwm_status_pub);
    nh.advertise(arduino_dbg_pub);

    nh.subscribe(pwm_input_sub);
}

void loop(){

    
    nh.spinOnce();
    dbg_count++;
    nh.spinOnce();
  
    
    String dbg_msg = "pwm updated after " + String(dbg_count) + " cycles";
    nh.spinOnce();
     arduino_dbg_msg.data = dbg_msg.c_str();
    nh.spinOnce();
    arduino_dbg_pub.publish( &arduino_dbg_msg );
    nh.spinOnce();
    
    
    delay(1000/60);
    nh.spinOnce();
    analogWrite(11, internal_pwm_out[0]);
    
}
