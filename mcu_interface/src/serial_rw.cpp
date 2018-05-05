#include "ros/ros.h"
#include "SerialCom.h"

int main(int argc, char **argv)
{
	SerialCom uart;

	ros::init(argc, argv, "mcu_interface");

	ros::NodeHandle nh_thruster_pwm, nh_light_pwm, nh_heartbeat, nh_arming;

	ros::Subscriber thruster_pwm_sub = nh_thruster_pwm.subscribe("/pwm", 1000, &SerialCom::thruster_pwm_callback, &uart);
	ros::Subscriber heartbeat_sub = nh_heartbeat.subscribe("/mcu_heartbeat", 1000, &SerialCom::heartbeat_callback, &uart);
	ros::Subscriber arming_sub = nh_arming.subscribe("/mcu_arm", 1000, &SerialCom::arming_callback, &uart);
	ros::Subscriber light_pwm_sub = nh_light_pwm.subscribe("/mcu_light_pwm", 1000, &SerialCom::light_pwm_callback, &uart);
	
	ros::spin();

	return 0;
}
