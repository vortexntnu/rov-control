#include "ros/ros.h"
#include "SerialCom.h"

int main(int argc, char **argv)
{
	SerialCom uart;

	ros::init(argc, argv, "mcu_interface");

	ros::NodeHandle nh_pwm, nh_heartbeat;

	ros::Subscriber pwm_sub = nh_pwm.subscribe("/pwm", 1000, &SerialCom::thruster_pwm_callback, &uart);
	ros::Subscriber heartbeat_sub = nh_heartbeat.subscribe("/mcu_heartbeat", 1000, &SerialCom::heartbeat_callback, &uart);
	
	ros::spin();

	return 0;
}