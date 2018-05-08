#include "ros/ros.h"
#include "SerialCom.h"

int main(int argc, char **argv)
{
	SerialCom uart;

	ros::init(argc, argv, "mcu_interface");

	ros::NodeHandle nh_mcu;

	ros::Subscriber thruster_pwm_sub = nh_mcu.subscribe("/pwm", 1000, &SerialCom::thruster_pwm_callback, &uart);
	ros::Subscriber heartbeat_sub = nh_mcu.subscribe("/mcu_heartbeat", 1000, &SerialCom::heartbeat_callback, &uart);
	ros::Subscriber arming_sub = nh_mcu.subscribe("/mcu_arm", 1000, &SerialCom::arming_callback, &uart);
	ros::Subscriber light_pwm_sub = nh_mcu.subscribe("/mcu_light_pwm", 1000, &SerialCom::light_pwm_callback, &uart);

	ros::Publisher is_leak_pub = nh_mcu.advertise<std_msgs::UInt8>("/mcu_is_leak", 1000);

	ros::Rate loop_rate(1);

	std_msgs::UInt8 msg;
	msg.data = 0;

	while (ros::ok())
	{		
		
	
		if(uart.is_leak())
		{
			msg.data = 1;
		}
		else
		{
			msg.data = 0;
		}

		is_leak_pub.publish(msg);
			

		ros::spinOnce();

		loop_rate.sleep();
	}

	//ros::spin();

	return 0;
}
