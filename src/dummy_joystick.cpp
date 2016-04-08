#include "ros/ros.h"
#include "joystick/Joystick.h"
#include "geometry_msgs/Wrench.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummyJoystick");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);
    ros::Publisher joy_pub = nh.advertise<joystick::Joystick>("joystick", 10);
    ROS_INFO("Launching node dummy_joystick.");

    joystick::Joystick joy_msg;
    ros::spinOnce();
    loop_rate.sleep();

    joy_msg.strafe_X = -29051;
    joy_msg.strafe_Y = -27049;
    joy_msg.turn_X   =  24354;
    joy_msg.turn_Y   = - 7259;
    ROS_INFO("Will publish 1st joy msg now...");
    joy_pub.publish(joy_msg);
    ros::spinOnce();
    loop_rate.sleep();

    joy_msg.strafe_X = -31386;
    joy_msg.strafe_Y =  14644;
    joy_msg.turn_X   =    387;
    joy_msg.turn_Y   =   1261;
    ROS_INFO("Will publish 2nd joy msg now...");
    joy_pub.publish(joy_msg);
    ros::spinOnce();
    loop_rate.sleep();

    joy_msg.strafe_X = -11246;
    joy_msg.strafe_Y =  24472;
    joy_msg.turn_X   = -17748;
    joy_msg.turn_Y   =  11321;
    ROS_INFO("Will publish 3rd joy msg now...");
    joy_pub.publish(joy_msg);
    ros::spinOnce();
    loop_rate.sleep();

    joy_msg.strafe_X = -10374;
    joy_msg.strafe_Y = -10121;
    joy_msg.turn_X   = -21393;
    joy_msg.turn_Y   = - 8417;
    ROS_INFO("Will publish 4th joy msg now...");
    joy_pub.publish(joy_msg);
    ros::spinOnce();
    loop_rate.sleep();

    joy_msg.strafe_X =   4845;
    joy_msg.strafe_Y = -19688;
    joy_msg.turn_X   =  22314;
    joy_msg.turn_Y   =  29198;
    ROS_INFO("Will publish 5th joy msg now...");
    joy_pub.publish(joy_msg);
    ros::spinOnce();
    loop_rate.sleep();

    return 0;
}
