#include "ros/ros.h"
#include "quaternion_pd_controller.h"
#include "open_loop_controller.h"
#include "uranus_dp/ToggleControlMode.h"
#include "joystick/DirectionalInput.h"

int control_mode; // Maybe turn this whole thing into a class with control_mode as a member

bool toggleControlMode(uranus_dp::ToggleControlMode::Request &req, uranus_dp::ToggleControlMode::Response &resp)
{
    if (control_mode == joystick::DirectionalInput::OPEN_LOOP)
    {
        control_mode = joystick::DirectionalInput::STATIONKEEPING;
        ROS_INFO("Switching to stationkeeping control mode.");
    }
    else if (control_mode == joystick::DirectionalInput::STATIONKEEPING)
    {
        control_mode = joystick::DirectionalInput::OPEN_LOOP;
        ROS_INFO("Switching to open loop control mode");
    }
    else
    {
        ROS_ERROR("Invalid mode");
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ROS_INFO("Launching node controller.");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("toggle_control_mode", toggleControlMode);

    control_mode = joystick::DirectionalInput::OPEN_LOOP;

    QuaternionPdController stationkeeper; // I hardly know her.
    OpenLoopController     openlooper;

    unsigned int frequency = 10; // To param server
    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {
        ros::spinOnce();
        if (control_mode == joystick::DirectionalInput::STATIONKEEPING)
            stationkeeper.compute();
        loop_rate.sleep();
    }
    return 0;
}
