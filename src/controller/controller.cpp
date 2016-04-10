#include "ros/ros.h"
#include "quaternion_pd_controller.h"
#include "open_loop_controller.h"
#include "uranus_dp/ToggleControlMode.h"
#include "joystick/DirectionalInput.h"
#include "../control_mode_enum.h"

ControlMode control_mode;

bool toggleControlMode(uranus_dp::ToggleControlMode::Request &req, uranus_dp::ToggleControlMode::Response &resp)
{
    switch (control_mode)
    {
    case ControlModes::OPEN_LOOP:
        control_mode = ControlModes::STATIONKEEPING;
        ROS_INFO("Switching to stationkeeping control mode.");
        break;
    case ControlModes::STATIONKEEPING:
        control_mode = ControlModes::OPEN_LOOP;
        ROS_INFO("Switching to open loop control mode");
        break;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ROS_INFO("Launching node controller.");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("toggle_control_mode", toggleControlMode);

    control_mode = ControlModes::OPEN_LOOP;

    QuaternionPdController stationkeeper; // I hardly know her.
    OpenLoopController     openlooper;

    stationkeeper.disable();
    openlooper.enable();

    unsigned int frequency = 10; // To param server
    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {
        ros::spinOnce();
        stationkeeper.compute();
    }
    return 0;
}
