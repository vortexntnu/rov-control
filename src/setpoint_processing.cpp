#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include <joystick/DirectionalInput.h>
#include <string>
#include "uranus_dp/ToggleControlMode.h"

class SetpointProcessing
{
public:
    SetpointProcessing()
    {
        joystickSub = nh.subscribe("joy_input", 1, &SetpointProcessing::callback, this);
        wrenchPub   = nh.advertise<geometry_msgs::Wrench>("open_loop_setpoint", 1);
        posePub     = nh.advertise<geometry_msgs::Pose>("stationkeeping_setpoint", 1);
        modeClient  = nh.serviceClient<uranus_dp::ToggleControlMode>("toggle_control_mode");

        control_mode = joystick::DirectionalInput::OPEN_LOOP;
    }

    void callback(const joystick::DirectionalInput &joy_msg)
    {
        if (joy_msg.control_mode != control_mode)
        {
            control_mode = joy_msg.control_mode;
            uranus_dp::ToggleControlMode srv;
            if (!modeClient.call(srv))
            {
                ROS_ERROR_STREAM("Failed to call service toggle_control_mode. New mode " << controlModeString(control_mode) << " not activated.");
            }
        }

        if (control_mode == joystick::DirectionalInput::OPEN_LOOP)
        {
            geometry_msgs::Wrench setpoint_msg;
            setpoint_msg.force.x  = joy_msg.strafe_X * NORMALIZATION * SCALING_LIN;
            setpoint_msg.force.y  = joy_msg.strafe_Y * NORMALIZATION * SCALING_LIN;
            setpoint_msg.force.z  = 0;
            setpoint_msg.torque.x = 0;
            setpoint_msg.torque.y = joy_msg.turn_X * NORMALIZATION * SCALING_ANG;
            setpoint_msg.torque.z = joy_msg.turn_Y * NORMALIZATION * SCALING_ANG;
            wrenchPub.publish(setpoint_msg);
        }
        else if (control_mode == joystick::DirectionalInput::STATIONKEEPING)
        {
            // Todo: Actually populate the pose message with values
            geometry_msgs::Pose setpoint_msg;
            setpoint_msg.position.x    = 0;
            setpoint_msg.position.y    = 0;
            setpoint_msg.position.z    = 0;
            setpoint_msg.orientation.x = 0;
            setpoint_msg.orientation.y = 0;
            setpoint_msg.orientation.z = 0;
            setpoint_msg.orientation.w = 0;
            posePub.publish(setpoint_msg);
        }
        else
        {
            ROS_ERROR_STREAM("Invalid mode " << control_mode << " detected. Will not send setpoint message.");
        }
    }

private:
    ros::NodeHandle    nh;
    ros::Subscriber    joystickSub;
    ros::Publisher     wrenchPub;
    ros::Publisher     posePub;
    ros::ServiceClient modeClient;

    int control_mode;

    static const double NORMALIZATION = 0.000030517578125; // Scale joystick inputs down to [-1, 1]
    static const double SCALING_LIN   = 10;                // Scale forces up to [-10, 10] (Newton)
    static const double SCALING_ANG   = 2;                 // Scale torques up to [-2, 2] (Newton meters)

    std::string controlModeString(int mode)
    {
        if (mode == joystick::DirectionalInput::OPEN_LOOP)
            return "open loop";
        else if (mode == joystick::DirectionalInput::STATIONKEEPING)
            return "stationkeeping";
        else
            return "invalid mode";
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "setpoint_processing");
    ROS_INFO("Launching node setpoint_processing.");
    SetpointProcessing s;
    ros::spin();
    return 0;
}
