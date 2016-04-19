#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include <joystick/DirectionalInput.h>
#include <string>
#include "uranus_dp/SetControlMode.h"
#include "control_mode_enum.h"

class SetpointProcessing
{
public:
    SetpointProcessing()
    {
        joystickSub = nh.subscribe("joy_input", 10, &SetpointProcessing::callback, this);
        wrenchPub   = nh.advertise<geometry_msgs::Wrench>("wrench_setpoints", 10);
        posePub     = nh.advertise<geometry_msgs::Pose>("pose_setpoints", 10);
        modeClient  = nh.serviceClient<uranus_dp::SetControlMode>("set_control_mode");

        control_mode = ControlModes::OPEN_LOOP;
    }

    void callback(const joystick::DirectionalInput &joy_msg)
    {
        if (joy_msg.control_mode != control_mode)
        {
            control_mode = static_cast<ControlMode>(joy_msg.control_mode);
            uranus_dp::SetControlMode srv;
            srv.request.mode = control_mode;
            if (!modeClient.call(srv))
            {
                ROS_ERROR_STREAM("Failed to call service set_control_mode. New mode " << control_mode << " not activated.");
            }
        }

        if (control_mode == ControlModes::OPEN_LOOP)
        {
            ROS_INFO("setpoint_processing: Sending open loop setpoints.");
            geometry_msgs::Wrench setpoint_msg;
            setpoint_msg.force.x  = joy_msg.strafe_X * NORMALIZATION * MAX_FORCE;
            setpoint_msg.force.y  = joy_msg.strafe_Y * NORMALIZATION * MAX_FORCE;
            setpoint_msg.force.z  = joy_msg.ascend   * NORMALIZATION * MAX_FORCE;
            setpoint_msg.torque.x = 0;
            setpoint_msg.torque.y = joy_msg.turn_X * NORMALIZATION * MAX_TORQUE;
            setpoint_msg.torque.z = joy_msg.turn_Y * NORMALIZATION * MAX_TORQUE;
            wrenchPub.publish(setpoint_msg);
        }
        else if (control_mode == ControlModes::STATIONKEEPING)
        {
            ROS_INFO("setpoint_processing: Sending stationkeeping setpoints.");
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

    ControlMode control_mode;

    static const double NORMALIZATION = 0.000030517578125; // Scale joystick inputs down to [-1, 1]
    static const double MAX_FORCE     = 10;                // Scale forces up to [-10, 10] (Newton)
    static const double MAX_TORQUE    = 5;                 // Scale torques up to [-5, 5] (Newton meters)
};

int main(int argc, char** argv){
    ros::init(argc, argv, "setpoint_processing");
    ROS_INFO("Launching node setpoint_processing.");
    SetpointProcessing s;
    ros::spin();
    return 0;
}
