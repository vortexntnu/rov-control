#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include <maelstrom_msgs/JoystickMotionCommand.h>
#include "uranus_dp/SetControlMode.h"
#include "control_mode_enum.h"

class SetpointProcessing
{
public:
    SetpointProcessing()
    {
        joystickSub = nh.subscribe("joystick_motion_command", 10, &SetpointProcessing::callback, this);
        wrenchPub   = nh.advertise<geometry_msgs::Wrench>("wrench_setpoints", 10);
        posePub     = nh.advertise<geometry_msgs::Pose>("pose_setpoints", 10);
        modeClient  = nh.serviceClient<uranus_dp::SetControlMode>("set_control_mode");

        control_mode = ControlModes::OPEN_LOOP;
    }

    void callback(const maelstrom_msgs::JoystickMotionCommand& msg)
    {
        if (msg.control_mode != control_mode)
        {
            control_mode = static_cast<ControlMode>(msg.control_mode);
            uranus_dp::SetControlMode srv;
            srv.request.mode = control_mode;
            if (!modeClient.call(srv))
                ROS_ERROR_STREAM("Failed to call service set_control_mode. New mode " << control_mode << " not activated.");
        }

        if (control_mode == ControlModes::OPEN_LOOP)
        {
            ROS_INFO("setpoint_processing: Sending open loop setpoints.");
            geometry_msgs::Wrench setpoint_msg;
            setpoint_msg.force.x  = msg.forward * MAX_FORCE;
            setpoint_msg.force.y  = msg.right   * MAX_FORCE;
            setpoint_msg.force.z  = msg.down    * MAX_FORCE;
            setpoint_msg.torque.x = 0;
            setpoint_msg.torque.y = msg.tilt_up    * MAX_TORQUE;
            setpoint_msg.torque.z = msg.turn_right * MAX_TORQUE;
            wrenchPub.publish(setpoint_msg);
        }
        else if (control_mode == ControlModes::POSITION_HOLD)
        {
            ROS_INFO("setpoint_processing: Sending position hold setpoints.");
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

    static const double MAX_FORCE  = 10; // Scale forces up to [-10, 10] (Newton)
    static const double MAX_TORQUE = 5;  // Scale torques up to [-5, 5] (Newton meters)
};

int main(int argc, char** argv){
    ros::init(argc, argv, "setpoint_processing");
    ROS_INFO("Launching node setpoint_processing.");
    SetpointProcessing s;
    ros::spin();
    return 0;
}
