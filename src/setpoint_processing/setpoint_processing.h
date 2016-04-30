#ifndef SETPOINT_PROCESSING_H
#define SETPOINT_PROCESSING_H

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/FluidPressure.h"

#include "maelstrom_msgs/JoystickMotionCommand.h"
#include "uranus_dp/SetControlMode.h"
#include "uranus_dp/ResetStateEstimator.h"
#include "../control_mode_enum.h"

class SetpointProcessing
{
public:
    SetpointProcessing();
    void callback(const maelstrom_msgs::JoystickMotionCommand& msg);
private:
    ros::NodeHandle    nh;
    ros::Subscriber    joystickSub;
    ros::Publisher     wrenchPub;
    ros::Publisher     posePub;
    ros::Publisher     pressurePub;
    ros::ServiceClient modeClient;
    ros::ServiceClient resetClient;

    ControlMode control_mode;

    double depth_setpoint;
    ros::Time depth_setpoint_time;

    static const double MAX_FORCE  = 10; // Scale forces up to [-10, 10] (Newton)
    static const double MAX_TORQUE = 5;  // Scale torques up to [-5, 5] (Newton meters)

    void updateOpenLoop(const maelstrom_msgs::JoystickMotionCommand& msg);
    void updatePositionHold(const maelstrom_msgs::JoystickMotionCommand& msg);
    void updateDepthHold(const maelstrom_msgs::JoystickMotionCommand& msg);
    bool healthyMessage(const maelstrom_msgs::JoystickMotionCommand& msg);
};

#endif