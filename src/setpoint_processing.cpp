#include "setpoint_processing.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "setpoint_processing");
    ROS_INFO("Launching node setpoint_processing.");
    SetpointProcessing s;
    ros::spin();
    return 0;
}

SetpointProcessing::SetpointProcessing()
{
    joystickSub = nh.subscribe("joystick_motion_command", 10, &SetpointProcessing::callback, this);
    wrenchPub   = nh.advertise<geometry_msgs::Wrench>("wrench_setpoints", 10);
    posePub     = nh.advertise<geometry_msgs::Pose>("pose_setpoints", 10);
    modeClient  = nh.serviceClient<uranus_dp::SetControlMode>("set_control_mode");
    resetClient = nh.serviceClient<uranus_dp::ResetStateEstimator>("reset_state_estimator");

    control_mode = ControlModes::OPEN_LOOP;
}

void SetpointProcessing::callback(const maelstrom_msgs::JoystickMotionCommand& msg)
{
    if (!healthyMessage(msg))
    {
        ROS_WARN("setpoint_processing: Joystick motion command message out of range, ignoring...");
        return;
    }

    if (msg.control_mode != control_mode)
    {
        control_mode = static_cast<ControlMode>(msg.control_mode);
        if (control_mode == ControlModes::POSITION_HOLD)
        {
                    // Reset estimator when mode changes to position hold
            uranus_dp::ResetStateEstimator reset_srv;
            resetClient.call(reset_srv);
        }

        uranus_dp::SetControlMode srv;
        srv.request.mode = control_mode;
        if (!modeClient.call(srv))
            ROS_ERROR_STREAM("Failed to call service set_control_mode. New mode " << control_mode << " not activated.");
    }

    switch (control_mode)
    {
        case ControlModes::OPEN_LOOP:
        {
            ROS_INFO("setpoint_processing: Sending OPEN_LOOP setpoints.");
            geometry_msgs::Wrench open_loop_msg;
            open_loop_msg.force.x  = msg.forward * MAX_FORCE;
            open_loop_msg.force.y  = msg.right   * MAX_FORCE;
            open_loop_msg.force.z  = msg.down    * MAX_FORCE;
            open_loop_msg.torque.x = 0;
            open_loop_msg.torque.y = msg.tilt_up    * MAX_TORQUE;
            open_loop_msg.torque.z = msg.turn_right * MAX_TORQUE;
            wrenchPub.publish(open_loop_msg);
            break;
        }
        case ControlModes::POSITION_HOLD:
        {
            ROS_INFO("setpoint_processing: Sending POSITION_HOLD setpoints.");
                    // Todo: Actually populate the pose message with values
            geometry_msgs::Pose position_hold_msg;
            position_hold_msg.position.x    = msg.forward;
            position_hold_msg.position.y    = msg.right;
            position_hold_msg.position.z    = msg.down;
            position_hold_msg.orientation.x = 0;
            position_hold_msg.orientation.y = 0;
            position_hold_msg.orientation.z = 0;
            position_hold_msg.orientation.w = 1;
            posePub.publish(position_hold_msg);
            break;
        }
        default:
        {
            ROS_WARN("setpoint_processing: Default control mode switch case reached.");
        }
    }
}

bool SetpointProcessing::healthyMessage(const maelstrom_msgs::JoystickMotionCommand& msg)
{
    if (abs(msg.forward) > 1)
    {
        ROS_WARN("setpoint_processing: Forward motion command out of range");
        return false;
    }
    if (abs(msg.right) > 1)
    {
        ROS_WARN("setpoint_processing: Right motion command out of range");
        return false;
    }
    if (abs(msg.down) > 1)
    {
        ROS_WARN("setpoint_processing: Down motion command out of range.");
        return false;
    }
    if (abs(msg.tilt_up) > 1)
    {
        ROS_WARN("setpoint_processing: Tilt up motion command out of range");
        return false;
    }
    if (abs(msg.turn_right) > 1)
    {
        ROS_WARN("setpoint_processing: Turn right motion command out of range");
        return false;
    }

    bool validControlMode = (msg.control_mode == ControlModes::OPEN_LOOP || msg.control_mode == ControlModes::POSITION_HOLD);
    if (!validControlMode)
    {
        ROS_WARN("setpoint_processing: Invalid control mode.");
        return false;
    }
    return true;
}
