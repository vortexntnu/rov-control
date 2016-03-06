#ifndef FUNKYCONTROLLER_H
#define FUNKYCONTROLLER_H

#include "ros/ros.h"
#include <Eigen/Dense>
#include <cmath>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"
#include "uranus_dp/State.h"

class NonlinearQuaternionPidController
{
public:
    NonlinearQuaternionPidController();
    void stateCallback(const uranus_dp::State &state_msg);
    void setpointCallback(const geometry_msgs::Twist &setpoint_msg);
    void compute(void);
    void setFrequency(unsigned int newFrequency);
private:
    ros::NodeHandle n;
    ros::Publisher  outputPub;
    ros::Subscriber stateSub;
    ros::Subscriber setpointSub;

    // Controller frequency
    unsigned int frequency;

    // State
    Eigen::Vector3d p;     // Position
    Eigen::Vector4d q;     // Orientation
    Eigen::Vector3d v;     // Linear velocity
    Eigen::Vector3d omega; // Angular velocity

    // Setpoints
    Eigen::Vector3d p_sp;     // Position
    Eigen::Vector4d q_sp;     // Orientation
    Eigen::Vector3d v_sp;     // Linear velocity
    Eigen::Vector3d omega_sp; // Angular velocity

    // Errors
    Eigen::Vector3d p_err;     // Position
    Eigen::Vector4d q_err;     // Orientation
    Eigen::Vector3d v_err;     // Linear velocity
    Eigen::Vector3d omega_err; // Angular velocity

    // Transformation matrices
    Eigen::Matrix3d R;
    Eigen::MatrixXd T;

    // Control output
    Eigen::VectorXd tau;

    // Controller gains
    Eigen::Matrix3d K_lin_d;
    Eigen::Matrix3d K_lin_p;
    Eigen::Matrix3d K_ang_d;
    Eigen::Matrix3d K_ang_p;

    void updateTransformationMatrices(void);
};

#endif