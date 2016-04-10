#ifndef QUATERNION_PD_CONTROLLER_H
#define QUATERNION_PD_CONTROLLER_H

#include "ros/ros.h"
#include "uranus_dp/State.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

// typedef Eigen::Matrix<double,6,6> Eigen::Matrix6d;
// typedef Eigen::Matrix<double,6,1> Eigen::Vector6d;

// Change name to reflect that this is a position and attitude controller, not a velocity controller
// Maybe "StationkeepingController"
class QuaternionPdController
{
public:
    QuaternionPdController();
    void stateCallback(const uranus_dp::State &stateMsg);
    void setpointCallback(const geometry_msgs::Pose &setpointMsg);
    void compute();
    void enable();
    void disable();
private:
    ros::NodeHandle nh;
    ros::Subscriber stateSub;
    ros::Subscriber setpointSub;
    ros::Publisher  controlInputPub;
    bool enabled;

    void updateProportionalGainMatrix();
    void updateErrorVector();
    void updateRestoringForceVector();

    // State
    Eigen::Vector3d    p;           // Position
    Eigen::Quaterniond q;           // Orientation
    Eigen::Matrix<double, 6, 1> nu; // Velocity (linear and angular)
    
    // Setpoints
    Eigen::Vector3d    p_d; // Desired position
    Eigen::Quaterniond q_d; // Desired attitude
    
    // Error variables
    Eigen::Matrix<double,6,1> z; // (6) Pose error vector
    
    // Output
    Eigen::Matrix<double,6,1> tau; // (6) Control forces

    // Other stuff
    Eigen::Matrix<double,6,1> g; // (6)   Restoring force vector
    Eigen::Matrix<double,3,3> R; // (3*3) Rotation matrix from {n} to {b}

    // Controller gains
    Eigen::Matrix<double,6,6> K_P; // (6*6) Proportional gain matrix
    Eigen::Matrix<double,6,6> K_D; // (6*6) Derivative gain matrix
    Eigen::Matrix3d           K_p; // (3*3) Position error gain matrix (part of K_P)
    double c;            //       Attitude error gain

    // Constants
    Eigen::Vector3d r_g; // (3) Center of gravity, expressed in {b}
    Eigen::Vector3d r_b; // (3) Center of buoyancy, expressed in {b}
    double W;            // [N] Weight of ROV
    double B;            // [N] Buoyancy of ROV

    int sgn(double x);
    Eigen::Matrix3d skew(const Eigen::Vector3d &v);
};

#endif
