#ifndef QUATERNION_PD_CONTROLLER_H
#define QUATERNION_PD_CONTROLLER_H

#include "ros/ros.h"
#include "uranus_dp/State.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include "../eigen_typedefs.h"

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
    ros::Publisher  controlPub;
    bool enabled;

    void updateProportionalGainMatrix();
    void updateErrorVector();
    void updateRestoringForceVector();

    // State
    Eigen::Vector3d    p;  // Position
    Eigen::Quaterniond q;  // Orientation
    Eigen::Vector6d    nu; // Velocity (linear and angular)
    
    // Setpoints
    Eigen::Vector3d    p_d; // Desired position
    Eigen::Quaterniond q_d; // Desired attitude
    
    // Error variables
    Eigen::Vector6d z; // (6) Pose error vector
    
    // Output
    Eigen::Vector6d tau; // (6) Control forces

    // Other stuff
    Eigen::Vector6d g; // (6)   Restoring force vector
    Eigen::Matrix3d R; // (3*3) Rotation matrix from {n} to {b}

    // Controller gains
    Eigen::Matrix6d K_P; // (6*6) Proportional gain matrix
    Eigen::Matrix6d K_D; // (6*6) Derivative gain matrix
    Eigen::Matrix3d K_p; // (3*3) Position error gain matrix (part of K_P)
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
