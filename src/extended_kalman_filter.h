#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include "ros/ros.h"
#include <Eigen/Dense>
#include "uranus_dp/State.h"
#include "geometry_msgs/Wrench.h"

class ExtendedKalmanFilter
{
public:
    ExtendedKalmanFilter(double sampleTime);
    void controlCallback(const geometry_msgs::Wrench &uMsg);
    // void sensorCallback(const uranus_dp::Sensor &yMsg);
    void update();
private:
    ros::NodeHandle nh;
    ros::Subscriber controlSub;
    ros::Subscriber sensorSub;
    ros::Publisher  statePub;

    // Sampling time
    double h;

    // System variables
    Eigen::Matrix<double,13, 1> x; // State
    Eigen::Matrix<double,13, 1> f; // System dynamics
    Eigen::Matrix<double,13, 6> B; // Input dynamics
    Eigen::Matrix<double, 6, 1> u; // Input
    Eigen::Matrix<double,13,13> E; // System noise dynamics
    Eigen::Matrix<double,13, 1> w; // System noise
    Eigen::Matrix<double,10, 1> y; // Measurements
    Eigen::Matrix<double,10,13> H; // Sensor dynamics
    Eigen::Matrix<double,10, 1> v; // Sensor noise

    // Filter variables
    Eigen::Matrix<double,13,13> Q;     // Design matrix
    Eigen::Matrix<double,10,10> R;     // Design matrix
    Eigen::Matrix<double,13,10> K;     // Kalman gain
    Eigen::Matrix<double,13, 1> x_hat; // State estimate
    Eigen::Matrix<double,13, 1> x_bar; // State projection (or something)
    Eigen::Matrix<double,13,13> P_hat; // Covariance estimate
    Eigen::Matrix<double,13,13> P_bar; // Covariance projection (or something)
    Eigen::Matrix<double,13,13> Phi;   // 
    Eigen::Matrix<double,13,13> Gamma; // 

    // Helpful extra shit
    Eigen::Matrix<double,13,13> I;     // 13*13 identity
    Eigen::Matrix<double,13,10> H_transpose;
    Eigen::Matrix<double,7,1> eta; // Pose (first seven elements of state)
    Eigen::Matrix<double,6,1> nu;  // Velocity (last six elements of state)

    void updateSystemDynamics();
    void updatePhi();
};

#endif
