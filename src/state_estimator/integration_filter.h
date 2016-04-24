#ifndef INTEGRATION_FILTER_H
#define INTEGRATION_FILTER_H

#include "ros/ros.h"
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "uranus_dp/ResetIntegrationFilter.h"
#include "../eigen_typedefs.h"

class IntegrationFilter
{
public:
    IntegrationFilter(double frequency);
    bool reset(uranus_dp::ResetIntegrationFilter::Request &req, uranus_dp::ResetIntegrationFilter::Response &resp);
    void callback(const sensor_msgs::Imu& msg);
    void update();
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher  pub;

    // Time step
    double dt;

    // Measurements
    Eigen::Vector3d a_imu; // Accelerometer
    Eigen::Vector3d w_imu; // Gyro

    // State estimates
    Eigen::Vector3d    p;
    Eigen::Quaterniond q;
    Eigen::Vector3d    v;
    Eigen::Vector3d    w;

    // Intermediate variables
    Eigen::Vector3d v_dot;
    Eigen::Vector4d q_dot;

    // Gravity vector in NED frame
    Eigen::Vector3d g_n;

    Eigen::Matrix<double,4,3> Tmatrix();

    Eigen::Matrix3d skew(const Eigen::Vector3d &v);
};

#endif