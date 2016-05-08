#ifndef STATE_ESTIMATOR_TEST_H
#define STATE_ESTIMATOR_TEST_H

#include "ros/ros.h"
#include <gtest/gtest.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include "../src/eigen_typedefs.h"
#include <eigen_conversions/eigen_msg.h>
#include "uranus_dp/ResetIntegrationFilter.h"

class IntegrationFilterTest : public ::testing::Test
{
public:
    IntegrationFilterTest();
    void SetUp();
    void Publish(double ax, double ay, double az, double wx, double wy, double wz, double qx, double qy, double qz, double qw);
    void OneSecondPublish(double ax, double ay, double az, double wx, double wy, double wz, double qx, double qy, double qz, double qw);
    void WaitForMessage();
    void ResetFilter();

    Eigen::Vector3d    p;
    Eigen::Quaterniond q;
    Eigen::Vector3d    v;
    Eigen::Vector3d    w;

    static const double MAX_ERROR = 0.01;
    bool message_received;
    ros::ServiceClient client;

 private:
    ros::NodeHandle nh;
    ros::Publisher  pub;
    ros::Subscriber sub;

    void Callback(const nav_msgs::Odometry& msg);
    bool IsNodeReady();
};

#endif
