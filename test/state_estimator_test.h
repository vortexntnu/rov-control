#ifndef STATE_ESTIMATOR_TEST_H
#define STATE_ESTIMATOR_TEST_H

#include "ros/ros.h"
#include <gtest/gtest.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include "../src/eigen_typedefs.h"
#include <eigen_conversions/eigen_msg.h>
#include "uranus_dp/ResetStateEstimator.h"

class StateEstimatorTest : public ::testing::Test
{
public:
    StateEstimatorTest();
    void SetUp();
    void Publish(double ax, double ay, double az, double wx, double wy, double wz);
    void WaitForMessage();
    void OneSecondSpin();
    void OneSecondPublish(double ax, double ay, double az, double wx, double wy, double wz);
    void ResetFilter();
    bool GetMessageReceived();

    Eigen::Vector3d    p;
    Eigen::Quaterniond q;
    Eigen::Vector3d    v;
    Eigen::Vector3d    w;

    Eigen::Vector3d    p_0;
    Eigen::Quaterniond q_0;
    Eigen::Vector3d    v_0;
    Eigen::Vector3d    w_0;

    static const double MAX_ERROR = 1e-3;
    static const double STANDARD_GRAVITY = 9.80665;
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