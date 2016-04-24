#include "ros/ros.h"
#include <gtest/gtest.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include "../src/eigen_typedefs.h"
#include <eigen_conversions/eigen_msg.h>
#include <boost/shared_ptr.hpp>

class ImuIntegrationFilterTest : public ::testing::Test {
public:
    ImuIntegrationFilterTest()
    {
        pub = nh.advertise<sensor_msgs::Imu>("sensor_raw", 10);
        sub = nh.subscribe("state_estimate", 10, &ImuIntegrationFilterTest::Callback, this);
    }

    void SetUp()
    {
        while (!IsNodeReady())
            ros::spinOnce();
    }

    void Publish(double ax, double ay, double az,
                 double wx, double wy, double wz)
    {
        sensor_msgs::Imu msg;
        msg.linear_acceleration.x  = ax;
        msg.linear_acceleration.y  = ay;
        msg.linear_acceleration.z  = az;
        msg.angular_velocity.x = wx;
        msg.angular_velocity.y = wy;
        msg.angular_velocity.z = wz;
        pub.publish(msg);
    }

    boost::shared_ptr<const nav_msgs::Odometry> WaitForMessage()
    {
        return ros::topic::waitForMessage<nav_msgs::Odometry>(sub.getTopic(), ros::Duration(1));
    }

    Eigen::Vector3d    p;
    Eigen::Quaterniond q;
    Eigen::Vector3d    v;
    Eigen::Vector3d    w;
    ros::Time reply_time;
    static const double MAX_ERROR = 1e-3;

 private:
    ros::NodeHandle nh;
    ros::Publisher  pub;
    ros::Subscriber sub;

    void Callback(const nav_msgs::Odometry& msg)
    {
        tf::pointMsgToEigen(msg.pose.pose.position, p);
        tf::quaternionMsgToEigen(msg.pose.pose.orientation, q);
        tf::vectorMsgToEigen(msg.twist.twist.linear, v);
        tf::vectorMsgToEigen(msg.twist.twist.angular, w);
        reply_time = ros::Time::now();
    }

    bool IsNodeReady()
    {
        return (pub.getNumSubscribers() > 0) && (sub.getNumPublishers() > 0);
    }
};

TEST_F(ImuIntegrationFilterTest, CheckResponsivenes)
{
    Publish(0,0,0,0,0,0);
    WaitForMessage();
}

TEST_F(ImuIntegrationFilterTest, ZeroInput)
{
    Publish(0,0,0,0,0,0);
    WaitForMessage();
    WaitForMessage();

    EXPECT_NEAR(p(0), 0, MAX_ERROR);
    EXPECT_NEAR(p(1), 0, MAX_ERROR);
    EXPECT_NEAR(p(2), 0, MAX_ERROR);

    EXPECT_NEAR(q.w(), 1, MAX_ERROR);
    EXPECT_NEAR(q.x(), 0, MAX_ERROR);
    EXPECT_NEAR(q.y(), 0, MAX_ERROR);
    EXPECT_NEAR(q.z(), 0, MAX_ERROR);

    EXPECT_NEAR(v(0), 0, MAX_ERROR);
    EXPECT_NEAR(v(1), 0, MAX_ERROR);
    EXPECT_NEAR(v(2), 0, MAX_ERROR);

    EXPECT_NEAR(w(0), 0, MAX_ERROR);
    EXPECT_NEAR(w(1), 0, MAX_ERROR);
    EXPECT_NEAR(w(2), 0, MAX_ERROR);
}

TEST_F(ImuIntegrationFilterTest, OnlyGravity)
{
    Publish(0,0,9.81,0,0,0);
    ros::Duration(1).sleep();

    EXPECT_NEAR(p(0), 0, MAX_ERROR);
    EXPECT_NEAR(p(1), 0, MAX_ERROR);
    EXPECT_NEAR(p(2), 0, MAX_ERROR);
    EXPECT_NEAR(v(0), 0, MAX_ERROR);
    EXPECT_NEAR(v(1), 0, MAX_ERROR);
    EXPECT_NEAR(v(2), 0, MAX_ERROR);
}

// TEST_F(ImuIntegrationFilterTest, FreeFall)
// {
//     Publish(0,0,0,0,0,0);
//     ros::Duration(1).sleep();
//     message_received = false;
//     WaitForMessage();

//     EXPECT_NEAR(p(0), 0,      MAX_ERROR);
//     EXPECT_NEAR(p(1), 0,      MAX_ERROR);
//     EXPECT_NEAR(p(2), -4.905, MAX_ERROR);
//     EXPECT_NEAR(v(0), 0,      MAX_ERROR);
//     EXPECT_NEAR(v(1), 0,      MAX_ERROR);
//     EXPECT_NEAR(v(2), -9.81 , MAX_ERROR);
// }

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "imuintegrationfilter_test");

    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
