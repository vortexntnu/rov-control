#include "ros/ros.h"
#include <gtest/gtest.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include "../src/eigen_typedefs.h"
#include <eigen_conversions/eigen_msg.h>
#include "uranus_dp/ResetIntegrationFilter.h"

class IntegrationFilterTest : public ::testing::Test {
public:
    IntegrationFilterTest()
    {
        pub = nh.advertise<sensor_msgs::Imu>("sensor_raw", 10);
        sub = nh.subscribe("state_estimate", 10, &IntegrationFilterTest::Callback, this);
        client = nh.serviceClient<uranus_dp::ResetIntegrationFilter>("reset_integration_filter");
        message_received = false;
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

    void WaitForMessage()
    {
        message_received = false;
        while (!message_received)
            ros::spinOnce();
    }

    void OneSecondSpin()
    {
        ros::Time start = ros::Time::now();
        while (ros::Time::now() < start + ros::Duration(1))
            ros::spinOnce();
    }

    void ResetFilter()
    {
        uranus_dp::ResetIntegrationFilter srv;
        client.call(srv);
    }

    Eigen::Vector3d    p;
    Eigen::Quaterniond q;
    Eigen::Vector3d    v;
    Eigen::Vector3d    w;

    static const double MAX_ERROR = 1e-3;
    static const double STANDARD_GRAVITY = 9.80665;
    bool message_received;
    ros::ServiceClient client;

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
        message_received = true;
    }

    bool IsNodeReady()
    {
        return (pub.getNumSubscribers() > 0) && (sub.getNumPublishers() > 0);
    }
};

TEST_F(IntegrationFilterTest, CheckResponsivenes)
{
    WaitForMessage();
}

TEST_F(IntegrationFilterTest, CorrectInitialization)
{
    ResetFilter();
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

TEST_F(IntegrationFilterTest, OnlyGravity)
{
    ResetFilter();
    Publish(0,0,STANDARD_GRAVITY,0,0,0);
    OneSecondSpin();

    EXPECT_NEAR(p(0), 0, MAX_ERROR);
    EXPECT_NEAR(p(1), 0, MAX_ERROR);
    EXPECT_NEAR(p(2), 0, MAX_ERROR);
    EXPECT_NEAR(v(0), 0, MAX_ERROR);
    EXPECT_NEAR(v(1), 0, MAX_ERROR);
    EXPECT_NEAR(v(2), 0, MAX_ERROR);
}

TEST_F(IntegrationFilterTest, ForwardAcceleration)
{
    ResetFilter();
    Publish(1,0,STANDARD_GRAVITY,0,0,0);
    OneSecondSpin();

    EXPECT_NEAR(p(0), 0.5, 0.01);
    EXPECT_NEAR(p(1), 0, 0.01);
    EXPECT_NEAR(p(2), 0, 0.01);
    EXPECT_NEAR(v(0), 1, 0.01);
    EXPECT_NEAR(v(1), 0, 0.01);
    EXPECT_NEAR(v(2), 0, 0.01);
}

TEST_F(IntegrationFilterTest, Rotate)
{
    ResetFilter();
    Publish(0,0,STANDARD_GRAVITY,0,0,1); // 1 rad/sec in yaw  (I think)
    OneSecondSpin();

    EXPECT_NEAR(p(0), 0, 0.01);
    EXPECT_NEAR(p(1), 0, 0.01);
    EXPECT_NEAR(p(2), 0, 0.01);

    EXPECT_NEAR(q.w(), 0.877582561890373, 0.01);
    EXPECT_NEAR(q.x(), 0, 0.01);
    EXPECT_NEAR(q.y(), 0, 0.01);
    EXPECT_NEAR(q.z(), 0.479425538604203, 0.01);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "integration_filter_test");

    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
