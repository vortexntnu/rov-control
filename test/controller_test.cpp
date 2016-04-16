#include "ros/ros.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "uranus_dp/State.h"
#include "uranus_dp/SetControlMode.h"
#include "../src/control_mode_enum.h"

class OpenLoopControllerTest : public ::testing::Test
{
public:
    OpenLoopControllerTest()
    {
        pub    = nh.advertise<geometry_msgs::Wrench>("wrench_setpoints", 1);
        sub    = nh.subscribe("rov_forces", 5, &OpenLoopControllerTest::Callback, this);
        client = nh.serviceClient<uranus_dp::SetControlMode>("set_control_mode");

        message_received = false;
    }

    void SetUp()
    {
        while (!IsNodeReady())
        {
            ros::spinOnce();
        }
    }

    void PublishSetpoint(double X, double Y, double Z, double K, double M, double N)
    {
        geometry_msgs::Wrench msg;
        msg.force.x  = X;
        msg.force.y  = Y;
        msg.force.z  = Z;
        msg.torque.x = K;
        msg.torque.y = M;
        msg.torque.z = N;
        pub.publish(msg);
    }

    void WaitForMessage()
    {
        while (!message_received)
        {
            ros::spinOnce();
        }
    }

    ros::ServiceClient client; // Should be private?
    Eigen::Matrix<double,6,1> tau;
    static const double EPSILON = 1e-6; // Max absolute error

 private:
    ros::NodeHandle nh;
    ros::Publisher  pub;
    ros::Subscriber sub;

    bool message_received;

    void Callback(const geometry_msgs::Wrench& msg)
    {
        tf::wrenchMsgToEigen(msg, tau);
        message_received = true;
    }

    bool IsNodeReady()
    {
        return (pub.getNumSubscribers() > 0) && (sub.getNumPublishers() > 0);
    }
};

class QuaternionPdControllerTest : public ::testing::Test
{
public:
    QuaternionPdControllerTest()
    {
        setpointPub = nh.advertise<geometry_msgs::Pose>("pose_setpoints", 10);
        statePub    = nh.advertise<uranus_dp::State>("state_estimate", 10);
        sub         = nh.subscribe("rov_forces", 10, &QuaternionPdControllerTest::Callback, this);
        client      = nh.serviceClient<uranus_dp::SetControlMode>("set_control_mode");

        message_received = false;
    }

    void SetUp()
    {
        while (!IsNodeReady())
        {
            ros::spinOnce();
        }
    }

    void PublishSetpoint(Eigen::Vector3d p, Eigen::Quaterniond q)
    {
        geometry_msgs::Pose msg;
        tf::pointEigenToMsg(p, msg.position);
        tf::quaternionEigenToMsg(q, msg.orientation);
        setpointPub.publish(msg);
    }

    void PublishState(Eigen::Vector3d p, Eigen::Quaterniond q, Eigen::Vector3d v, Eigen::Vector3d omega)
    {
        uranus_dp::State msg;
        tf::pointEigenToMsg(p, msg.pose.position);
        tf::quaternionEigenToMsg(q, msg.pose.orientation);
        tf::vectorEigenToMsg(v, msg.twist.linear);
        tf::vectorEigenToMsg(omega, msg.twist.angular);
        statePub.publish(msg);
    }

    void WaitForMessage()
    {
        while (!message_received)
        {
            ros::spinOnce();
        }
    }

    ros::ServiceClient client; // Should be private?
    Eigen::Matrix<double,6,1> tau;
    static const double EPSILON = 1e-6; // Max absolute error

 private:
    ros::NodeHandle nh;
    ros::Publisher  setpointPub;
    ros::Publisher statePub;
    ros::Subscriber sub;

    bool message_received;

    void Callback(const geometry_msgs::Wrench& msg)
    {
        tf::wrenchMsgToEigen(msg, tau);
        message_received = true;
    }

    bool IsNodeReady()
    {
        return (setpointPub.getNumSubscribers() > 0) && (sub.getNumPublishers() > 0);
    }
};

 /*
  * Make sure we receive any messages at all.
  */
TEST_F(OpenLoopControllerTest, CheckResponsiveness)
{
    ros::Duration(0.5).sleep();

    uranus_dp::SetControlMode srv;
    srv.request.mode = ControlModes::OPEN_LOOP;
    if (!client.call(srv))
        ROS_ERROR("Failed to call service set_control_mode. New mode OPEN_LOOP not set.");

    PublishSetpoint(0,0,0,0,0,0);
    WaitForMessage();
    EXPECT_TRUE(true);
}

 /*
  * Make sure open loop outputs the same as the input.
  */
TEST_F(OpenLoopControllerTest, DirectFeedthrough)
{
    uranus_dp::SetControlMode srv;
    srv.request.mode = ControlModes::OPEN_LOOP;
    if (!client.call(srv))
        ROS_ERROR("Failed to call service set_control_mode. New mode OPEN_LOOP not set.");

    PublishSetpoint(-9.966, -7.907, 7.626, -6.023, 1.506, 2.805);
    WaitForMessage();
    EXPECT_NEAR(tau(0), -9.966, EPSILON);
    EXPECT_NEAR(tau(1), -7.907, EPSILON);
    EXPECT_NEAR(tau(2),  7.626, EPSILON);
    EXPECT_NEAR(tau(3), -6.023, EPSILON);
    EXPECT_NEAR(tau(4),  1.506, EPSILON);
    EXPECT_NEAR(tau(5),  2.805, EPSILON);
}

 /*
  * Input zero speed and nonzero pose. Input same pose state and pose setpoint. Expect zero output.
  */
TEST_F(QuaternionPdControllerTest, ZeroErrorZeroOutput)
{
    uranus_dp::SetControlMode srv;
    srv.request.mode = ControlModes::STATIONKEEPING;
    if (!client.call(srv))
        ROS_ERROR("Failed to call service set_control_mode. New mode STATIONKEEPING not set.");

    Eigen::Vector3d    p(1,2,3);
    Eigen::Quaterniond q(4,5,6,7);
    q.normalize();

    Eigen::Vector3d v(0,0,0);
    Eigen::Vector3d omega(0,0,0);

    PublishState(p, q, v, omega);
    PublishSetpoint(p, q);

    WaitForMessage();

    EXPECT_NEAR(tau(0), 0, EPSILON);
    EXPECT_NEAR(tau(1), 0, EPSILON);
    EXPECT_NEAR(tau(2), 0, EPSILON);
    EXPECT_NEAR(tau(3), 0, EPSILON);
    EXPECT_NEAR(tau(4), 0, EPSILON);
    EXPECT_NEAR(tau(5), 0, EPSILON);
}

 /*
  * Give error only in surge, assure rov force command only in surge.
  */
TEST_F(QuaternionPdControllerTest, OnlySurge)
{
    uranus_dp::SetControlMode srv;
    srv.request.mode = ControlModes::STATIONKEEPING;
    if (!client.call(srv))
        ROS_ERROR("Failed to call service set_control_mode. New mode STATIONKEEPING not set.");

    Eigen::Vector3d    p(0,0,0);
    Eigen::Quaterniond q(1,0,0,0);
    q.normalize();
    Eigen::Vector3d    v(0,0,0);
    Eigen::Vector3d    omega(0,0,0);

    Eigen::Vector3d p_sp(1,0,0);

    PublishState(p, q, v, omega);
    PublishSetpoint(p_sp, q);

    ros::Duration(0.1).sleep(); // Controller runs at 10 Hz, must give it time to compute new value.
    WaitForMessage();

    EXPECT_NEAR(tau(0), 30, EPSILON);
    EXPECT_NEAR(tau(1),  0, EPSILON);
    EXPECT_NEAR(tau(2),  0, EPSILON);
    EXPECT_NEAR(tau(3),  0, EPSILON);
    EXPECT_NEAR(tau(4),  0, EPSILON);
    EXPECT_NEAR(tau(5),  0, EPSILON);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "controller_test");
    ROS_INFO("Launching node controller_test.");

    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
