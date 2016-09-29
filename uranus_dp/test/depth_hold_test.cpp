#include "ros/ros.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include "geometry_msgs/Wrench.h"
#include "sensor_msgs/FluidPressure.h"

#include "../src/eigen_typedefs.h"
#include "../src/control_mode_enum.h"
#include "uranus_dp/SetControlMode.h"

class DepthHoldTest : public ::testing::Test
{
public:
    DepthHoldTest()
    {
        pressurePub = nh.advertise<sensor_msgs::FluidPressure>("imu/pressure", 10);
        setpointPub = nh.advertise<sensor_msgs::FluidPressure>("pressure_setpoint", 10);
        openloopPub = nh.advertise<geometry_msgs::Wrench>("wrench_setpoints", 10);
        controlSub  = nh.subscribe("rov_forces", 10, &DepthHoldTest::Callback, this);
        client      = nh.serviceClient<uranus_dp::SetControlMode>("set_control_mode");

        message_received = false;
    }

    void SetUp()
    {
        while (!IsNodeReady())
            ros::spinOnce();
    }

    void Publish(double depth_setpoint, double depth_reading)
    {
        sensor_msgs::FluidPressure setpoint_msg;
        sensor_msgs::FluidPressure reading_msg;
        setpoint_msg.fluid_pressure = depth_setpoint * 1000 * 9.80665;
        reading_msg.fluid_pressure = depth_reading * 1000 * 9.80665;
        setpointPub.publish(setpoint_msg);
        pressurePub.publish(reading_msg);
    }

    void WaitForMessage()
    {
        message_received = false;
        while (!message_received)
            ros::spinOnce();
    }

    void SetModeToDepthHold()
    {
        uranus_dp::SetControlMode srv;
        srv.request.mode = ControlModes::DEPTH_HOLD;
        client.call(srv);
    }

    ros::ServiceClient client;
    Eigen::Vector6d tau;

private:
    ros::NodeHandle nh;
    ros::Publisher pressurePub;
    ros::Publisher setpointPub;
    ros::Publisher openloopPub;
    ros::Subscriber controlSub;
    bool message_received;

    void Callback(const geometry_msgs::Wrench& msg)
    {
        tf::wrenchMsgToEigen(msg, tau);
        message_received = true;
    }

    bool IsNodeReady()
    {
        return (pressurePub.getNumSubscribers() > 0)
            && (setpointPub.getNumSubscribers() > 0)
            && (openloopPub.getNumSubscribers() > 0)
            && (controlSub.getNumPublishers() > 0);
    }
};

TEST_F(DepthHoldTest, CheckResponsiveness)
{
    SetModeToDepthHold();
    Publish(0,0);
    WaitForMessage();
}

TEST_F(DepthHoldTest, GoDownWhenShouldGoDown)
{
    SetModeToDepthHold();
    Publish(1,0);
    ros::Duration(0.1).sleep();
    WaitForMessage();

    EXPECT_TRUE(tau(2) > 0);
}

TEST_F(DepthHoldTest, GoUpWhenShouldGoUp)
{
    SetModeToDepthHold();
    Publish(0,1);
    ros::Duration(0.1).sleep();
    WaitForMessage();

    EXPECT_TRUE(tau(2) < 0);
}

TEST_F(DepthHoldTest, CorrectDownForce)
{
    SetModeToDepthHold();
    Publish(1,0);
    ros::Duration(0.1).sleep();
    WaitForMessage();

    EXPECT_NEAR(tau(2), 9.80665, 1e-3);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "depth_hold_test");
    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}