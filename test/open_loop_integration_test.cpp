#include "ros/ros.h"
#include <gtest/gtest.h>
#include "maelstrom_msgs/JoystickMotionCommand.h"
#include "maelstrom_msgs/ThrusterForces.h"
#include "../src/control_mode_enum.h"
#include <Eigen/Dense>

class OpenLoopIntegrationTest : public ::testing::Test
{
public:
    OpenLoopIntegrationTest()
    {
        pub = nh.advertise<maelstrom_msgs::JoystickMotionCommand>("joystick_motion_command", 10);
        sub = nh.subscribe("thruster_forces", 10, &OpenLoopIntegrationTest::Callback, this);
        message_received = false;
    }

    void SetUp()
    {
        while (!IsNodeReady())
            ros::spinOnce();
    }

    void Publish(int forward, int right, int down, int tilt_up, int turn_right)
    {
        maelstrom_msgs::JoystickMotionCommand msg;
        msg.forward    = forward;
        msg.right      = right;
        msg.down       = down;
        msg.tilt_up    = tilt_up;
        msg.turn_right = turn_right;
        msg.control_mode = static_cast<int>(ControlModes::OPEN_LOOP);
        pub.publish(msg);
    }

    void WaitForMessage()
    {
        while (!message_received)
            ros::spinOnce();
    }

    bool HasReceivedMessage()
    {
        return message_received;
    }

    void OneSecondSpin()
    {
        ros::Time start = ros::Time::now();
        while (ros::Time::now() < start + ros::Duration(1))
            ros::spinOnce();
    }

    double F_A;
    double F_B;
    double F_C;
    double F_D;
    double F_E;
    double F_F;

 private:
    ros::NodeHandle nh;
    ros::Publisher  pub;
    ros::Subscriber sub;
    bool message_received;

    void Callback(const maelstrom_msgs::ThrusterForces& msg)
    {
        F_A = msg.F1;
        F_B = msg.F2;
        F_C = msg.F3;
        F_D = msg.F4;
        F_E = msg.F5;
        F_F = msg.F6;
        message_received = true;
    }

    bool IsNodeReady()
    {
        return (pub.getNumSubscribers() > 0) && (sub.getNumPublishers() > 0);
    }
};

TEST_F(OpenLoopIntegrationTest, CheckResponsiveness)
{
    Publish(0,0,0,0,0);
    WaitForMessage();
}

// Command forward motion, assure correct forces for each thruster.
TEST_F(OpenLoopIntegrationTest, Forward)
{
    Publish(1,0,0,0,0);
    WaitForMessage();

    EXPECT_TRUE(F_A < 0);
    EXPECT_TRUE(F_B > 0);
    EXPECT_TRUE(F_C < 0);
    EXPECT_TRUE(F_D > 0);
    EXPECT_TRUE(F_E > 0);
    EXPECT_TRUE(F_F < 0);
}

// Publish message with out of range value, assure no reply.
TEST_F(OpenLoopIntegrationTest, OutOfRange)
{
    Publish(0,0,-2,0,0);
    OneSecondSpin();
    EXPECT_TRUE(!HasReceivedMessage());
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "open_loop_integration_test");

    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
