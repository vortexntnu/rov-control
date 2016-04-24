#include "ros/ros.h"
#include <gtest/gtest.h>
#include "maelstrom_msgs/JoystickMotionCommand.h"
#include "maelstrom_msgs/ThrusterForces.h"
#include "../src/control_mode_enum.h"
#include <Eigen/Dense>

class OpenLoopIntegrationTest : public ::testing::Test {
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
        {
            ros::spinOnce();
        }
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
        {
            ros::spinOnce();
        }
    }

    Eigen::Matrix<double,6,1> u;

 private:
    ros::NodeHandle nh;
    ros::Publisher  pub;
    ros::Subscriber sub;
    bool message_received;

    void Callback(const maelstrom_msgs::ThrusterForces& msg)
    {
        u(0) = msg.F1;
        u(1) = msg.F2;
        u(2) = msg.F3;
        u(3) = msg.F4;
        u(4) = msg.F5;
        u(5) = msg.F6;
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

TEST_F(OpenLoopIntegrationTest, Forward)
{
    Publish(1,0,0,0,0);
    WaitForMessage();

    EXPECT_TRUE(u(0) > 0);
    EXPECT_TRUE(u(1) > 0);
    EXPECT_TRUE(u(2) < 0);
    EXPECT_TRUE(u(3) < 0);
    EXPECT_TRUE(u(4) < 0);
    EXPECT_TRUE(u(5) > 0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "open_loop_integration_test");

    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
