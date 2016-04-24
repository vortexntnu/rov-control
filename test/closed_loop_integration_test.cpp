#include "ros/ros.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "maelstrom_msgs/JoystickMotionCommand.h"
#include "maelstrom_msgs/ThrusterForces.h"
#include "../src/control_mode_enum.h"
#include "../src/eigen_typedefs.h"

class ClosedLoopIntegrationTest : public ::testing::Test
{
public:
    ClosedLoopIntegrationTest()
    {
        pub = nh.advertise<maelstrom_msgs::JoystickMotionCommand>("joystick_motion_command", 10);
        sub = nh.subscribe("thruster_forces", 10, &ClosedLoopIntegrationTest::Callback, this);
        message_received = false;
    }

    void SetUp()
    {
        while (!IsNodeReady())
            ros::spinOnce();
    }

    void Publish(int forward, int right, int down, int tilt_up, int turn_right, ControlMode control_mode)
    {
        maelstrom_msgs::JoystickMotionCommand msg;
        msg.forward    = forward;
        msg.right      = right;
        msg.down       = down;
        msg.tilt_up    = tilt_up;
        msg.turn_right = turn_right;
        msg.control_mode = static_cast<int>(control_mode);
        pub.publish(msg);
    }

    void WaitForMessage()
    {
        message_received = false;
        while (!message_received)
            ros::spinOnce();
    }

    Eigen::Vector6d u;

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

TEST_F(ClosedLoopIntegrationTest, CheckResponsiveness)
{
    ros::Duration(0.5).sleep();
    Publish(0,0,0,0,0,ControlModes::POSITION_HOLD);
    WaitForMessage();
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "closed_loop_integration_test");

    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
