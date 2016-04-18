#include "ros/ros.h"
#include <gtest/gtest.h>
#include "maelstrom_msgs/DirectionalInput.h"
#include "maelstrom_msgs/ThrusterForces.h"
#include "../src/control_mode_enum.h"
#include <Eigen/Dense>

class OpenLoopIntegrationTest : public ::testing::Test {
public:
    OpenLoopIntegrationTest()
    {
        publisher_  = node_handle_.advertise<maelstrom_msgs::DirectionalInput>("joy_input", 10);
        subscriber_ = node_handle_.subscribe("thruster_forces", 10, &OpenLoopIntegrationTest::Callback, this);
        message_received_ = false;
    }

    void SetUp()
    {
        while (!IsNodeReady())
        {
            ros::spinOnce();
        }
    }

    void Publish(int strafe_X, int strafe_Y, int turn_X, int turn_Y, int ascend)
    {
        maelstrom_msgs::DirectionalInput msg;
        msg.strafe_X = strafe_X;
        msg.strafe_Y = strafe_Y;
        msg.turn_X = turn_X;
        msg.turn_Y = turn_Y;
        msg.ascend = ascend;
        msg.control_mode = static_cast<int>(control_mode_);
        publisher_.publish(msg);
    }

    void WaitForMessage()
    {
        while (!message_received_)
        {
            ros::spinOnce();
        }
    }

    Eigen::Matrix<double,6,1> u_;
    static const ControlMode control_mode_ = ControlModes::OPEN_LOOP;
    // static const double EPSILON = 1e-6; // Max absolute error 1 micronewton

 private:
    ros::NodeHandle node_handle_;
    ros::Publisher  publisher_;
    ros::Subscriber subscriber_;
    bool message_received_;

    void Callback(const maelstrom_msgs::ThrusterForces& msg)
    {
        u_(0) = msg.F1;
        u_(1) = msg.F2;
        u_(2) = msg.F3;
        u_(3) = msg.F4;
        u_(4) = msg.F5;
        u_(5) = msg.F6;

        message_received_ = true;
    }

    bool IsNodeReady()
    {
        return (publisher_.getNumSubscribers() > 0) && (subscriber_.getNumPublishers() > 0);
    }
};

TEST_F(OpenLoopIntegrationTest, CheckResponsiveness)
{
    ros::Duration(0.5).sleep();

    Publish(1,2,3,4,5);
    WaitForMessage();
    EXPECT_TRUE(true);
}

 /*
  * Command maximum forward thrust. Check this gives 10 N forward.
  */
TEST_F(OpenLoopIntegrationTest, MaxForward)
{
    Publish(32767,0,0,0,0);
    WaitForMessage();
    double totalForwardThrust = 0.7071*(u_(0) + u_(1) + u_(2) + u_(3)) + u_(4) + u_(5);
    EXPECT_NEAR(totalForwardThrust, 10, 1e-3);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "open_loop_integration_test");

    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
