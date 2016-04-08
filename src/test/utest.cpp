#include "../allocator/lagrange_allocator.h"
#include "geometry_msgs/Wrench.h"
#include "uranus_dp/ThrusterForces.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>

class LagrangeAllocatorTest : public ::testing::Test {
public:
    LagrangeAllocatorTest()
    {
        publisher_  = node_handle_.advertise<geometry_msgs::Wrench>("control_forces", 5);
        subscriber_ = node_handle_.subscribe("control_inputs", 5, &LagrangeAllocatorTest::Callback, this);
        message_received_ = false;
    }

    void SetUp()
    {
        while (!IsNodeReady())
        {
            ros::spinOnce();
        }
    }

    void Publish(double surge, double sway, double heave, double roll, double pitch, double yaw)
    {
        geometry_msgs::Wrench msg;
        msg.force.x = surge  * NORMALIZATION * SCALING_LIN;
        msg.force.y = sway   * NORMALIZATION * SCALING_LIN;
        msg.force.z = heave  * NORMALIZATION * SCALING_LIN;
        msg.torque.x = roll  * NORMALIZATION * SCALING_ANG;
        msg.torque.y = pitch * NORMALIZATION * SCALING_ANG;
        msg.torque.z = yaw   * NORMALIZATION * SCALING_ANG;
        publisher_.publish(msg);
    }

    void WaitForMessage()
    {
        while(!message_received_)
        {
            ros::spinOnce();
        }
    }

    Eigen::Matrix<double,6,1> u_;
    static const double EPSILON        = 0.000000000001;

 private:
    ros::NodeHandle node_handle_;
    ros::Publisher  publisher_;
    ros::Subscriber subscriber_;
    bool message_received_;

    static const double NORMALIZATION  = 0.000030517578125; // Scale inputs down to [-1, 1]
    static const double SCALING_LIN    = 10;                // [N]  Max force in given direction
    static const double SCALING_ANG    = 2;                 // [Nm] Max torque around given axis

    void Callback(const uranus_dp::ThrusterForces& msg)
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

TEST_F(LagrangeAllocatorTest, ExpectedOutput)
{
    Publish(-29051, -27049, 0, 0, 24354, -7259);
    WaitForMessage();
    EXPECT_TRUE(abs(u_(0) - ( 2.237364853185521)) < EPSILON);
    EXPECT_TRUE(abs(u_(1) - (-3.600108488633174)) < EPSILON);
    EXPECT_TRUE(abs(u_(2) - ( 2.729874354234020)) < EPSILON);
    EXPECT_TRUE(abs(u_(3) - (-3.107598987584675)) < EPSILON);
    EXPECT_TRUE(abs(u_(4) - (-3.720447824399290)) < EPSILON);
    EXPECT_TRUE(abs(u_(5) - (-3.915232262733428)) < EPSILON);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "lagrange_allocator_test");
    ROS_INFO("Launching node lagrange_allocator_test.");

    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}