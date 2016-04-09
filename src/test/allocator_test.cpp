#include "ros/ros.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "geometry_msgs/Wrench.h"
#include "uranus_dp/ThrusterForces.h"

class AllocatorTest : public ::testing::Test {
public:
    AllocatorTest()
    {
        publisher_  = node_handle_.advertise<geometry_msgs::Wrench>("control_forces", 5);
        subscriber_ = node_handle_.subscribe("control_inputs", 5, &AllocatorTest::Callback, this);
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
        while (!message_received_)
        {
            ros::spinOnce();
        }
    }

    Eigen::Matrix<double,6,1> u_;
    static const double EPSILON = 1e-6; // Max absolute error 1 micronewton

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

TEST_F(AllocatorTest, CheckResponsiveness)
{
    Publish(1, 2, 3, 4, 5, 5);
    WaitForMessage();
    ASSERT_TRUE(true);
}

TEST_F(AllocatorTest, NormalInput)
{
    Publish(-29051, -27049, 0, 0, 24354, -7259);
    WaitForMessage();
    EXPECT_NEAR(u_(0),  2.237187310000600, EPSILON);
    EXPECT_NEAR(u_(1), -3.599822806695235, EPSILON);
    EXPECT_NEAR(u_(2),  2.729657728596679, EPSILON);
    EXPECT_NEAR(u_(3), -3.107352388099156, EPSILON);
    EXPECT_NEAR(u_(4), -3.720152593089502, EPSILON);
    EXPECT_NEAR(u_(5), -3.914921574557270, EPSILON);
}

TEST_F(AllocatorTest, MinimumInput)
{
    Publish(-32768, -32768, 0, 0, -32768, -32768);
    WaitForMessage();
    EXPECT_NEAR(u_(0), -2.328576868278356, EPSILON);
    EXPECT_NEAR(u_(1), -9.399712492659631, EPSILON);
    EXPECT_NEAR(u_(2), -0.105506104184636, EPSILON);
    EXPECT_NEAR(u_(3), -7.176641728565912, EPSILON);
    EXPECT_NEAR(u_(4),  2.160745386314036, EPSILON);
    EXPECT_NEAR(u_(5),  1.281534753343126, EPSILON);
}

TEST_F(AllocatorTest, MaximumInput)
{
    Publish(32767, 32767, 0, 0, 32767, 32767);
    WaitForMessage();
    EXPECT_NEAR(u_(0),  2.328505805751858, EPSILON);
    EXPECT_NEAR(u_(1),  9.399425636199283, EPSILON);
    EXPECT_NEAR(u_(2),  0.105502884393859, EPSILON);
    EXPECT_NEAR(u_(3),  7.176422714841284, EPSILON);
    EXPECT_NEAR(u_(4), -2.160679445597902, EPSILON);
    EXPECT_NEAR(u_(5), -1.281495644006171, EPSILON);
}

TEST_F(AllocatorTest, ZeroInput)
{
    Publish(0, 0, 0, 0, 0, 0);
    WaitForMessage();
    EXPECT_NEAR(u_(0), 0, EPSILON);
    EXPECT_NEAR(u_(1), 0, EPSILON);
    EXPECT_NEAR(u_(2), 0, EPSILON);
    EXPECT_NEAR(u_(3), 0, EPSILON);
    EXPECT_NEAR(u_(4), 0, EPSILON);
    EXPECT_NEAR(u_(5), 0, EPSILON);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "allocator_test");
    ROS_INFO("Launching node allocator_test.");

    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
