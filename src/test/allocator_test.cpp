#include "ros/ros.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "geometry_msgs/Wrench.h"
#include "uranus_dp/ThrusterForces.h"
#include <eigen_conversions/eigen_msg.h>

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
        msg.force.x  = surge;
        msg.force.y  = sway;
        msg.force.z  = heave;
        msg.torque.x = roll;
        msg.torque.y = pitch;
        msg.torque.z = yaw;
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
    Publish(1,2,3,4,5,6);
    WaitForMessage();
    EXPECT_TRUE(true);
}

TEST_F(AllocatorTest, NormalInput)
{
    Publish(3.286, -7.510, 0, 0, -1.272, 0.196);
    WaitForMessage();

    EXPECT_NEAR(u_(0),  2.089572758999718, EPSILON);
    EXPECT_NEAR(u_(1), -3.220850094910619, EPSILON);
    EXPECT_NEAR(u_(2),  1.871711824118534, EPSILON);
    EXPECT_NEAR(u_(3), -3.438711029791804, EPSILON);
    EXPECT_NEAR(u_(4),  2.553894350261510, EPSILON);
    EXPECT_NEAR(u_(5),  2.640056992292659, EPSILON);
}

TEST_F(AllocatorTest, MinimumInput)
{
    Publish(-10, -10, 0, 0, -2, -2);
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
    Publish(10, 10, 0, 0, 2, 2);
    WaitForMessage();

    EXPECT_NEAR(u_(0),  2.328576868278356, EPSILON);
    EXPECT_NEAR(u_(1),  9.399712492659631, EPSILON);
    EXPECT_NEAR(u_(2),  0.105506104184636, EPSILON);
    EXPECT_NEAR(u_(3),  7.17664172856591, EPSILON);
    EXPECT_NEAR(u_(4), -2.160745386314036, EPSILON);
    EXPECT_NEAR(u_(5), -1.281534753343126, EPSILON);
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
