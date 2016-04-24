#include "ros/ros.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "geometry_msgs/Wrench.h"
#include "maelstrom_msgs/ThrusterForces.h"
#include <eigen_conversions/eigen_msg.h>
#include "../src/eigen_typedefs.h"

class AllocatorTest : public ::testing::Test {
public:
    AllocatorTest()
    {
        pub = nh.advertise<geometry_msgs::Wrench>("rov_forces", 10);
        sub = nh.subscribe("thruster_forces", 10, &AllocatorTest::Callback, this);
        message_received = false;
    }

    void SetUp()
    {
        while (!IsNodeReady())
            ros::spinOnce();
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
        pub.publish(msg);
    }

    void WaitForMessage()
    {
        while (!message_received)
            ros::spinOnce();
    }

    Eigen::Vector6d u;
    static const double MAX_ERROR = 1e-6; // Max absolute error 1 micronewton

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

TEST_F(AllocatorTest, CheckResponsiveness)
{
    Publish(0, 0, 0, 0, 0, 0);
    WaitForMessage();
}

TEST_F(AllocatorTest, ZeroInput)
{
    Publish(0, 0, 0, 0, 0, 0);
    WaitForMessage();

    EXPECT_NEAR(u(0), 0, MAX_ERROR);
    EXPECT_NEAR(u(1), 0, MAX_ERROR);
    EXPECT_NEAR(u(2), 0, MAX_ERROR);
    EXPECT_NEAR(u(3), 0, MAX_ERROR);
    EXPECT_NEAR(u(4), 0, MAX_ERROR);
    EXPECT_NEAR(u(5), 0, MAX_ERROR);
}

TEST_F(AllocatorTest, Forward)
{
    Publish(1, 0, 0, 0, 0, 0);
    WaitForMessage();

    EXPECT_TRUE(u(0) > 0);
    EXPECT_TRUE(u(1) > 0);
    EXPECT_TRUE(u(2) < 0);
    EXPECT_TRUE(u(3) < 0);
    EXPECT_TRUE(u(4) > 0);
    EXPECT_TRUE(u(5) < 0);
}

TEST_F(AllocatorTest, Sideways)
{
    Publish(0, 1, 0, 0, 0, 0);
    WaitForMessage();

    EXPECT_TRUE(u(0) > 0);
    EXPECT_TRUE(u(1) < 0);
    EXPECT_TRUE(u(2) < 0);
    EXPECT_TRUE(u(3) > 0);
    EXPECT_NEAR(u(4), 0, MAX_ERROR);
    EXPECT_NEAR(u(5), 0, MAX_ERROR);
}

TEST_F(AllocatorTest, Downward)
{
    Publish(0, 0, 1, 0, 0, 0);
    WaitForMessage();

    EXPECT_NEAR(u(0), 0, MAX_ERROR);
    EXPECT_NEAR(u(1), 0, MAX_ERROR);
    EXPECT_NEAR(u(2), 0, MAX_ERROR);
    EXPECT_NEAR(u(3), 0, MAX_ERROR);
    EXPECT_TRUE(u(4) > 0); // Positive force 5 pushes front down
    EXPECT_TRUE(u(5) > 0); // Positive force 6 pushes rear down
}

TEST_F(AllocatorTest, TiltUp)
{
    Publish(0, 0, 0, 0, 1, 0);
    WaitForMessage();

    EXPECT_NEAR(u(0), 0, MAX_ERROR);
    EXPECT_NEAR(u(1), 0, MAX_ERROR);
    EXPECT_NEAR(u(2), 0, MAX_ERROR);
    EXPECT_NEAR(u(3), 0, MAX_ERROR);
    EXPECT_TRUE(u(4) < 0); // Negative force 5 pushes front up
    EXPECT_TRUE(u(5) > 0); // Positive force 6 pushes rear down
}

TEST_F(AllocatorTest, TurnRight)
{
    Publish(0, 0, 0, 0, 0, 1);
    WaitForMessage();

    EXPECT_TRUE(u(0) > 0);
    EXPECT_TRUE(u(1) < 0);
    EXPECT_TRUE(u(2) > 0);
    EXPECT_TRUE(u(3) < 0);
    EXPECT_NEAR(u(4), 0, MAX_ERROR);
    EXPECT_NEAR(u(5), 0, MAX_ERROR);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "allocator_test");

    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
