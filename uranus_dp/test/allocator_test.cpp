#include "ros/ros.h"
#include <gtest/gtest.h>
#include "geometry_msgs/Wrench.h"
#include "maelstrom_msgs/ThrusterForces.h"

class AllocatorTest : public ::testing::Test
{
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

    double F1;
    double F2;
    double F3;
    double F4;
    double F5;
    double F6;

    static const double MAX_ERROR = 1e-6; // Max absolute error 1 micronewton

 private:
    ros::NodeHandle nh;
    ros::Publisher  pub;
    ros::Subscriber sub;
    bool message_received;

    void Callback(const maelstrom_msgs::ThrusterForces& msg)
    {
        F1 = msg.F1;
        F2 = msg.F2;
        F3 = msg.F3;
        F4 = msg.F4;
        F5 = msg.F5;
        F6 = msg.F6;
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

    EXPECT_NEAR(F1, 0, MAX_ERROR);
    EXPECT_NEAR(F2, 0, MAX_ERROR);
    EXPECT_NEAR(F3, 0, MAX_ERROR);
    EXPECT_NEAR(F4, 0, MAX_ERROR);
    EXPECT_NEAR(F5, 0, MAX_ERROR);
    EXPECT_NEAR(F6, 0, MAX_ERROR);
}

TEST_F(AllocatorTest, Forward)
{
    Publish(1, 0, 0, 0, 0, 0);
    WaitForMessage();

    EXPECT_TRUE(F1 < 0);
    EXPECT_TRUE(F2 > 0);
    EXPECT_TRUE(F3 < 0);
    EXPECT_TRUE(F4 > 0);
    EXPECT_TRUE(F5 > 0);
    EXPECT_TRUE(F6 < 0);
}

TEST_F(AllocatorTest, Sideways)
{
    Publish(0, 1, 0, 0, 0, 0);
    WaitForMessage();

    EXPECT_NEAR(F1, 0, MAX_ERROR);
    EXPECT_TRUE(F2 < 0);
    EXPECT_TRUE(F3 < 0);
    EXPECT_NEAR(F4, 0, MAX_ERROR);
    EXPECT_TRUE(F5 > 0);
    EXPECT_TRUE(F6 > 0);
}

TEST_F(AllocatorTest, Downward)
{
    Publish(0, 0, 1, 0, 0, 0);
    WaitForMessage();

    EXPECT_TRUE(F1 > 0);
    EXPECT_NEAR(F2, 0, MAX_ERROR);
    EXPECT_NEAR(F3, 0, MAX_ERROR);
    EXPECT_TRUE(F4 > 0);
    EXPECT_NEAR(F5, 0, MAX_ERROR);
    EXPECT_NEAR(F6, 0, MAX_ERROR);
}

TEST_F(AllocatorTest, TiltUp)
{
    Publish(0, 0, 0, 0, 1, 0);
    WaitForMessage();

    EXPECT_TRUE(F1 < 0);
    EXPECT_NEAR(F2, 0, MAX_ERROR);
    EXPECT_NEAR(F3, 0, MAX_ERROR);
    EXPECT_TRUE(F4 > 0);
    EXPECT_NEAR(F5, 0, MAX_ERROR);
    EXPECT_NEAR(F6, 0, MAX_ERROR);
}

TEST_F(AllocatorTest, TurnRight)
{
    Publish(0, 0, 0, 0, 0, 1);
    WaitForMessage();

    EXPECT_NEAR(F1, 0, MAX_ERROR);
    EXPECT_TRUE(F2 < 0);
    EXPECT_TRUE(F3 > 0);
    EXPECT_NEAR(F4, 0, MAX_ERROR);
    EXPECT_TRUE(F5 > 0);
    EXPECT_TRUE(F6 < 0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "allocator_test");

    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
