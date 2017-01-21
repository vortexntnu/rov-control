#include "ros/ros.h"
#include <gtest/gtest.h>
#include "geometry_msgs/Wrench.h"
#include "vortex_msgs/ThrusterForces.h"

class AllocatorTest : public ::testing::Test
{
public:
  AllocatorTest()
  {
    pub = nh.advertise<geometry_msgs::Wrench>("rov_forces", 10);
    sub = nh.subscribe("thruster_forces", 10, &AllocatorTest::Callback, this);
    message_received = false;

    if (!nh.getParam("/propulsion/thrusters/num", num_thrusters))
      ROS_FATAL("Failed to read parameter number of thrusters.");

    thrust.resize(num_thrusters);
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

  int                 num_thrusters;
  std::vector<double> thrust;
  static const double MAX_ERROR = 1e-6;

 private:
  ros::NodeHandle nh;
  ros::Publisher  pub;
  ros::Subscriber sub;
  bool message_received;

  void Callback(const vortex_msgs::ThrusterForces& msg)
  {
    thrust = msg.thrust;
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

  for(std::vector<double>::iterator it = thrust.begin(); it != thrust.end(); ++it)
  {
    EXPECT_NEAR(*it, 0, MAX_ERROR);
  }
}

TEST_F(AllocatorTest, Forward)
{
  Publish(1, 0, 0, 0, 0, 0);
  WaitForMessage();

  EXPECT_TRUE(thrust[0] > 0);
  EXPECT_TRUE(thrust[1] > 0);
  EXPECT_TRUE(thrust[2] < 0);
  EXPECT_TRUE(thrust[3] < 0);
  EXPECT_TRUE(thrust[4] < 0);
  EXPECT_TRUE(thrust[5] > 0);
}

TEST_F(AllocatorTest, Sideways)
{
  Publish(0, 1, 0, 0, 0, 0);
  WaitForMessage();

  EXPECT_TRUE(thrust[0] > 0);
  EXPECT_TRUE(thrust[1] < 0);
  EXPECT_TRUE(thrust[2] < 0);
  EXPECT_TRUE(thrust[3] > 0);
  EXPECT_NEAR(thrust[4], 0, MAX_ERROR);
  EXPECT_NEAR(thrust[5], 0, MAX_ERROR);
}

TEST_F(AllocatorTest, Downward)
{
  Publish(0, 0, 1, 0, 0, 0);
  WaitForMessage();

  EXPECT_NEAR(thrust[0], 0, MAX_ERROR);
  EXPECT_NEAR(thrust[1], 0, MAX_ERROR);
  EXPECT_NEAR(thrust[2], 0, MAX_ERROR);
  EXPECT_NEAR(thrust[3], 0, MAX_ERROR);
  EXPECT_TRUE(thrust[4] > 0);
  EXPECT_TRUE(thrust[5] > 0);
}

TEST_F(AllocatorTest, TiltUp)
{
  Publish(0, 0, 0, 0, 1, 0);
  WaitForMessage();

  EXPECT_NEAR(thrust[0], 0, MAX_ERROR);
  EXPECT_NEAR(thrust[1], 0, MAX_ERROR);
  EXPECT_NEAR(thrust[2], 0, MAX_ERROR);
  EXPECT_NEAR(thrust[3], 0, MAX_ERROR);
  EXPECT_TRUE(thrust[4] < 0);
  EXPECT_TRUE(thrust[5] > 0);
}

TEST_F(AllocatorTest, TurnRight)
{
  Publish(0, 0, 0, 0, 0, 1);
  WaitForMessage();

  EXPECT_TRUE(thrust[0] > 0);
  EXPECT_TRUE(thrust[1] < 0);
  EXPECT_TRUE(thrust[2] > 0);
  EXPECT_TRUE(thrust[3] < 0);
  EXPECT_NEAR(thrust[4], 0, MAX_ERROR);
  EXPECT_NEAR(thrust[5], 0, MAX_ERROR);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "allocator_test");

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
