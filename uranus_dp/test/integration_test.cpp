#include "ros/ros.h"
#include <gtest/gtest.h>
#include "boost/array.hpp"
#include "vortex_msgs/PropulsionCommand.h"
#include "vortex_msgs/ThrusterForces.h"
// #include "uranus_dp/control_mode_enum.h"
// #include <Eigen/Dense>

class IntegrationTest : public ::testing::Test
{
public:
  IntegrationTest()
  {
    pub = nh.advertise<vortex_msgs::PropulsionCommand>("propulsion_command", 10);
    sub = nh.subscribe("thruster_forces", 10, &IntegrationTest::Callback, this);
    message_received = false;
  }

  void SetUp()
  {
    while (!IsNodeReady())
      ros::spinOnce();
  }

  void PublishOpenloop(double forward, double right, double down, double roll_right, double tilt_up, double turn_right)
  {
    boost::array<double,6> propulsion = { {forward, right, down, roll_right, tilt_up, turn_right} };
    const unsigned char arr[] = {1, 0};
    std::vector<unsigned char> mode (arr, arr + sizeof(arr) / sizeof(arr[0]) );

    vortex_msgs::PropulsionCommand msg;
    msg.motion = propulsion;
    msg.control_mode = mode;
    pub.publish(msg);
  }

  void ExpectThrustNear(double* arr)
  {
    for (int i = 0; i < thrust.size(); ++i)
      EXPECT_NEAR(thrust[i], arr[i], MAX_ERROR);
  }

  void WaitForMessage()
  {
    while (!message_received)
      ros::spinOnce();
  }

  // bool HasReceivedMessage()
  // {
  //   return message_received;
  // }

  void OneSecondSpin()
  {
    ros::Time start = ros::Time::now();
    while (ros::Time::now() < start + ros::Duration(1))
      ros::spinOnce();
  }

  std::vector<double> thrust;
  static const double MAX_ERROR = 1e-4;

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

TEST_F(IntegrationTest, CheckResponsiveness)
{
  PublishOpenloop(0,0,0,0,0,0);
  WaitForMessage();
}

// Command forward motion, assure correct forces for each thruster.
TEST_F(IntegrationTest, Forward)
{
  PublishOpenloop(1,0,0,0,0,0);
  // OneSecondSpin();
  WaitForMessage();

  double thrust_expected[] = {0.35356, 0.35356, -0.35356, -0.35356, -0.20639, 0.20639};
  ExpectThrustNear(thrust_expected);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "integration_test");

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
