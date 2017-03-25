#include "ros/ros.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include "vortex_msgs/PropulsionCommand.h"

class ControllerTest : public ::testing::Test
{
public:
  ControllerTest()
  {
    cmdPub = nh.advertise<vortex_msgs::PropulsionCommand>("propulsion_command", 10);
    message_received = false;
  }

  void SetUp()
  {
    while (!IsNodeReady())
      ros::spinOnce();
  }

  void PublishCommand()
  {
    vortex_msgs::PropulsionCommand msg;
    cmdPub.publish(msg);
  }

  void WaitForMessage()
  {
    while (!message_received)
      ros::spinOnce();
  }

  Eigen::Matrix<double,6,1> tau;

private:
  ros::NodeHandle nh;
  ros::Publisher cmdPub;
  ros::Subscriber sub;

  bool message_received;

  void Callback(const geometry_msgs::Wrench& msg)
  {
    tf::wrenchMsgToEigen(msg, tau);
    message_received = true;
  }

  bool IsNodeReady()
  {
    return ((cmdPub.getNumSubscribers() > 0) && (sub.getNumPublishers() > 0));
  }
};

TEST_F(ControllerTest, CheckResponsiveness)
{
  ros::Duration(0.5).sleep();
  PublishCommand();
  WaitForMessage();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "controller_test");

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
