#include "ros/ros.h"
#include <gtest/gtest.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/FluidPressure.h"
#include "nav_msgs/Odometry.h"

// Pub imu/pressure and imu/data
// Sub state_estimate
// only msg.pose.pose.orientation and pose.pose.position.z are set
// all others should be zero

class EstimatorTest : public ::testing::Test
{
public:
  EstimatorTest()
  {
    imuPub = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
    pressurePub = nh.advertise<sensor_msgs::FluidPressure>("imu/pressure", 10);
    sub = nh.subscribe("state_estimate", 10, &EstimatorTest::Callback, this);
    message_received = false;
  }

  void SetUp()
  {
    while (!IsNodeReady())
      ros::spinOnce();
  }

  void PublishOrientation(double w, double x, double y, double z)
  {
    sensor_msgs::Imu msg;
    msg.orientation.w = w;
    msg.orientation.x = x;
    msg.orientation.y = y;
    msg.orientation.z = z;
    imuPub.publish(msg);
  }

  void PublishPressure(double p)
  {
    sensor_msgs::FluidPressure msg;
    msg.fluid_pressure = p;
    pressurePub.publish(msg);
  }

  // void ExpectThrustNear(double* arr)
  // {
  //   for (int i = 0; i < thrust.size(); ++i)
  //     EXPECT_NEAR(thrust[i], arr[i], MAX_ERROR);
  // }

  void WaitForMessage()
  {
    while (!message_received)
      ros::spinOnce();
  }

  // Orientation quaternion
  double q_w;
  double q_x;
  double q_y;
  double q_z;

  double depth;
  // const double        MAX_ERROR = 1e-4;

 private:
  ros::NodeHandle nh;
  ros::Publisher  imuPub;
  ros::Publisher  pressurePub;
  ros::Subscriber sub;
  bool message_received;

  void Callback(const nav_msgs::Odometry& msg)
  {
    q_w   = msg.pose.pose.orientation.w;
    q_x   = msg.pose.pose.orientation.x;
    q_y   = msg.pose.pose.orientation.y;
    q_z   = msg.pose.pose.orientation.z;
    depth = msg.pose.pose.position.z;
    message_received = true;
  }

  bool IsNodeReady()
  {
    return (imuPub.getNumSubscribers() > 0)
      && (pressurePub.getNumSubscribers() > 0)
      && (sub.getNumPublishers() > 0);
  }
};

TEST_F(EstimatorTest, CheckResponsiveness)
{
  PublishOrientation(1, 0, 0, 0);
  PublishPressure(0);
  WaitForMessage();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "estimator_test");

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
