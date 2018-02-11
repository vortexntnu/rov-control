#include "ros/ros.h"
#include <gtest/gtest.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/FluidPressure.h"
#include "vortex_msgs/RovState.h"

// Pub imu/pressure and imu/data
// Sub state_estimate
// only msg.pose.pose.orientation and pose.pose.position.z are set
// all others should be zero

class EstimatorTest : public ::testing::Test
{
public:
  EstimatorTest()
  {
    imuPub = nh.advertise<sensor_msgs::Imu>("/sensors/imu/data", 10);
    pressurePub = nh.advertise<sensor_msgs::FluidPressure>("/sensors/pressure", 10);
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

  void WaitForMessage()
  {
    while (!message_received)
      ros::spinOnce();
  }

  void SpinSeconds(double duration)
  {
    ros::Time start = ros::Time::now();
    while (ros::Time::now() - start < ros::Duration(duration))
    {
      ros::spinOnce();
    }
  }

  // Orientation quaternion
  double q_w;
  double q_x;
  double q_y;
  double q_z;

  double depth;

  const double MAX_ERROR = 1e-4;

 private:
  ros::NodeHandle nh;
  ros::Publisher  imuPub;
  ros::Publisher  pressurePub;
  ros::Subscriber sub;
  bool message_received;

  void Callback(const vortex_msgs::RovState& msg)
  {
    q_w   = msg.pose.orientation.w;
    q_x   = msg.pose.orientation.x;
    q_y   = msg.pose.orientation.y;
    q_z   = msg.pose.orientation.z;
    depth = msg.pose.position.z;
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

TEST_F(EstimatorTest, OrientationCorrect)
{
  PublishOrientation(1, 2, 3, 4);
  SpinSeconds(0.1);

  EXPECT_NEAR(q_w, 1, MAX_ERROR);
  EXPECT_NEAR(q_x, 2, MAX_ERROR);
  EXPECT_NEAR(q_y, 3, MAX_ERROR);
  EXPECT_NEAR(q_z, 4, MAX_ERROR);
}

TEST_F(EstimatorTest, DepthCorrect)
{
  PublishPressure(200000);
  SpinSeconds(0.1);

  EXPECT_NEAR(depth, 10.0728, MAX_ERROR);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "estimator_test");

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
