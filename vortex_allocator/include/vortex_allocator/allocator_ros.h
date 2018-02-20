#ifndef VORTEX_ALLOCATOR_ALLOCATOR_ROS_H
#define VORTEX_ALLOCATOR_ALLOCATOR_ROS_H

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"

#include "vortex_allocator/pseudoinverse_allocator.h"

#include <map>
#include <string>

class Allocator
{
public:
  explicit Allocator(ros::NodeHandle nh);
  void callback(const geometry_msgs::Wrench &msg) const;
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher  pub;

  int num_degrees_of_freedom;
  int num_thrusters;
  std::map<std::string, bool> active_degrees_of_freedom;
  double min_thrust;
  double max_thrust;
  const double FORCE_RANGE_LIMIT = 100;

  std::unique_ptr<PseudoinverseAllocator> pseudoinverse_allocator;

  Eigen::VectorXd rovForcesMsgToEigen(const geometry_msgs::Wrench &msg) const;
  bool healthyWrench(const Eigen::VectorXd &v) const;
};

#endif  // VORTEX_ALLOCATOR_ALLOCATOR_ROS_H
