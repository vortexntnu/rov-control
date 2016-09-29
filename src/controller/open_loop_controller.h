#ifndef OPEN_LOOP_CONTROLLER_H
#define OPEN_LOOP_CONTROLLER_H

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"

class OpenLoopController
{
public:
  OpenLoopController();
  void callback(const geometry_msgs::Wrench &msg);
  void enable();
  void disable();
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher  pub;
  bool enabled;
};

#endif
