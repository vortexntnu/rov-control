#ifndef DEPTH_HOLD_CONTROLLER_H
#define DEPTH_HOLD_CONTROLLER_H

#include "ros/ros.h"
#include "sensor_msgs/FluidPressure.h"
#include "geometry_msgs/Wrench.h"

class DepthHoldController
{
public:
  DepthHoldController();
  void pressureCallback(const sensor_msgs::FluidPressure& msg);
  void setpointCallback(const sensor_msgs::FluidPressure& msg);
  void openloopCallback(const geometry_msgs::Wrench& msg);
  void compute();
  void enable();
  void disable();
private:
  ros::NodeHandle nh;
  ros::Subscriber pressureSub;
  ros::Subscriber setpointSub;
  ros::Subscriber openloopSub;
  ros::Publisher  controlPub;
  bool enabled;

  double k_p;
  double pressure_reading;
  double pressure_setpoint;

  double surge_command;
  double sway_command;
  double pitch_command;
  double yaw_command;

  // static const double WATER_DENSITY    = 1000;    // [kg/m^3]
  // static const double STANDARD_GRAVITY = 9.08665; // [m/s^2]
};

#endif
