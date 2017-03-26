// This state estimator simply reads IMU orientation and a pressure sensor,
// and publishes orientation and z-axis position based on the readings.
// The velocity twist is always zero, and x- and y-axis position is always
// zero.

#ifndef VORTEX_ESTIMATOR_SIMPLE_ESTIMATOR_H
#define VORTEX_ESTIMATOR_SIMPLE_ESTIMATOR_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/FluidPressure.h"

class SimpleEstimator
{
  public:
    SimpleEstimator();
    void imuCallback(const sensor_msgs::Imu &msg);
    void pressureCallback(const sensor_msgs::FluidPressure &msg);
    void publish();
  private:
    ros::NodeHandle nh;
    ros::Subscriber imu_sub;
    ros::Subscriber pressure_sub;
    ros::Publisher  state_pub;

    bool is_initialized;
    bool imu_initialized;
    bool pressure_initialized;

    double atmospheric_pressure;
    double water_density;
    double gravitational_acceleration;

    nav_msgs::Odometry state;
};

#endif  // VORTEX_ESTIMATOR_SIMPLE_ESTIMATOR_H
