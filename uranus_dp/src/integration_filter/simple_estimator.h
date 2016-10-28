#ifndef SIMPLE_ESTIMATOR_H
#define SIMPLE_ESTIMATOR_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/FluidPressure.h"

#include <Eigen/Dense>

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

    double atmospheric_pressure;
    double water_density;
    double gravitational_acceleration;

    Eigen::Vector3d    position;
    Eigen::Quaterniond orientation;
};

#endif
