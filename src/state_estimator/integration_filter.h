#ifndef INTEGRATION_FILTER_H
#define INTEGRATION_FILTER_H

#include "ros/ros.h"
#include <Eigen/Dense>
#include "ros_arduino/SensorRaw.h"
#include "uranus_dp/State.h"

class IntegrationFilter
{
public:
    IntegrationFilter();
    void callback(const ros_arduino::SensorRaw &msg);
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher  pub_;

    Eigen::Quaterniond q;

    double curr_time_;
    double prev_time_;
};

#endif
