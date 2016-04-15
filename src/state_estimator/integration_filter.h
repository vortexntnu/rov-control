#ifndef INTEGRATION_FILTER_H
#define INTEGRATION_FILTER_H

#include "ros/ros.h"
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
};

#endif
