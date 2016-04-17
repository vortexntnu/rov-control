#ifndef INTEGRATION_FILTER_H
#define INTEGRATION_FILTER_H

#include "ros/ros.h"
#include <Eigen/Dense>
#include "maelstrom_msgs/SensorRaw.h"
#include "maelstrom_msgs/State.h"

class IntegrationFilter
{
public:
    IntegrationFilter();
    void callback(const maelstrom_msgs::SensorRaw &msg);
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher  pub_;

    Eigen::Quaterniond q;

    double curr_time_;
    double prev_time_;
};

#endif
