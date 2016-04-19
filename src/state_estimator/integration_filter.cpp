#include "integration_filter.h"

IntegrationFilter::IntegrationFilter()
{
    sub_ = nh_.subscribe("sensor", 10, &IntegrationFilter::callback, this);
    pub_ = nh_.advertise<maelstrom_msgs::State>("state_estimate", 10);
}

void IntegrationFilter::callback(const maelstrom_msgs::SensorRaw &msg)
{
    maelstrom_msgs::State state_msg;
    pub_.publish(state_msg);
}
