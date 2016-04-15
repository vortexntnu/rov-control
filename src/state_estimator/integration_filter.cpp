#include "integration_filter.h"

IntegrationFilter::IntegrationFilter()
{
    sub_ = nh_.subscribe("sensor", 10, &IntegrationFilter::callback, this);
    pub_ = nh_.advertise<uranus_dp::State>("state_estimate", 10);
}

void IntegrationFilter::callback(const ros_arduino::SensorRaw &msg)
{
    uranus_dp::State state_msg;
    pub_.publish(state_msg);
}
