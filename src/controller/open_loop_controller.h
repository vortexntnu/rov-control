/*
 * This 'controller' passes force/torque setpoints directly to the allocator.
 */

#ifndef OPEN_LOOP_CONTROLLER_H
#define OPEN_LOOP_CONTROLLER_H

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"

class OpenLoopController
{
public:
    OpenLoopController();
    void callback(const geometry_msgs::Wrench &msg);
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher  pub;
};

#endif
