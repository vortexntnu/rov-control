#ifndef ALLOCATOR_ROS_H
#define ALLOCATOR_ROS_H

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"

#include "pseudoinverse_allocator.h"

class Allocator
{
public:
    Allocator(ros::NodeHandle nh);
    void callback(const geometry_msgs::Wrench &msg);
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher  pub;

    int num_dof;
    int num_thrusters;
    std::map<std::string,bool> dofs; // Map of controllable dofs
    double min_thrust;
    double max_thrust;
    const double FORCE_RANGE_LIMIT = 100;

    PseudoinverseAllocator *pseudoinverse_allocator;

    Eigen::VectorXd rovForcesMsgToEigen(const geometry_msgs::Wrench &msg);
    bool healthyWrench(const Eigen::VectorXd &v);
};

#endif
