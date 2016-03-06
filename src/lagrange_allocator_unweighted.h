#ifndef LAGRANGE_ALLOCATOR_UNWEIGHTED_H
#define LAGRANGE_ALLOCATOR_UNWEIGHTED_H

#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/Wrench.h"
#include "uranus_dp/ThrusterForces.h"

class LagrangeAllocatorUnweighted
{
public:
    LagrangeAllocatorUnweighted();
    void tauCallback(const geometry_msgs::Wrench& tauMsg);
private:
    ros::NodeHandle n;
    ros::Subscriber tauSub;
    ros::Publisher  uPub;

    int nThrusters; // Param server
    double thrustCoeff; // Ditto

    Eigen::VectorXd u;
    Eigen::VectorXd tau;
    Eigen::MatrixXd K;
    Eigen::MatrixXd K_inverse;
    Eigen::MatrixXd T;
    Eigen::MatrixXd T_pseudoinverse;
};

#endif
