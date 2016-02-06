// See Fossen 2011, chapter 12.3.2
//
// Explanation of variables:
//  u   - control input vector (6)
//  K   - thrust coefficient matrix (6x6)
//  T   - thrust configuration matrix (6x6?)
//  tau - vector of forces on the ROV (6)
//
// Explanation of relationships:
//  f = K*u are the forces of each actuator in Newtons
//  tau = T*f are the forces on the ROV in Newtons and Newton meters

#include "ros/ros.h"
#include <iostream>
#include <Eigen/Dense> // Require Eigen library for matrix operations

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

const int nActuators = 6; // Move to parameter server

int main(int argc, char **argv)
{
    ros::init(argc, argv, "allocation");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("forces", 1 );

    // K and T should probably be given by a ROS parameter server.
    // For now, temporary values are hard-coded here.
    double thrustCoeff = 5; // Use same value for all actuators as they are the same.
    MatrixXd K = (thrustCoeff * VectorXd::Ones(nActuators)).asDiagonal();
    MatrixXd T = MatrixXd::Random(6,6);

    //
    // Place inside a callback function:
    //

    // read tau from topic (or just hardcode one for now)
    VectorXd tau(6);
    tau << 1,2,3,4,5,6;

    // then calculate the control inputs with Moore-Penrose pseudoinverse
    MatrixXd T_pseudoinverse = T.transpose() * (T * T.transpose()).inverse();
    VectorXd u = K.inverse() * T_pseudoinverse * tau;

    // send u on some topic

    return 0;
}
