// See Fossen 2011, chapter 12.3.2
//
// Explanation of variables:
//  u   - control input vector (6)
//  K   - thrust coefficient matrix (6x6)
//  T   - thrust configuration matrix (6x6?)
//  tau - vector of forces on the ROV (6)
//
// Mathematical relationships:
//  f = K*u are the forces of each actuator in Newtons
//  tau = T*f are the forces on the ROV in Newtons and Newton meters

#include "ros/ros.h"
#include <iostream>
#include <Eigen/Dense> // Library for matrix operations
#include "geometry_msgs/Wrench.h"
#include "uranus_dp/ThrusterForces.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


const int nActuators = 6; // Move to parameter server

class Allocator{
    private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
        MatrixXd T_pseudoinverse;
        VectorXd uVector;
        uranus_dp::ThrusterForces u;
        VectorXd tau;
        MatrixXd K;
        MatrixXd T;
    public:
        Allocator(){
            sub = n.subscribe("forces", 1, &Allocator::tauCallBack, this);
            pub = n.advertise<uranus_dp::ThrusterForces>("U", 1);
            tau = VectorXd(6);
        }
        void tauCallBack(const geometry_msgs::Wrench& tauMsg){
            double thrustCoeff = 5;
            K = (thrustCoeff * VectorXd::Ones(nActuators)).asDiagonal();
            T = MatrixXd::Random(6,6);

            tau(0) = tauMsg.force.x;
            tau(1) = tauMsg.force.y;
            tau(2) = tauMsg.force.z;
            tau(3) = tauMsg.torque.x;
            tau(4) = tauMsg.torque.y;
            tau(5) = tauMsg.torque.z;

            T_pseudoinverse = T.transpose() * (T * T.transpose()).inverse();
            uVector = K.inverse() * T_pseudoinverse * tau;

            u.F1 = uVector(0);
            u.F2 = uVector(1);
            u.F3 = uVector(2);
            u.F4 = uVector(3);
            u.F5 = uVector(4);
            u.F6 = uVector(5);


            pub.publish(u);
        }

};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "allocation");
    Allocator allocator;
    ros::spin();
    return 0;
}
