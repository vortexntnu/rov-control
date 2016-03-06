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
#include <Eigen/Dense>
#include "geometry_msgs/Wrench.h"
#include "uranus_dp/ThrusterForces.h"

class LagrangeAllocatorUnweighted
{
public:
    LagrangeAllocatorUnweighted()
    {
        nThrusters = 6;
        thrustCoeff = 5;
        tauSub = n.subscribe("forces", 1, &LagrangeAllocatorUnweighted::tauCallback, this);
        uPub   = n.advertise<uranus_dp::ThrusterForces>("U", 1);
        tau = Eigen::VectorXd(6);
        K = thrustCoeff * Eigen::MatrixXd::Identity(nThrusters, nThrusters);
        K_inverse = K.inverse();
        T = Eigen::MatrixXd::Random(6,6);
        T_pseudoinverse = T.transpose() * (T * T.transpose()).inverse();
    }
    void tauCallback(const geometry_msgs::Wrench& tauMsg)
    {
        tau(0) = tauMsg.force.x;
        tau(1) = tauMsg.force.y;
        tau(2) = tauMsg.force.z;
        tau(3) = tauMsg.torque.x;
        tau(4) = tauMsg.torque.y;
        tau(5) = tauMsg.torque.z;

        u = K_inverse * T_pseudoinverse * tau;

        uranus_dp::ThrusterForces uMsg;
        uMsg.F1 = u(0);
        uMsg.F2 = u(1);
        uMsg.F3 = u(2);
        uMsg.F4 = u(3);
        uMsg.F5 = u(4);
        uMsg.F6 = u(5);
        uPub.publish(uMsg);
    }
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "allocation");
    LagrangeAllocatorUnweighted allocator;
    ros::spin();
    return 0;
}
