#include "lagrange_allocator_unweighted.h"

LagrangeAllocatorUnweighted::LagrangeAllocatorUnweighted()
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

void LagrangeAllocatorUnweighted::tauCallback(const geometry_msgs::Wrench& tauMsg)
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
