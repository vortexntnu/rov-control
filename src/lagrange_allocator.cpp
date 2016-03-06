// See Fossen 2011, chapter 12.3.2

#include "lagrange_allocator.h"

LagrangeAllocator::LagrangeAllocator()
{
    n = 6;
    r = 6;

    tauSub = nh.subscribe("controlForces", 1, &LagrangeAllocator::tauCallback, this);
    uPub   = nh.advertise<uranus_dp::ThrusterForces>("controlInputs", 1);

    tau = Eigen::VectorXd(n);

    W.setIdentity(n,n);
    K = Eigen::MatrixXd::Identity(r,r); // Set to identity because it's not yet known
    T = Eigen::MatrixXd::Ones(n,r);

    K_inverse = K.inverse();
    T_geninverse = W.inverse()*T.transpose() * (T*W.inverse()*T.transpose()).inverse();
}

void LagrangeAllocator::tauCallback(const geometry_msgs::Wrench& tauMsg)
{
    tau << tauMsg.force.x,
           tauMsg.force.y,
           tauMsg.force.z,
           tauMsg.torque.x,
           tauMsg.torque.y,
           tauMsg.torque.z;

    u = K_inverse * T_geninverse * tau;

    uranus_dp::ThrusterForces uMsg;
    uMsg.F1 = u(0);
    uMsg.F2 = u(1);
    uMsg.F3 = u(2);
    uMsg.F4 = u(3);
    uMsg.F5 = u(4);
    uMsg.F6 = u(5);
    uPub.publish(uMsg);
}
