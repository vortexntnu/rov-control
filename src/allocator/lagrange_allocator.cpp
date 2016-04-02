// See Fossen 2011, chapter 12.3.2
#include "lagrange_allocator.h"

LagrangeAllocator::LagrangeAllocator()
{
    n = 6;
    r = 6;

    tauSub = nh.subscribe("controlForces", 1, &LagrangeAllocator::tauCallback, this);
    uPub   = nh.advertise<uranus_dp::ThrusterForces>("controlInputs", 1);

    W.setIdentity(6,6);                 // Default to identity (i.e. no weights)
    K = Eigen::MatrixXd::Identity(6,6); // Set to identity because it's not yet known
    T = Eigen::MatrixXd::Ones(6,6);     // Set to ones for same reason

    K_inverse = K.inverse();
    computeGeneralizedInverse();
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

void LagrangeAllocator::setWeights(const Eigen::MatrixXd &W_new)
{
    bool correctDimensions = ( W_new.rows() == r && W_new.cols() == r );
    // ROS_INFO("correctDimensions = %d\n", correctDimensions);
    if (!correctDimensions)
    {
        ROS_WARN_STREAM("Attempt to set weight matrix in LagrangeAllocator with wrong dimensions " << W_new.rows() << "*" << W_new.cols() << ".\n");
        return;
    }

    W = W_new; // I have checked that Eigen does a deep copy here

    // New weights mean we must recompute the generalized inverse of the T matrix
    computeGeneralizedInverse();
}

void LagrangeAllocator::computeGeneralizedInverse()
{
    T_geninverse = W.inverse()*T.transpose() * (T*W.inverse()*T.transpose()).inverse();
}
