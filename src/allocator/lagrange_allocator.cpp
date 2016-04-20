// See Fossen 2011, chapter 12.3.2
#include "lagrange_allocator.h"
#include <iostream>
#include <math.h>

template<typename Derived>
inline bool isFucked(const Eigen::MatrixBase<Derived>& x)
{
    return !((x.array() == x.array())).all() && !( (x - x).array() == (x - x).array()).all();
}

// Function to check for existence of Indian bread
template<typename T, int r, int c>
inline bool hasNan(const Eigen::Matrix<T,r,c>& M)
{
    int rows = M.rows();
    int cols = M.cols();
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            if (isnan(M(i,j)))
            {
                return true;
            }
        }
    }
    return false;
}

void printMatrix6(Eigen::MatrixXd m);

LagrangeAllocator::LagrangeAllocator()
{
    sub = nh.subscribe("rov_forces", 10, &LagrangeAllocator::callback, this);
    pub = nh.advertise<maelstrom_msgs::ThrusterForces>("thruster_forces", 10);

    W.setIdentity(); // Default to identity (i.e. equal weights)
    K.setIdentity(); // Scaling is done on Arduino, so this can be identity

    // Thruster configuration does not allow control of roll.
    T <<  0.7071 ,  0.7071 , -0.7071 , -0.7071 ,  0     ,  0     , // Surge
          0.7071 , -0.7071 , -0.7071 ,  0.7071 ,  0     ,  0     , // Sway
          0      ,  0      ,  0      ,  0      ,  1     ,  1     , // Heave
         -0.04667, -0.04667,  0.04667,  0.04667, -0.1600,  0.1600, // Pitch
          0.2143 , -0.2143 ,  0.2143 , -0.2143 ,  0     ,  0     ; // Yaw

    K_inverse = K.inverse();
    computeGeneralizedInverse();
}

void LagrangeAllocator::callback(const geometry_msgs::Wrench& tauMsg)
{
    tau << tauMsg.force.x,
           tauMsg.force.y,
           tauMsg.force.z,
           // tauMsg.torque.x, // Roll is not controllable
           tauMsg.torque.y,
           tauMsg.torque.z;

    u = K_inverse * T_geninverse * tau;

    if (isFucked(K_inverse))
    {
        ROS_WARN("K is not invertible");
    }

    maelstrom_msgs::ThrusterForces uMsg;
    uMsg.F1 = u(0);
    uMsg.F2 = u(1);
    uMsg.F3 = u(2);
    uMsg.F4 = u(3);
    uMsg.F5 = u(4);
    uMsg.F6 = u(5);
    pub.publish(uMsg);

    ROS_INFO_STREAM("lagrange_allocator: Sending " << u(0) << ", " << u(1) << ", " << u(2) << ", " << u(3) << ", " << u(4) << ", " << u(5));
}

void LagrangeAllocator::setWeights(const Eigen::MatrixXd &W_new)
{
    bool correctDimensions = ( W_new.rows() == r && W_new.cols() == r );
    if (!correctDimensions)
    {
        ROS_WARN_STREAM("Attempt to set weight matrix in LagrangeAllocator with wrong dimensions " << W_new.rows() << "*" << W_new.cols() << ".");
        return;
    }

    W = W_new;

    // New weights require recomputing the generalized inverse of the thrust config matrix
    computeGeneralizedInverse();
}

void LagrangeAllocator::computeGeneralizedInverse()
{
    T_geninverse = W.inverse()*T.transpose() * (T*W.inverse()*T.transpose()).inverse();

    // ROS_INFO("Printing in order: T_geninverse, W, T, K_inverse.");
    // printMatrix6(T_geninverse);
    // printMatrix6(W);
    // printMatrix6(T);
    // printMatrix6(K_inverse);

    if (isFucked(T_geninverse))
    {
        ROS_WARN("T_geninverse NAN");
    }

    if (isFucked( W.inverse() ))
    {
        ROS_WARN("W is not invertible");
    }

    if (isFucked( (T*W.inverse()*T.transpose()).inverse() ) )
    {
        ROS_WARN("T * W_inv * T transposed is not invertible");
    }
} 


// Worlds worst print function :DDD
void printMatrix6(Eigen::MatrixXd m)
{
    ROS_INFO("----------------------------------");
    for(int col = 0; col < 6; col++)
    {
        char buffer[512];

        for(int row = 0; row < 6; row++)
        {
            snprintf( (buffer + (row*8) ), sizeof(buffer), "  %f  ", m(col, row));
        }
        ROS_INFO("%s", buffer);
    }
    ROS_INFO("----------------------------------\n");
}