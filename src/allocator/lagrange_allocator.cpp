// See Fossen 2011, chapter 12.3.2
#include "lagrange_allocator.h"

template<typename Derived>
inline bool isFucked(const Eigen::MatrixBase<Derived>& x)
{
  return !((x.array() == x.array())).all() && !( (x - x).array() == (x - x).array()).all();
}

LagrangeAllocator::LagrangeAllocator()
{
  sub = nh.subscribe("rov_forces", 10, &LagrangeAllocator::callback, this);
  pub = nh.advertise<maelstrom_msgs::ThrusterForces>("thruster_forces", 10);

  W.setIdentity(); // Default to identity (i.e. equal weights)
  K.setIdentity(); // Scaling is done on Arduino, so this can be identity

  // Thruster configuration does not allow control of roll.
  T <<  0     ,  0.7071, -0.7071, 0     ,  0.7071, -0.7071, // Surge
        0     , -0.7071, -0.7071, 0     ,  0.7071,  0.7071, // Sway
        1     ,  0     ,  0     , 1     ,  0     ,  0     , // Heave
       -0.1600, -0.0467,  0.0467, 0.1600, -0.0467,  0.0467, // Pitch
        0     , -0.2143,  0.2143, 0     ,  0.2143, -0.2143; // Yaw

  K_inverse = K.inverse();
  computeGeneralizedInverse();
}

void LagrangeAllocator::callback(const geometry_msgs::Wrench& tauMsg)
{
  tau << tauMsg.force.x,
         tauMsg.force.y,
         tauMsg.force.z,
         tauMsg.torque.y,
         tauMsg.torque.z;

  u = K_inverse * T_geninverse * tau;

  if (isFucked(K_inverse))
    ROS_WARN("K is not invertible");

  maelstrom_msgs::ThrusterForces uMsg;
  uMsg.F1 = u(0);
  uMsg.F2 = u(1);
  uMsg.F3 = u(2);
  uMsg.F4 = u(3);
  uMsg.F5 = u(4);
  uMsg.F6 = u(5);
  pub.publish(uMsg);
  ROS_INFO("lagrange_allocator: Publishing thruster forces.");
  // ROS_INFO_STREAM("lagrange_allocator: Sending " << u(0) << ", " << u(1) << ", " << u(2) << ", " << u(3) << ", " << u(4) << ", " << u(5));
}

void LagrangeAllocator::setWeights(const Eigen::MatrixXd &W_new)
{
  bool isCorrectDimensions = ( W_new.rows() == r && W_new.cols() == r );
  if (!isCorrectDimensions)
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

  if (isFucked(T_geninverse))
    ROS_WARN("NaN in T_geninverse.");
  if (isFucked(W.inverse()))
    ROS_WARN("W not invertible.");
  if (isFucked((T*W.inverse()*T.transpose()).inverse()))
    ROS_WARN("T * W_inv * T transposed is not invertible.");
}
