// See Fossen 2011, chapter 12.3.2
#include "pseudoinverse_allocator.h"

PseudoinverseAllocator::PseudoinverseAllocator()
{
  sub = nh.subscribe("rov_forces", 10, &PseudoinverseAllocator::callback, this);
  pub = nh.advertise<vortex_msgs::ThrusterForces>("thruster_forces", 10);

  if (!nh.getParam("/thrusters/dof", n))
    ROS_FATAL("Failed to read parameter thrusters/dof.");
  if (!nh.getParam("/thrusters/num", r))
    ROS_FATAL("Failed to read parameter thrusters/dof.");
  if (!getMatrixParam(nh, "thrusters/configuration_matrix", T))
    ROS_ERROR("Failed to read thrust config matrix from param server.");

  tau.setZero(n);
  u.setZero(r);
  K_inv.setIdentity(r,r);

  if (!pinv(T, T_pinv))
    ROS_ERROR("Failed to compute pseudoinverse of thrust config matrix.");
}

void PseudoinverseAllocator::callback(const geometry_msgs::Wrench& tauMsg)
{
  tau << tauMsg.force.x,
         tauMsg.force.y,
         tauMsg.force.z,
         tauMsg.torque.y,
         tauMsg.torque.z;

  u = K_inv * T_pinv * tau;

  if (isFucked(u))
  {
    ROS_WARN("Thrust vector u invalid, will not publish.");
    return;
  }

  vortex_msgs::ThrusterForces uMsg;
  thrustEigenToMsg(u, uMsg);
  pub.publish(uMsg);
}
