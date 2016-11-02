// See Fossen 2011, chapter 12.3.2
#include "pseudoinverse_allocator.h"

#include <vector>
#include <limits>
#include <string>

PseudoinverseAllocator::PseudoinverseAllocator()
{
  sub = nh.subscribe("rov_forces", 10, &PseudoinverseAllocator::callback, this);
  pub = nh.advertise<vortex_msgs::ThrusterForces>("thruster_forces", 10);

  getParams();

  tau.setZero(n);
  u.setZero(r);
  K_inv.setIdentity(r,r);

  if (!pseudoinverse(T, T_pinv))
    ROS_ERROR("Failed to compute pseudoinverse of thrust config matrix.");
}

void PseudoinverseAllocator::callback(const geometry_msgs::Wrench& tau_msg)
{
  {
    int i = 0;
    if (dofs["surge"])
    {
      tau(i) = tau_msg.force.x;
      ++i;
    }
    if (dofs["sway"])
    {
      tau(i) = tau_msg.force.y;
      ++i;
    }
    if (dofs["heave"])
    {
      tau(i) = tau_msg.force.z;
      ++i;
    }
    if (dofs["roll"])
    {
      tau(i) = tau_msg.torque.x;
      ++i;
    }
    if (dofs["pitch"])
    {
      tau(i) = tau_msg.torque.y;
      ++i;
    }
    if (dofs["yaw"])
    {
      tau(i) = tau_msg.torque.z;
      ++i;
    }

    if (i != n)
      ROS_WARN_STREAM("Allocator: Invalid length of tau vector. Is " << i << ", should be " << n);
  }

  u = K_inv * T_pinv * tau;
  if (!saturateVector(u, min_thrust, max_thrust))
    ROS_WARN("Thrust vector u required saturation.");

  if (isFucked(u))
  {
    ROS_WARN("Thrust vector u invalid, will not publish.");
    return;
  }

  vortex_msgs::ThrusterForces u_msg;
  thrustEigenToMsg(u, u_msg);
  pub.publish(u_msg);
}

void PseudoinverseAllocator::getParams()
{
  if (!nh.getParam("/thrusters/dof", n))
    ROS_FATAL("Failed to read parameter thrusters/dof.");
  if (!nh.getParam("/thrusters/num", r))
    ROS_FATAL("Failed to read parameter thrusters/dof.");
  if (!getMatrixParam(nh, "thrusters/configuration_matrix", T))
    ROS_ERROR("Failed to read thrust config matrix from param server.");

  std::vector<double> thrust;
  if (!nh.getParam("/thrust", thrust))
  {
    min_thrust = -std::numeric_limits<double>::infinity();
    max_thrust =  std::numeric_limits<double>::infinity();
    ROS_ERROR_STREAM("Failed to read parameters min/max thrust. Defaulting to " << min_thrust << "/" << max_thrust << ".");
  }
  else
  {
    min_thrust = thrust.front();
    max_thrust = thrust.back();
  }

  if (!nh.getParam("/control/dofs", dofs))
    ROS_FATAL("Failed to read parameter controllable dofs map.");
}
