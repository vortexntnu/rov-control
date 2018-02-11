#include "vortex_allocator/allocator_ros.h"

#include <cmath>
#include <vector>
#include <limits>

#include <eigen_conversions/eigen_msg.h>
#include "vortex_msgs/ThrusterForces.h"
#include "vortex/eigen_typedefs.h"
#include "vortex/eigen_helper.h"

Allocator::Allocator(ros::NodeHandle nh)
 :
nh(nh),
min_thrust(-std::numeric_limits<double>::infinity()),
max_thrust(std::numeric_limits<double>::infinity())
{
  sub = nh.subscribe("rov_forces", 1, &Allocator::callback, this);
  pub = nh.advertise<vortex_msgs::ThrusterForces>("thruster_forces", 1);

  if (!nh.getParam("/propulsion/dofs/num", num_degrees_of_freedom))
    ROS_FATAL("Failed to read parameter number of dofs.");
  if (!nh.getParam("/propulsion/thrusters/num", num_thrusters))
    ROS_FATAL("Failed to read parameter number of thrusters.");
  if (!nh.getParam("/propulsion/dofs/which", active_degrees_of_freedom))
    ROS_FATAL("Failed to read parameter which dofs.");

  // Read thrust limits
  std::vector<double> thrust;
  if (!nh.getParam("/thrusters/characteristics/thrust", thrust))
  {
    min_thrust = -std::numeric_limits<double>::infinity();
    max_thrust =  std::numeric_limits<double>::infinity();
    ROS_WARN_STREAM("Failed to read params min/max thrust. Defaulting to " << min_thrust << "/" << max_thrust << ".");
  }

  // Read thrust config matrix
  Eigen::MatrixXd thrust_configuration;
  if (!getMatrixParam(nh, "propulsion/thrusters/configuration_matrix", &thrust_configuration))
  {
    ROS_FATAL("Failed to read parameter thrust config matrix. Killing node...");
    ros::shutdown();
  }
  Eigen::MatrixXd thrust_configuration_pseudoinverse;
  if (!pseudoinverse(thrust_configuration, &thrust_configuration_pseudoinverse))
  {
    ROS_FATAL("Failed to compute pseudoinverse of thrust config matrix. Killing node...");
    ros::shutdown();
  }

  pseudoinverse_allocator.reset(new PseudoinverseAllocator(thrust_configuration_pseudoinverse));
  ROS_INFO("Initialized.");
}

void Allocator::callback(const geometry_msgs::Wrench &msg_in) const
{
  const Eigen::VectorXd rov_forces = rovForcesMsgToEigen(msg_in);

  if (!healthyWrench(rov_forces))
  {
    ROS_ERROR("Rov forces vector invalid, ignoring.");
    return;
  }

  Eigen::VectorXd thruster_forces = pseudoinverse_allocator->compute(rov_forces);

  if (isFucked(thruster_forces))
  {
    ROS_ERROR("Thruster forces vector invalid, ignoring.");
    return;
  }

  if (!saturateVector(&thruster_forces, min_thrust, max_thrust))
    ROS_WARN_THROTTLE(1, "Thruster forces vector required saturation.");

  vortex_msgs::ThrusterForces msg_out;
  arrayEigenToMsg(thruster_forces, &msg_out);
  msg_out.header.stamp = ros::Time::now();
  pub.publish(msg_out);
}

Eigen::VectorXd Allocator::rovForcesMsgToEigen(const geometry_msgs::Wrench &msg) const
{
  Eigen::VectorXd rov_forces(num_degrees_of_freedom);
  unsigned i = 0;
  if (active_degrees_of_freedom.at("surge"))
    rov_forces(i++) = msg.force.x;
  if (active_degrees_of_freedom.at("sway"))
    rov_forces(i++) = msg.force.y;
  if (active_degrees_of_freedom.at("heave"))
    rov_forces(i++) = msg.force.z;
  if (active_degrees_of_freedom.at("roll"))
    rov_forces(i++) = msg.torque.x;
  if (active_degrees_of_freedom.at("pitch"))
    rov_forces(i++) = msg.torque.y;
  if (active_degrees_of_freedom.at("yaw"))
    rov_forces(i++) = msg.torque.z;

  if (i != num_degrees_of_freedom)
  {
    ROS_WARN_STREAM("Invalid length of rov_forces vector. Is " << i << ", should be " << num_degrees_of_freedom <<
                    ". Returning zero thrust vector.");
    return Eigen::VectorXd::Zero(num_degrees_of_freedom);
  }

  return rov_forces;
}

bool Allocator::healthyWrench(const Eigen::VectorXd &v) const
{
  // Check for NaN/Inf
  if (isFucked(v))
    return false;

  // Check reasonableness
  for (unsigned i = 0; i < v.size(); ++i)
    if (std::abs(v[i]) > FORCE_RANGE_LIMIT)
      return false;

  return true;
}
