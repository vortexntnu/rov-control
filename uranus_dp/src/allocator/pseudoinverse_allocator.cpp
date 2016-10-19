// See Fossen 2011, chapter 12.3.2
#include "pseudoinverse_allocator.h"

void printEigen(std::string name, const Eigen::MatrixXd &x)
{
  std::stringstream ss;
  ss << name << " = " << std::endl << x;
  ROS_INFO_STREAM(ss.str());
}

Eigen::MatrixXd getMatrixParam(ros::NodeHandle nh, std::string name)
{
  XmlRpc::XmlRpcValue matrix;
  nh.getParam(name, matrix);

  try
  {
    const int rows = matrix.size();
    const int cols = matrix[0].size();
    Eigen::MatrixXd X(rows,cols);
    for (int i = 0; i < rows; ++i)
      for (int j = 0; j < cols; ++j)
        X(i,j) = matrix[i][j];
    return X;
  }
  catch(...)
  {
    ROS_ERROR("Error in getMatrixParam. Returning 1-by-1 zero matrix.");
    return Eigen::MatrixXd::Zero(1,1);
  }
}

Eigen::MatrixXd pinv(const Eigen::MatrixXd &X)
{
  Eigen::MatrixXd X_pinv = X.transpose() * ( X*X.transpose() ).inverse();

  if (isFucked(X_pinv))
  {
    ROS_WARN("Could not compute pseudoinverse. Returning transpose.");
    return X.transpose();
  }

  return X_pinv;
}

PseudoinverseAllocator::PseudoinverseAllocator()
{
  sub = nh.subscribe("rov_forces", 10, &PseudoinverseAllocator::callback, this);
  pub = nh.advertise<vortex_msgs::ThrusterForces>("thruster_forces", 10);

  if (!nh.getParam("/num_control_dof", n))
    ROS_FATAL("Failed to read parameter num_control_dof.");
  if (!nh.getParam("/num_thrusters", r))
    ROS_FATAL("Failed to read parameter num_thrusters.");

  tau.setZero(n);
  u.setZero(r);
  K_inv.setIdentity(r,r);
  T = getMatrixParam(nh, "thrust_configuration");
  T_pinv = pinv(T);
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
    ROS_WARN("Thruster forces u invalid, will not publish.");
    return;
  }

  vortex_msgs::ThrusterForces uMsg;
  uMsg.F1 = u(0);
  uMsg.F2 = u(1);
  uMsg.F3 = u(2);
  uMsg.F4 = u(3);
  uMsg.F5 = u(4);
  uMsg.F6 = u(5);
  pub.publish(uMsg);
}
