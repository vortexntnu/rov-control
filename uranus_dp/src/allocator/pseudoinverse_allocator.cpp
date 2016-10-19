// See Fossen 2011, chapter 12.3.2
#include "pseudoinverse_allocator.h"

template<typename Derived>
inline bool isFucked(const Eigen::MatrixBase<Derived>& x)
{
  return !((x.array() == x.array())).all() && !( (x - x).array() == (x - x).array()).all();
}

void printEigen(std::string name, const Eigen::MatrixXd &x)
{
  std::stringstream ss;
  ss << name << " = " << std::endl << x;
  ROS_INFO_STREAM(ss.str());
}

Eigen::MatrixXd getParamMatrix(ros::NodeHandle nh, std::string name)
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
    ROS_WARN("Error in getParamMatrix. Returning 1-by-1 identity matrix.");
    return Eigen::MatrixXd::Identity(1,1);
  }
}

PseudoinverseAllocator::PseudoinverseAllocator()
{
  sub = nh.subscribe("rov_forces", 10, &PseudoinverseAllocator::callback, this);
  pub = nh.advertise<vortex_msgs::ThrusterForces>("thruster_forces", 10);

  if (!nh.getParam("/num_control_dof", n))
    ROS_WARN("Failed to read parameter num_control_dof.");
  if (!nh.getParam("/num_thrusters", r))
    ROS_WARN("Failed to read parameter num_thrusters.");

  W.setIdentity(); // Default to identity (i.e. equal weights)
  K.setIdentity(); // Scaling is done on Arduino, so this can be identity

  Eigen::MatrixXd T_copy = getParamMatrix(nh, "thrust_configuration");
  T = T_copy;

  K_inverse = K.inverse();
  computePseudoinverse();
}

void PseudoinverseAllocator::callback(const geometry_msgs::Wrench& tauMsg)
{
  tau << tauMsg.force.x,
         tauMsg.force.y,
         tauMsg.force.z,
         tauMsg.torque.y,
         tauMsg.torque.z;

  u = K_inverse * T_pseudoinverse * tau;

  if (isFucked(K_inverse))
    ROS_WARN("K is not invertible");

  vortex_msgs::ThrusterForces uMsg;
  uMsg.F1 = u(0);
  uMsg.F2 = u(1);
  uMsg.F3 = u(2);
  uMsg.F4 = u(3);
  uMsg.F5 = u(4);
  uMsg.F6 = u(5);
  pub.publish(uMsg);
  ROS_INFO("pseudoinverse_allocator: Publishing thruster forces.");
}

void PseudoinverseAllocator::setWeights(const Eigen::MatrixXd &W_new)
{
  bool isCorrectDimensions = ( W_new.rows() == r && W_new.cols() == r );
  if (!isCorrectDimensions)
  {
    ROS_WARN_STREAM("Attempt to set weight matrix in PseudoinverseAllocator with wrong dimensions " << W_new.rows() << "*" << W_new.cols() << ".");
    return;
  }

  W = W_new;

  // New weights require recomputing the generalized inverse of the thrust config matrix
  computePseudoinverse();
}

void PseudoinverseAllocator::computePseudoinverse()
{
  T_pseudoinverse = W.inverse()*T.transpose() * (T*W.inverse()*T.transpose()).inverse();

  if (isFucked(T_pseudoinverse))
    ROS_WARN("NaN in T_pseudoinverse.");
  if (isFucked(W.inverse()))
    ROS_WARN("W not invertible.");
  if (isFucked((T*W.inverse()*T.transpose()).inverse()))
    ROS_WARN("T * W_inv * T transposed is not invertible.");
}
