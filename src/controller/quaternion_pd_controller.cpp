#include "quaternion_pd_controller.h"

template<typename Derived>
inline bool isFucked(const Eigen::MatrixBase<Derived>& x)
{
  return !((x.array() == x.array())).all() && !( (x - x).array() == (x - x).array()).all();
}

QuaternionPdController::QuaternionPdController()
{
  stateSub    = nh.subscribe("state_estimate", 10, &QuaternionPdController::stateCallback, this);
  setpointSub = nh.subscribe("pose_setpoints", 10, &QuaternionPdController::setpointCallback, this);
  controlPub  = nh.advertise<geometry_msgs::Wrench>("rov_forces", 10);
  enabled = false;

  p.setZero();
  q.setIdentity();
  nu.setZero();
  p_d.setZero();
  q_d.setIdentity();
  z.setZero();
  tau.setZero();
  g.setZero();
  R = q.toRotationMatrix();

  setGains(0.1, 3.0, 20.0); // Gains taken from paper and divided by 10 (our ROV is rougly a tenth the size of the one in the paper)
  r_g << 0, 0, -0.05;       // Center of gravity estimated 5 cm below origin of body frame
  r_b << 0, 0,  0.10;       // Center of buoyancy estimated 10 cm above origin of body frame
  W = 15*9.80665;           // Weight in water estimated to 15 kg
  B = 16*9.80665;           // Buoyancy estimated to 16 kg
}

void QuaternionPdController::stateCallback(const nav_msgs::Odometry &msg)
{
  tf::pointMsgToEigen(msg.pose.pose.position, p);
  tf::quaternionMsgToEigen(msg.pose.pose.orientation, q);
  tf::twistMsgToEigen(msg.twist.twist, nu);
  q.normalize();
  R = q.toRotationMatrix();

  if (isFucked(p) || isFucked(nu) || isFucked(R))
    ROS_WARN("p, nu, or R is fucked.");
}

void QuaternionPdController::setpointCallback(const geometry_msgs::Pose &msg)
{
  tf::pointMsgToEigen(msg.position, p_d);
  tf::quaternionMsgToEigen(msg.orientation, q_d);
  q_d.normalize();

  if (isFucked(p_d))
    ROS_WARN("p_d is fucked.");
}

void QuaternionPdController::setGains(double a_new, double b_new, double c_new)
{
  a = a_new;
  b = b_new;
  c = c_new;
  K_D = a * Eigen::MatrixXd::Identity(6,6);
  K_p = b * Eigen::MatrixXd::Identity(3,3);
}

void QuaternionPdController::compute()
{
  if (enabled)
  {
    updateProportionalGainMatrix();
    updateErrorVector();
    updateRestoringForceVector();

    tau = - K_D*nu - K_P*z + g;

    if (isFucked(tau))
      ROS_WARN("tau is fucked.");

    geometry_msgs::Wrench msg;
    tf::wrenchEigenToMsg(tau, msg);
    controlPub.publish(msg);
  }
}

void QuaternionPdController::enable()
{
  enabled = true;
}

void QuaternionPdController::disable()
{
  enabled = false;
}

void QuaternionPdController::updateProportionalGainMatrix()
{
  K_P << R.transpose() * K_p,        Eigen::MatrixXd::Zero(3,3),
         Eigen::MatrixXd::Zero(3,3), c*Eigen::MatrixXd::Identity(3,3);

  if (isFucked(K_P))
    ROS_WARN("K_P is fucked.");
}

void QuaternionPdController::updateErrorVector()
{
  Eigen::Vector3d p_tilde = p - p_d;

  if (isFucked(p_tilde))
    ROS_WARN("p_tilde is fucked.");

  Eigen::Quaterniond q_tilde = q_d.conjugate()*q;
  q_tilde.normalize();

  z << p_tilde,
     sgn(q_tilde.w())*q_tilde.vec();

  if (isFucked(z))
    ROS_WARN("z is fucked.");
}

void QuaternionPdController::updateRestoringForceVector()
{
  Eigen::Vector3d f_g = R.transpose() * Eigen::Vector3d(0, 0, W);
  Eigen::Vector3d f_b = R.transpose() * Eigen::Vector3d(0, 0, -B);

  g << f_g + f_b,
       r_g.cross(f_g) + r_b.cross(f_b);

  if (isFucked(g))
    ROS_WARN("g is fucked.");
}

int QuaternionPdController::sgn(double x)
{
  if (x < 0)
    return -1;
  return 1;
}

Eigen::Matrix3d QuaternionPdController::skew(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d S;
  S <<  0   , -v(2),  v(1),
        v(2),  0,    -v(0),
       -v(1),  v(0),  0   ;
  return S;
}
