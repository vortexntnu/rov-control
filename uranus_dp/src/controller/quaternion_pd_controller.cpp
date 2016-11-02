#include "quaternion_pd_controller.h"

QuaternionPdController::QuaternionPdController(double a_init,
                                               double b_init,
                                               double c_init,
                                               Eigen::Vector3d r_G_init,
                                               Eigen::Vector3d r_B_init,
                                               double W_init,
                                               double B_init)
{
  setGains(a_init, b_init, c_init);
  r_G = r_G_init;
  r_B = r_B_init;
  W = W_init;
  B = B_init;
  // TODO: look into initialiser lists
}

void QuaternionPdController::setGains(double a_new, double b_new, double c_new)
{
  c   = c_new;
  K_d = a_new * Eigen::MatrixXd::Identity(6,6);
  K_x = b_new * Eigen::MatrixXd::Identity(3,3);
}

Eigen::Vector6d QuaternionPdController::compute(Eigen::Vector3d x, Eigen::Quaterniond q, Eigen::Vector6d nu, Eigen::Vector3d x_d, Eigen::Quaterniond q_d)
{
  Eigen::Matrix6d K_p = proportionalGainMatrix(q);
  Eigen::Vector6d z   = errorVector(x, x_d, q, q_d);
  Eigen::Vector6d g   = restoringForceVector(q);

  Eigen::Vector6d tau;
  tau = - K_d*nu - K_p*z + g;
  return tau;
}

Eigen::Matrix6d QuaternionPdController::proportionalGainMatrix(Eigen::Quaterniond q)
{
  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Matrix6d K_p;
  K_p << R.transpose() * K_x,        Eigen::MatrixXd::Zero(3,3),
         Eigen::MatrixXd::Zero(3,3), c*Eigen::MatrixXd::Identity(3,3);
  return K_p;
  // TODO: This could be a oneliner
}

Eigen::Vector6d QuaternionPdController::errorVector(Eigen::Vector3d x, Eigen::Vector3d x_d, Eigen::Quaterniond q, Eigen::Quaterniond q_d)
{
  Eigen::Vector3d x_tilde = x - x_d;

  Eigen::Quaterniond q_tilde = q_d.conjugate()*q;
  q_tilde.normalize();

  Eigen::Vector6d z;
  z << x_tilde,
       sgn(q_tilde.w())*q_tilde.vec();

  return z;
}

Eigen::Vector6d QuaternionPdController::restoringForceVector(Eigen::Quaterniond q)
{
  Eigen::Matrix3d R = q.toRotationMatrix();

  Eigen::Vector3d f_G = R.transpose() * Eigen::Vector3d(0, 0, W);
  Eigen::Vector3d f_B = R.transpose() * Eigen::Vector3d(0, 0, -B);

  Eigen::Vector6d g;
  g << f_G + f_B,
       r_G.cross(f_G) + r_B.cross(f_B);

  return g;
}

int QuaternionPdController::sgn(double x)
{
  if (x < 0)
    return -1;
  return 1;
}
