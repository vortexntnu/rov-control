#include "vortex_controller/quaternion_pd_controller.h"

QuaternionPdController::QuaternionPdController(double a,
                                               double b,
                                               double c,
                                               double W,
                                               double B,
                                               const Eigen::Vector3d &r_G,
                                               const Eigen::Vector3d &r_B)
: r_G(r_G), r_B(r_B), W(W), B(B)
{
  setGains(a, b, c);
}

void QuaternionPdController::setGains(double a_new, double b_new, double c_new)
{
  c   = c_new;
  K_d = a_new * Eigen::MatrixXd::Identity(6, 6);
  K_x = b_new * Eigen::MatrixXd::Identity(3, 3);
}

Eigen::Vector6d QuaternionPdController::compute(const Eigen::Vector3d    &x,
                                                const Eigen::Quaterniond &q,
                                                const Eigen::Vector6d    &nu,
                                                const Eigen::Vector3d    &x_d,
                                                const Eigen::Quaterniond &q_d)
{
  Eigen::Matrix3d R   = q.toRotationMatrix();
  Eigen::Matrix6d K_p = proportionalGainMatrix(R);
  Eigen::Vector6d z   = errorVector(x, x_d, q, q_d);
  Eigen::Vector6d g   = restoringForceVector(R);
  return (Eigen::Vector6d() << -K_d*nu - K_p*z + g).finished();
}

Eigen::Matrix6d QuaternionPdController::proportionalGainMatrix(const Eigen::Matrix3d &R)
{
  return (Eigen::Matrix6d() << R.transpose() * K_x,        Eigen::MatrixXd::Zero(3, 3),
                               Eigen::MatrixXd::Zero(3, 3), c*Eigen::MatrixXd::Identity(3, 3)).finished();
}

Eigen::Vector6d QuaternionPdController::errorVector(const Eigen::Vector3d    &x,
                                                    const Eigen::Vector3d    &x_d,
                                                    const Eigen::Quaterniond &q,
                                                    const Eigen::Quaterniond &q_d)
{
  Eigen::Quaterniond q_tilde = q_d.conjugate()*q;
  q_tilde.normalize();
  return (Eigen::Vector6d() << x - x_d, sgn(q_tilde.w())*q_tilde.vec()).finished();
}

Eigen::Vector6d QuaternionPdController::restoringForceVector(const Eigen::Matrix3d &R)
{
  Eigen::Vector3d f_G = R.transpose() * Eigen::Vector3d(0, 0, W);
  Eigen::Vector3d f_B = R.transpose() * Eigen::Vector3d(0, 0, -B);
  return (Eigen::Vector6d() << f_G + f_B, r_G.cross(f_G) + r_B.cross(f_B)).finished();
}

int QuaternionPdController::sgn(double x)
{
  if (x < 0)
    return -1;
  return 1;
}
