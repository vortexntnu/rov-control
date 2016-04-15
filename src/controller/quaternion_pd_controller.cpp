// Based on Fjellstad & Fossen 1994: Quaternion Feedback Regulation of Underwater Vehicles

#include "quaternion_pd_controller.h"

QuaternionPdController::QuaternionPdController()
{
    stateSub    = nh.subscribe("state_estimate", 10, &QuaternionPdController::stateCallback, this);
    setpointSub = nh.subscribe("pose_setpoints", 10, &QuaternionPdController::setpointCallback, this);
    controlPub  = nh.advertise<geometry_msgs::Wrench>("rov_forces", 10);
    enabled = false;

    // Initial values
    p         << 0, 0, 0;
    q.w()     =  0;
    q.vec()   << 0, 0, 0;
    nu        << 0, 0, 0, 0, 0, 0;
    p_d       << 0, 0, 0;
    q_d.w()   =  0;
    q_d.vec() << 0, 0, 0;
    tau       << -1, -1, -1, -1, -1, -1;

    // Gains etc. (from paper, temporary)
    K_D = Eigen::MatrixXd::Identity(6,6);
    K_p = 30*Eigen::MatrixXd::Identity(3,3);
    c = 200;
    r_g.setZero();
    r_b.setZero();
    W = 185*9.8;
    B = 185*9.8;
}

void QuaternionPdController::stateCallback(const uranus_dp::State &msg)
{
    tf::pointMsgToEigen(msg.pose.position, p);
    tf::quaternionMsgToEigen(msg.pose.orientation, q);
    tf::twistMsgToEigen(msg.twist, nu);

    // Update rotation matrix
    R = q.toRotationMatrix();
}

void QuaternionPdController::setpointCallback(const geometry_msgs::Pose &msg)
{
    tf::pointMsgToEigen(msg.position, p_d);
    tf::quaternionMsgToEigen(msg.orientation, q_d);
}

void QuaternionPdController::compute()
{
    if (enabled)
    {
        updateProportionalGainMatrix();
        updateErrorVector();
        updateRestoringForceVector();

        tau = - K_D*nu - K_P*z + g;

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
}

void QuaternionPdController::updateErrorVector()
{
    Eigen::Vector3d    p_tilde = p - p_d;
    Eigen::Quaterniond q_tilde = q_d.conjugate()*q;

    z << p_tilde,
         sgn(q_tilde.w())*q_tilde.vec();
}

void QuaternionPdController::updateRestoringForceVector()
{
    // Should I assume an updated R, or force an update here?
    Eigen::Vector3d f_g = R.transpose() * Eigen::Vector3d(0, 0, W);
    Eigen::Vector3d f_b = R.transpose() * Eigen::Vector3d(0, 0, -B);

    g << f_g + f_b,
         r_g.cross(f_g) + r_b.cross(f_b);
}

int QuaternionPdController::sgn(double x)
{
    if (x < 0)
    {
        return -1;
    }
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
