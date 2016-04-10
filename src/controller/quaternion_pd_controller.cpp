// Based on Fjellstad & Fossen 1994: Quaternion Feedback Regulation of Underwater Vehicles

#include "quaternion_pd_controller.h"

QuaternionPdController::QuaternionPdController()
{
    enabled = false;
    stateSub        = nh.subscribe("state", 1, &QuaternionPdController::stateCallback, this);
    setpointSub     = nh.subscribe("setpoint", 1, &QuaternionPdController::setpointCallback, this);
    controlInputPub = nh.advertise<geometry_msgs::Wrench>("control_input", 1);

    // Initial values
    p         << 0, 0, 0;
    q.w()     =  0;
    q.vec()   << 0, 0, 0;
    nu        << 0, 0, 0, 0, 0, 0;
    p_d       << 0, 0, 0;
    q_d.w()   =  0;
    q_d.vec() << 0, 0, 0;
    tau       << -1, -1, -1, -1, -1, -1;

    // Values from paper, only temporary
    K_D = Eigen::MatrixXd::Identity(6,6);
    K_p = 30*Eigen::MatrixXd::Identity(3,3);
    c = 200;
    r_g.setZero();
    r_b.setZero();
    W = 185*9.8;
    B = 185*9.8;
}

void QuaternionPdController::stateCallback(const uranus_dp::State &stateMsg)
{
    p << stateMsg.pose.position.x,
         stateMsg.pose.position.y,
         stateMsg.pose.position.z;

    q.w()   =  stateMsg.pose.orientation.x;
    q.vec() << stateMsg.pose.orientation.y,
               stateMsg.pose.orientation.z,
               stateMsg.pose.orientation.w;

    nu << stateMsg.twist.linear.x,
          stateMsg.twist.linear.y,
          stateMsg.twist.linear.z,
          stateMsg.twist.angular.x,
          stateMsg.twist.angular.y,
          stateMsg.twist.angular.z;

    // Update rotation matrix
    R = q.toRotationMatrix();
}

void QuaternionPdController::setpointCallback(const uranus_dp::State &setpointMsg)
{
    p_d << setpointMsg.pose.position.x,
           setpointMsg.pose.position.y,
           setpointMsg.pose.position.z;

    q_d.w()   =  setpointMsg.pose.orientation.x;
    q_d.vec() << setpointMsg.pose.orientation.y,
                 setpointMsg.pose.orientation.z,
                 setpointMsg.pose.orientation.w;
}

void QuaternionPdController::compute()
{
    if (enabled)
    {
        updateProportionalGainMatrix();
        updateErrorVector();
        updateRestoringForceVector();

        tau = - K_D * nu - K_P * z + g;

        // Publish tau
        geometry_msgs::Wrench tauMsg;
        tf::wrenchEigenToMsg(tau, tauMsg);
        // tauMsg.force.x  = tau(0);
        // tauMsg.force.y  = tau(1);
        // tauMsg.force.z  = tau(2);
        // tauMsg.torque.x = tau(3);
        // tauMsg.torque.y = tau(4);
        // tauMsg.torque.z = tau(5);
        controlInputPub.publish(tauMsg);
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

    z << p_tilde, sgn(q_tilde.w())*q_tilde.vec();
}

void QuaternionPdController::updateRestoringForceVector()
{
    // Should I assume an updated R, or force an update here?
    Eigen::Vector3d f_g = R.transpose() * Eigen::Vector3d(0, 0, W);
    Eigen::Vector3d f_b = R.transpose() * Eigen::Vector3d(0, 0, -B);

    g << f_g + f_b, r_g.cross(f_g) + r_b.cross(f_b);
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
