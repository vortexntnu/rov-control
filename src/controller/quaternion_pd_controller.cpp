// Based on Fjellstad & Fossen 1994: Quaternion Feedback Regulation of Underwater Vehicles

#include "quaternion_pd_controller.h"

// void Print

QuaternionPdController::QuaternionPdController()
{
    stateSub    = nh.subscribe("state_estimate", 10, &QuaternionPdController::stateCallback, this);
    setpointSub = nh.subscribe("pose_setpoints", 10, &QuaternionPdController::setpointCallback, this);
    controlPub  = nh.advertise<geometry_msgs::Wrench>("rov_forces", 10);
    enabled = false;

    // Initialize all values to zero/identity
    p.setZero();
    q.setIdentity();
    nu.setZero();
    p_d.setZero();
    q_d.setIdentity();
    z.setZero();
    tau.setZero();
    g.setZero();
    R = q.toRotationMatrix();

    // Gains etc. (temporary values from paper)
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
    // ROS_INFO("quaternion_pd_controller: stateCallback running.");

    tf::pointMsgToEigen(msg.pose.position, p);
    tf::quaternionMsgToEigen(msg.pose.orientation, q);
    tf::twistMsgToEigen(msg.twist, nu);

    // ROS_INFO_STREAM("p set to [" << p(0) << ", " << p(1) << ", " << p(2) << "].");
    // ROS_INFO_STREAM("q set to [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "].");
    // ROS_INFO_STREAM("nu set to [" << nu(0) << ", " << nu(1) << ", " << nu(2) << ", " << nu(3) << ", " << nu(4) << ", " << nu(5) << "].");

    R = q.toRotationMatrix();
    // ROS_INFO_STREAM("R = [" << R(0,0) << ", " << R(0,1) << ", " << R(0,2) << "]");
    // ROS_INFO_STREAM("R = [" << R(1,0) << ", " << R(1,1) << ", " << R(1,2) << "]");
    // ROS_INFO_STREAM("R = [" << R(2,0) << ", " << R(2,1) << ", " << R(2,2) << "]");
}

void QuaternionPdController::setpointCallback(const geometry_msgs::Pose &msg)
{
    // ROS_INFO("quaternion_pd_controller: setpointCallback running.");

    tf::pointMsgToEigen(msg.position, p_d);
    tf::quaternionMsgToEigen(msg.orientation, q_d);

    // ROS_INFO_STREAM("p_d set to [" << p_d(0) << ", " << p_d(1) << ", " << p_d(2) << "].");
    // ROS_INFO_STREAM("q_d set to [" << q_d.w() << ", " << q_d.x() << ", " << q_d.y() << ", " << q_d.z() << "].");
}

void QuaternionPdController::compute()
{
    if (enabled)
    {
        updateProportionalGainMatrix();
        updateErrorVector();
        updateRestoringForceVector();

        tau = - K_D*nu - K_P*z + g;
        // ROS_INFO_STREAM("compute(): nu = [" << nu(0) << ", " << nu(1) << ", " << nu(2) << ", " << nu(3) << ", " << nu(4) << ", " << nu(5) << "].");
        // ROS_INFO_STREAM("compute(): z = [" << z(0) << ", " << z(1) << ", " << z(2) << ", " << z(3) << ", " << z(4) << ", " << z(5) << "].");
        // ROS_INFO_STREAM("compute(): g = [" << g(0) << ", " << g(1) << ", " << g(2) << "].");
        // ROS_INFO_STREAM("tau =  [" << tau(0) << ", " << tau(1) << ", " << tau(2) << ", " << tau(3) << ", " << tau(4) << ", " << tau(5) << "].");

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

    // ROS_INFO_STREAM("updateRestoringForceVector(): f_g = [" << f_g(0) << ", " << f_g(1) << ", " << f_g(2) << "].");
    // ROS_INFO_STREAM("updateRestoringForceVector(): f_b = [" << f_b(0) << ", " << f_b(1) << ", " << f_b(2) << "].");

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
