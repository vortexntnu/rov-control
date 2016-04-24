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
    setGains(1,30,200);
    r_g.setZero();
    r_b.setZero();
    W = 185*9.8;
    B = 185*9.8;
}

void QuaternionPdController::stateCallback(const nav_msgs::Odometry &msg)
{
    tf::pointMsgToEigen(msg.pose.pose.position, p);
    tf::quaternionMsgToEigen(msg.pose.pose.orientation, q);
    tf::twistMsgToEigen(msg.twist.twist, nu);
    R = q.toRotationMatrix();
    if (isFucked(p) || isFucked(nu) || isFucked(R))
        ROS_WARN("p, nu, or R is fucked.");
}

void QuaternionPdController::setpointCallback(const geometry_msgs::Pose &msg)
{
    tf::pointMsgToEigen(msg.position, p_d);
    tf::quaternionMsgToEigen(msg.orientation, q_d);
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
    Eigen::Vector3d    p_tilde = p - p_d;
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



// ROS_INFO_STREAM("p set to [" << p(0) << ", " << p(1) << ", " << p(2) << "].");
// ROS_INFO_STREAM("q set to [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "].");
// ROS_INFO_STREAM("nu set to [" << nu(0) << ", " << nu(1) << ", " << nu(2) << ", " << nu(3) << ", " << nu(4) << ", " << nu(5) << "].");

// ROS_INFO_STREAM("updateRestoringForceVector(): f_g = [" << f_g(0) << ", " << f_g(1) << ", " << f_g(2) << "].");
// ROS_INFO_STREAM("updateRestoringForceVector(): f_b = [" << f_b(0) << ", " << f_b(1) << ", " << f_b(2) << "].");

// ROS_INFO_STREAM("compute(): nu = [" << nu(0) << ", " << nu(1) << ", " << nu(2) << ", " << nu(3) << ", " << nu(4) << ", " << nu(5) << "].");
// ROS_INFO_STREAM("compute(): z = [" << z(0) << ", " << z(1) << ", " << z(2) << ", " << z(3) << ", " << z(4) << ", " << z(5) << "].");
// ROS_INFO_STREAM("compute(): g = [" << g(0) << ", " << g(1) << ", " << g(2) << "].");
// ROS_INFO_STREAM("tau =  [" << tau(0) << ", " << tau(1) << ", " << tau(2) << ", " << tau(3) << ", " << tau(4) << ", " << tau(5) << "].");

// ROS_INFO_STREAM("p_d set to [" << p_d(0) << ", " << p_d(1) << ", " << p_d(2) << "].");
// ROS_INFO_STREAM("q_d set to [" << q_d.w() << ", " << q_d.x() << ", " << q_d.y() << ", " << q_d.z() << "].");
