#include "integration_filter.h"

IntegrationFilter::IntegrationFilter(double frequency)
{
    sub = nh.subscribe("sensor_raw", 10, &IntegrationFilter::callback, this);
    pub = nh.advertise<nav_msgs::Odometry>("state_estimate", 10);

    dt = 1/frequency;
    ROS_INFO_STREAM("integration_filter: Setting time step to " << dt);

    a_imu << 0, 0, 9.80665;
    w_imu.setZero();
    q_imu.setIdentity();
    p.setZero();
    v.setZero();
    v_dot.setZero();

    messages_received = 0;
    is_calibrated = false;

    // while (messages_received < 100)
    //     ros::spinOnce();

    // calibrate();
}

bool IntegrationFilter::reset(uranus_dp::ResetStateEstimator::Request &req, uranus_dp::ResetStateEstimator::Response &resp)
{
    ROS_INFO("integration_filter: Resetting...");
    a_imu << 0, 0, 9.80665;
    w_imu.setZero();
    q_imu.setIdentity();
    p.setZero();
    v.setZero();
    v_dot.setZero();
    return true;
}

void IntegrationFilter::calibrate()
{
    ROS_INFO("integration_filter: Calibrating...");
    // read acceleration a few times and average
    // set g_n equal to it (or with axes fixed)
    g_n << a_imu;
    is_calibrated = true;
}

void IntegrationFilter::callback(const sensor_msgs::Imu& msg)
{
    tf::quaternionMsgToEigen(msg.orientation, q_imu);
    tf::vectorMsgToEigen(msg.linear_acceleration, a_imu);
    tf::vectorMsgToEigen(msg.angular_velocity, w_imu);
    messages_received++;
    
    // ROS_INFO("Callback!");
    // ROS_INFO_STREAM("a_imu = [" << a_imu(0) << ", " << a_imu(1) << ", " << a_imu(2) << "]");
    // ROS_INFO_STREAM("w_imu = [" << w_imu(0) << ", " << w_imu(1) << ", " << w_imu(2) << "]");
}

void IntegrationFilter::update()
{
    ROS_INFO("integration_filter: Updating...");
    v_dot = q_imu.toRotationMatrix()*a_imu - g_n;
    v = v + dt*v_dot;
    p = p + dt*v;

    nav_msgs::Odometry msg;
    tf::pointEigenToMsg(p, msg.pose.pose.position);
    tf::quaternionEigenToMsg(q_imu, msg.pose.pose.orientation);
    tf::vectorEigenToMsg(v, msg.twist.twist.linear);
    tf::vectorEigenToMsg(w_imu, msg.twist.twist.angular);
    pub.publish(msg);

    // ROS_INFO_STREAM("v_dot = [" << v_dot(0) << ", " << v_dot(1) << ", " << v_dot(2) << "]");
    // ROS_INFO_STREAM("v = [" << v(0) << ", " << v(1) << ", " << v(2) << "]");
    // ROS_INFO_STREAM("p = [" << p(0) << ", " << p(1) << ", " << p(2) << "]");
    // ROS_INFO_STREAM("q_imu = [" << q_imu.w() << ", " << q_imu.x() << ", " << q_imu.y() << ", " << q_imu.z() << "]");
    // Eigen::MatrixXd T = Tmatrix();
    // ROS_INFO_STREAM("T = [" << T(0,0) << ", " << T(0,1) << ", " << T(0,2) << "]");
    // ROS_INFO_STREAM("    [" << T(1,0) << ", " << T(1,1) << ", " << T(1,2) << "]");
    // ROS_INFO_STREAM("    [" << T(2,0) << ", " << T(2,1) << ", " << T(2,2) << "]");
    // ROS_INFO_STREAM("    [" << T(3,0) << ", " << T(3,1) << ", " << T(3,2) << "]");
    // ROS_INFO_STREAM("q_dot = [" << q_dot(0) << ", " << q_dot(1) << ", " << q_dot(2) << ", " << q_dot(3) << "]");
}

Eigen::Matrix<double,4,3> IntegrationFilter::Tmatrix()
{
    Eigen::Matrix<double,4,3> T;
    T << -q_imu.vec().transpose(),
    q_imu.w()*Eigen::MatrixXd::Identity(3,3) + skew(q_imu.vec());
    return 0.5*T;
}

Eigen::Matrix3d IntegrationFilter::skew(const Eigen::Vector3d &v)
{
    // return Eigen::Matrix3d(0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0);
    Eigen::Matrix3d S;
    S <<  0   , -v(2),  v(1),
          v(2),  0   , -v(0),
         -v(1),  v(0),  0   ;
    return S;
}
