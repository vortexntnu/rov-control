#include "ros/ros.h"
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "uranus_dp/ResetIntegrationFilter.h"
#include "../eigen_typedefs.h"


class IntegrationFilter
{
public:
    IntegrationFilter(double frequency)
    {
        sub = nh.subscribe("sensor_raw", 10, &IntegrationFilter::callback, this);
        pub = nh.advertise<nav_msgs::Odometry>("state_estimate", 10);

        dt = 1/frequency;
        ROS_INFO_STREAM("integration_filter: Setting time step to " << dt);

        a_imu << 0, 0, 9.80665;
        w_imu.setZero();
        p.setZero();
        q.setIdentity();
        v.setZero();
        w.setZero();
        v_dot.setZero();
        q_dot.setZero();
        g_n << 0, 0, 9.80665;
    }

    bool reset(uranus_dp::ResetIntegrationFilter::Request &req, uranus_dp::ResetIntegrationFilter::Response &resp)
    {
        ROS_INFO("integration_filter: Resetting...");
        a_imu << 0, 0, 9.80665;
        w_imu.setZero();
        p.setZero();
        q.setIdentity();
        v.setZero();
        w.setZero();
        v_dot.setZero();
        q_dot.setZero();
        g_n << 0, 0, 9.80665;
        return true;
    }

    void callback(const sensor_msgs::Imu& msg)
    {
        tf::vectorMsgToEigen(msg.linear_acceleration, a_imu);
        tf::vectorMsgToEigen(msg.angular_velocity, w_imu);
        // ROS_INFO("Callback!");
        // ROS_INFO_STREAM("a_imu = [" << a_imu(0) << ", " << a_imu(1) << ", " << a_imu(2) << "]");
        // ROS_INFO_STREAM("w_imu = [" << w_imu(0) << ", " << w_imu(1) << ", " << w_imu(2) << "]");
    }

    void update()
    {
        v_dot = q.toRotationMatrix()*a_imu - g_n;
        // ROS_INFO_STREAM("v_dot = [" << v_dot(0) << ", " << v_dot(1) << ", " << v_dot(2) << "]");
        v = v + dt*v_dot;
        // ROS_INFO_STREAM("v = [" << v(0) << ", " << v(1) << ", " << v(2) << "]");
        p = p + dt*v;
        // ROS_INFO_STREAM("p = [" << p(0) << ", " << p(1) << ", " << p(2) << "]");

        // ROS_INFO_STREAM("q = [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]");

        // Eigen::MatrixXd T = Tmatrix();
        // ROS_INFO_STREAM("T = [" << T(0,0) << ", " << T(0,1) << ", " << T(0,2) << "]");
        // ROS_INFO_STREAM("    [" << T(1,0) << ", " << T(1,1) << ", " << T(1,2) << "]");
        // ROS_INFO_STREAM("    [" << T(2,0) << ", " << T(2,1) << ", " << T(2,2) << "]");
        // ROS_INFO_STREAM("    [" << T(3,0) << ", " << T(3,1) << ", " << T(3,2) << "]");

        q_dot = Tmatrix()*w_imu;
        // ROS_INFO_STREAM("q_dot = [" << q_dot(0) << ", " << q_dot(1) << ", " << q_dot(2) << ", " << q_dot(3) << "]");
        
        Eigen::Vector4d q_vector;
        q_vector << q.w(), q.vec();
        
        q_vector = q_vector + dt*q_dot;
        
        q.w()    = q_vector(0);
        q.vec() << q_vector(1), q_vector(2), q_vector(3);
        q.normalize();

        w = w_imu;

        nav_msgs::Odometry msg;
        tf::pointEigenToMsg(p, msg.pose.pose.position);
        tf::quaternionEigenToMsg(q, msg.pose.pose.orientation);
        tf::vectorEigenToMsg(v, msg.twist.twist.linear);
        tf::vectorEigenToMsg(w, msg.twist.twist.angular);
        pub.publish(msg);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher  pub;

    // Time step
    double dt;

    // Measurements
    Eigen::Vector3d a_imu; // Accelerometer
    Eigen::Vector3d w_imu; // Gyro

    // State estimates
    Eigen::Vector3d    p;
    Eigen::Quaterniond q;
    Eigen::Vector3d    v;
    Eigen::Vector3d    w;

    // Intermediate variables
    Eigen::Vector3d v_dot;
    Eigen::Vector4d q_dot;

    // Gravity vector in NED frame
    Eigen::Vector3d g_n;

    Eigen::Matrix<double,4,3> Tmatrix()
    {
        Eigen::Matrix<double,4,3> T;
        T << -q.vec().transpose(),
            q.w()*Eigen::MatrixXd::Identity(3,3) + skew(q.vec());
        return 0.5*T;
    }

    Eigen::Matrix3d skew(const Eigen::Vector3d &v)
    {
        Eigen::Matrix3d S;
        S <<  0   , -v(2),  v(1),
              v(2),  0,    -v(0),
             -v(1),  v(0),  0   ;
        return S;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "integration_filter");
    ROS_INFO("Launching node integration_filter.");
    ros::NodeHandle nh;


    double frequency = 100;
    IntegrationFilter filter(frequency);

    ros::ServiceServer ss = nh.advertiseService("reset_integration_filter", &IntegrationFilter::reset, &filter);
    
    ros::Rate rate(frequency);
    while (ros::ok())
    {
        ros::spinOnce();
        filter.update();
        rate.sleep();
    }
}