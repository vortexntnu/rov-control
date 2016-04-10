// See
// Fossen 2011, chapter 11.3.3
// and
// Dukan 2014, chapter 4.2.2

#include "extended_kalman_filter.h"

ExtendedKalmanFilter::ExtendedKalmanFilter(double sampleTime)
{
    h = sampleTime;

    Q.setIdentity();
    R.setIdentity();
    Gamma = h*E;
}

void ExtendedKalmanFilter::controlCallback(const geometry_msgs::Wrench &tauMsg)
{
    // The input to the model used for filter design is tau (control forces), not u (thruster forces),
    // but in the context of the filter, it makes sense to use the name u. Mmmmmkay?
    tf::wrenchMsgToEigen(tauMsg, u);
}

void ExtendedKalmanFilter::sensorCallback(const ros_arduino::SensorRaw &yMsg)
{
    y << yMsg.acceleration.x,
         yMsg.acceleration.y,
         yMsg.acceleration.z,
         yMsg.compass.x,
         yMsg.compass.y,
         yMsg.compass.z,
         yMsg.gyro.x,
         yMsg.gyro.y,
         yMsg.gyro.z,
         yMsg.pressure;
}

void ExtendedKalmanFilter::update()
{
    // Correction equations
    K = P_bar*H_transpose * (H*P_bar*H_transpose + R).inverse();
    x_hat = x_bar + K*(y - H*x_bar);
    P_hat = (I_13 - K*H)*P_bar*(I_13 - K*H).transpose() + K*R*K.transpose();

    // Update f and Phi
    updateSystemDynamics();
    updatePhi();

    // Prediction equation
    x_bar = x_hat + h*(f + B*u);
    P_bar = Phi*P_hat*Phi.transpose() + Gamma*Q*Gamma.transpose();

    // Publish
    uranus_dp::State xMsg;
    xMsg.pose.position.x    = x_bar(0);
    xMsg.pose.position.y    = x_bar(1);
    xMsg.pose.position.z    = x_bar(2);
    xMsg.pose.orientation.x = x_bar(3);
    xMsg.pose.orientation.y = x_bar(4);
    xMsg.pose.orientation.z = x_bar(5);
    xMsg.pose.orientation.w = x_bar(6);
    xMsg.twist.linear.x     = x_bar(7);
    xMsg.twist.linear.y     = x_bar(8);
    xMsg.twist.linear.z     = x_bar(9);
    xMsg.twist.angular.x    = x_bar(10);
    xMsg.twist.angular.y    = x_bar(11);
    xMsg.twist.angular.z    = x_bar(12);
    statePub.publish(xMsg);
}

void ExtendedKalmanFilter::updateSystemDynamics()
{
    // Copy attitude vector from current estimate
    Eigen::Quaterniond q;
    q.w() = x_hat(3);
    q.x() = x_hat(4);
    q.y() = x_hat(5);
    q.z() = x_hat(6);

    // Copy velocity vector from current estimate
    Eigen::Matrix<double,6,1> nu;
    nu << x_hat(7),
          x_hat(8),
          x_hat(9),
          x_hat(10),
          x_hat(11),
          x_hat(12);

    // Update matrices
    M = getM();
    C = getC();
    D = getD();
    g = getG();
    J = getJ(q);

    f << J*nu,
         -M.inverse() * (C*nu + D*nu + g);
}

void ExtendedKalmanFilter::updatePhi()
{
    Eigen::Matrix<double,13,13> del_f_del_x;
    // Evalute del_f_del_x at current x_hat
    Phi = I_13 + h*del_f_del_x;
}
