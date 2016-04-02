// See
// Fossen 2011, chapter 11.3.3
// and
// Dukan 2014, chapter 4.2.2

#include "extended_kalman_filter.h"

ExtendedKalmanFilter::ExtendedKalmanFilter(double sampleTime)
{
    clientM = nh.serviceClient<uranus_dp::GetM>("get_m");
    clientC = nh.serviceClient<uranus_dp::GetC>("get_c");
    clientD = nh.serviceClient<uranus_dp::GetD>("get_d");
    clientG = nh.serviceClient<uranus_dp::GetG>("get_g");
    clientJ = nh.serviceClient<uranus_dp::GetJ>("get_j");

    h = sampleTime;

    Q.setIdentity();
    R.setIdentity();
    Gamma = h*E;
}

void ExtendedKalmanFilter::controlCallback(const geometry_msgs::Wrench &tauMsg)
{
    // The input to the model used for filter design is tau (control forces), not u (thruster forces),
    // but in the context of the filter, it makes sense to use the name u. Mmmmmkay?
    u << tauMsg.force.x,
         tauMsg.force.y,
         tauMsg.force.z,
         tauMsg.torque.x,
         tauMsg.torque.y,
         tauMsg.torque.z;
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

    // Update J matrix
    uranus_dp::GetJ srvJ;
    tf::quaternionEigenToMsg(q, srvJ.request.q);
    if (clientJ.call(srvJ))
    {
        for (int i = 0; i < 42; i++)
        {
            J(i) = srvJ.response.J[i];
        }
    }
    else
    {
        ROS_ERROR("Failed to call service get_j");
        return;
    }

    // Update M matrix
    uranus_dp::GetM srvM;
    if (clientM.call(srvM))
    {
        for (int i = 0; i < 36; i++)
        {
            M(i) = srvM.response.M[i];
        }
    }
    else
    {
        ROS_ERROR("Failed to call service get_m");
        return;
    }

    // Update C matrix
    uranus_dp::GetC srvC;
    tf::twistEigenToMsg(nu, srvC.request.nu);
    if (clientC.call(srvC))
    {
        for (int i = 0; i < 36; i++)
        {
            C(i) = srvC.response.C[i];
        }
    }
    else
    {
        ROS_ERROR("Failed to call service get_c");
        return;
    }

    // Update D matrix
    uranus_dp::GetD srvD;
    tf::twistEigenToMsg(nu, srvD.request.nu);
    if (clientD.call(srvD))
    {
        for (int i = 0; i < 36; i++)
        {
            D(i) = srvD.response.D[i];
        }
    }
    else
    {
        ROS_ERROR("Failed to call service get_d");
        return;
    }

    // Update g vector
    uranus_dp::GetG srvG;
    tf::quaternionEigenToMsg(q, srvG.request.q);
    if (clientG.call(srvG))
    {
        for (int i = 0; i < 6; i++)
        {
            g(i) = srvG.response.g[i];
        }
    }
    else
    {
        ROS_ERROR("Failed to call service get_g");
        return;
    }

    f << J*nu,
         -M.inverse() * (C*nu + D*nu + g);
}

void ExtendedKalmanFilter::updatePhi()
{
    Eigen::Matrix<double,13,13> del_f_del_x;
    // Evalute del_f_del_x at current x_hat
    Phi = I_13 + h*del_f_del_x;
}
