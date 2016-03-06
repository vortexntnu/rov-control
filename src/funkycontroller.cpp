#include "funkycontroller.h"

NonlinearQuaternionPidController::NonlinearQuaternionPidController()
{
    outputPub   = n.advertise<geometry_msgs::Wrench>("outputTopic", 1);
    stateSub    = n.subscribe("stateTopic", 1, &NonlinearQuaternionPidController::stateCallback, this);
    setpointSub = n.subscribe("setpointTopic", 1, &NonlinearQuaternionPidController::setpointCallback, this);

    // Set default frequency
    frequency = 10;

    // Allocate dynamic member variables
    T   = Eigen::MatrixXd(4,3);
    tau = Eigen::VectorXd(6);

    // Initialize controller gains
    K_lin_p = Eigen::Matrix3d::Identity();
    K_lin_d = Eigen::Matrix3d::Identity();
    K_ang_p = Eigen::Matrix3d::Identity();
    K_ang_d = Eigen::Matrix3d::Identity();
}

void NonlinearQuaternionPidController::stateCallback(const uranus_dp::State &state_msg)
{
    // Copy position values
    p(0) = state_msg.pose.position.x;
    p(1) = state_msg.pose.position.y;
    p(2) = state_msg.pose.position.z;

    // Copy orientation values (quaternion)
    q(0) = state_msg.pose.orientation.x;
    q(1) = state_msg.pose.orientation.y;
    q(2) = state_msg.pose.orientation.z;
    q(3) = state_msg.pose.orientation.w;

    // Copy linear velocity values
    v(0) = state_msg.twist.linear.x;
    v(1) = state_msg.twist.linear.y;
    v(2) = state_msg.twist.linear.z;

    // Copy angular velocity values
    omega(0) = state_msg.twist.angular.x;
    omega(1) = state_msg.twist.angular.y;
    omega(2) = state_msg.twist.angular.z;
}

void NonlinearQuaternionPidController::setpointCallback(const geometry_msgs::Twist &setpoint_msg)
{
    // Copy linear velocity setpoint values
    v_sp(0) = setpoint_msg.linear.x;
    v_sp(1) = setpoint_msg.linear.y;
    v_sp(2) = setpoint_msg.linear.z;

    // Copy angular velocity setpoint values
    omega_sp(0) = setpoint_msg.angular.x;
    omega_sp(1) = setpoint_msg.angular.y;
    omega_sp(2) = setpoint_msg.angular.z;
}

void NonlinearQuaternionPidController::compute(void)
{
    // Only pose control for now

    // Position and orientation errors
    p_err = p_sp - p;
    q_err = q_sp - q;

    // Control output
    updateTransformationMatrices();
    Eigen::Vector3d tau_lin = K_lin_p * p_err + K_lin_d * v;
    Eigen::Vector3d tau_ang = - K_ang_p * T.transpose() * q_err - K_ang_d * omega;
    tau << tau_lin, tau_ang;

    // Create and send output message
    geometry_msgs::Wrench output;
    output.force.x  = tau(0);
    output.force.y  = tau(1);
    output.force.z  = tau(2);
    output.torque.x = tau(3);
    output.torque.y = tau(4);
    output.torque.z = tau(5);
    outputPub.publish(output);
}

void NonlinearQuaternionPidController::setFrequency(unsigned int newFrequency)
{
    frequency = newFrequency;
}

void NonlinearQuaternionPidController::updateTransformationMatrices(void)
{
    // Linear velocity transformation matrix
    R(0,0) = 1 - 2*(pow(q(2),2) + pow(q(3),2));
    R(0,1) = 2*(q(1)*q(2) - q(3)*q(0));
    R(0,2) = 2*(q(1)*q(3) - q(2)*q(0));

    R(1,0) = 2*(q(1)*q(2) + q(3)*q(0));
    R(1,1) = 1 - 2*(pow(q(1),2) + pow(q(0),2));
    R(1,2) = 2*(q(2)*q(3) + q(1)*q(0));

    R(2,0) = 2*(q(1)*q(3) + q(2)*q(0));
    R(2,1) = 2*(q(2)*q(3) + q(1)*q(0));
    R(2,2) = 1 - 2*(pow(q(1),2) + pow(q(2),2));

    // Angular velocity transformation matrix
    T(0,0) = -q(1);
    T(0,1) = -q(2);
    T(0,2) = -q(3);

    T(1,0) =  q(0);
    T(1,1) = -q(3);
    T(1,2) =  q(2);

    T(2,0) =  q(3);
    T(2,1) =  q(3);
    T(2,2) = -q(1);

    T(3,0) = -q(2);
    T(3,1) =  q(1);
    T(3,2) =  q(0);

    T *= 0.5;
}
