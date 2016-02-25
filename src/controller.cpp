#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"
#include "uranus_dp/State.h"

class Controller
{
public:
    Controller()
    {
        outputPub = n.advertise<geometry_msgs::Wrench>("outputTopic", 1);

        // I don't know why, but 'this' is necessary here
        stateSub = n.subscribe("stateTopic", 1, &Controller::stateCallback, this);
        setpointSub = n.subscribe("setpointTopic", 1, &Controller::setpointCallback, this);
    }

    // stateCallback updates the private state variables when a new state message arrives.
    void stateCallback(const uranus_dp::State &state_msg)
    {
        // Copy pose values (position and orientation)
        eta[0] = state_msg.pose.position.x;
        eta[1] = state_msg.pose.position.y;
        eta[2] = state_msg.pose.position.z;
        eta[3] = state_msg.pose.orientation.x;
        eta[4] = state_msg.pose.orientation.y;
        eta[5] = state_msg.pose.orientation.z;
        eta[6] = state_msg.pose.orientation.w;

        // Copy twist values (linear and angular velocity)
        nu[0] = state_msg.twist.linear.x;
        nu[1] = state_msg.twist.linear.y;
        nu[2] = state_msg.twist.linear.z;
        nu[3] = state_msg.twist.angular.x;
        nu[4] = state_msg.twist.angular.y;
        nu[5] = state_msg.twist.angular.z;
    }

    // setpointCallback updates the private setpoint variable when a new setpoint message arrives.
    void setpointCallback(const geometry_msgs::Twist &setpoint_msg)
    {
        // Copy velocity setpoint values
        nu_sp[0] = setpoint_msg.linear.x;
        nu_sp[1] = setpoint_msg.linear.y;
        nu_sp[2] = setpoint_msg.linear.z;
        nu_sp[3] = setpoint_msg.angular.x;
        nu_sp[4] = setpoint_msg.angular.y;
        nu_sp[5] = setpoint_msg.angular.z;
    }

    // calculate contains the control algorithm, and calculates the control output based on the
    // current state and setpoint.
    void calculate(void)
    {
        // Only velocity control for now.

        // geometry_msgs::Twist velocityError = setpoint - state.velocity;

        geometry_msgs::Wrench output;
        output.force.x = 0;
        output.force.y = 0;
        output.force.z = 0;
        output.torque.x = 0;
        output.torque.y = 0;
        output.torque.z = 0;

        outputPub.publish(output);
    }
private:
    ros::NodeHandle n;

    ros::Publisher  outputPub;
    ros::Subscriber stateSub;
    ros::Subscriber setpointSub;

    // State as pose eta = [p q], and twist nu = [v omega]
    double eta [7];
    double nu [6];

    // Velocity setpoints nu_sp = [v_sp omega_sp]
    double nu_sp [6];
}; // End of class Controller

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");

    Controller controllerObject;

    // Run the controller at 10 Hz
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        controllerObject.calculate();

        loop_rate.sleep();
    }

    return 0;
}
