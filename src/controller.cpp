#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "uranus_dp/State.h"

// NOTE: For now, I juse use Pose messages for state and setpoints, which is probably insufficient.

class Controller
{
public:
    Controller()
    {
        outputPub = n.advertise<geometry_msgs::Wrench>("outputTopic", 1);

        // I don't know why, but "this" is necessary here
        stateSub = n.subscribe("state", 1, &Controller::stateCallback, this);
        setpointSub = n.subscribe("setpoint", 1, &Controller::setpointCallback, this);
    }

    // stateCallback updates the private state variable when a new state message arrives.
    void stateCallback(const geometry_msgs::Pose &state_msg)
    {
        state = state_msg;
    }

    // setpointCallback updates the private setpoint variable when a new setpoint message arrives.
    void setpointCallback(const geometry_msgs::Pose &setpoint_msg)
    {
        setpoint = setpoint_msg;
    }

    // calculate contains the control algorithm, and calculates the control output based on the
    // current state and setpoint.
    void calculate(void)
    {
        geometry_msgs::Wrench output;

        // Put control algorithm here.

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
    geometry_msgs::Pose state;
    geometry_msgs::Pose setpoint;
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
