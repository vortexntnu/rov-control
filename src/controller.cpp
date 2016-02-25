#include "ros/ros.h"
#include "ros/topic.h"
#include "geometry_msgs/Wrench.h"

#if 0
class Controller
{
public:
    Controller()
    {

    }
private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::
}

// Node-global state variable
geometry_msgs::Pose state;

void stateCallback(const geometry_msgs::Pose& state)
{

}

#endif

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");

    ros::NodeHandle n;

    // Subscribe to setpoints and state
    std::string stateTopicName = "stateTopic";
    // ros::Subscriber sub = n.subscribe<geometry_msgs::Pose>(stateTopicName, 1, stateCallback);

    // Publish a Wrench message for commanded forces and moments
    ros::Publisher pub = n.advertise<geometry_msgs::Wrench>("tau", 1);

    // Run the controller at 10 Hz
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // state = ros::topic::waitForMessage<geometry_msgs::Pose>(stateTopicName);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
