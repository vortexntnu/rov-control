#include "open_loop_controller.h"

OpenLoopController::OpenLoopController()
{
    sub = nh.subscribe("open_loop_setpoint", 1, &OpenLoopController::callback, this);
    pub = nh.advertise<geometry_msgs::Wrench>("control_input", 1);
}

void OpenLoopController::callback(const geometry_msgs::Wrench &msg)
{
    pub.publish(msg);
}
