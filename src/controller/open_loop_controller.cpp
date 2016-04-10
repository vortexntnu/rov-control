#include "open_loop_controller.h"

OpenLoopController::OpenLoopController()
{
    enabled = false;
    sub = nh.subscribe("open_loop_setpoint", 1, &OpenLoopController::callback, this);
    pub = nh.advertise<geometry_msgs::Wrench>("control_input", 1);
}

void OpenLoopController::callback(const geometry_msgs::Wrench &msg)
{
    if (enabled)
    {
        pub.publish(msg);
    }
}

void OpenLoopController::enable()
{
    enabled = true;
}

void OpenLoopController::disable()
{
    enabled = false;
}