#include "open_loop_controller.h"

OpenLoopController::OpenLoopController()
{
    enabled = false;
    sub = nh.subscribe("wrench_setpoints", 10, &OpenLoopController::callback, this);
    pub = nh.advertise<geometry_msgs::Wrench>("rov_forces", 10);
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