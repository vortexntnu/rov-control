// See Fossen 2011, chapter 12.3.2
//
// Explanation of variables:
//  u   - control input vector (6)
//  K   - thrust coefficient matrix (6x6)
//  T   - thrust configuration matrix (6x6?)
//  tau - vector of forces on the ROV (6)
//
// Mathematical relationships:
//  f = K*u are the forces of each actuator in Newtons
//  tau = T*f are the forces on the ROV in Newtons and Newton meters

#include "ros/ros.h"
#include "lagrange_allocator_unweighted.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "allocation");
    LagrangeAllocatorUnweighted allocator;
    ros::spin();
    return 0;
}
