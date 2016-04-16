# Uranus DP
The Uranus dynamic positioning system, a complete control system for our ROV.

## Future ideas
- Would be cool to implement an adaptive controller.

## Dependencies
- Depends on the Eigen library which can be downloaded as the package libeigen3-dev with apt-get (or any other package manager).

## Todo
- Add values values to model node
- Get values r_g, r_b, W, B for Quaternion controller
- Normalize quaternions everywhere
- Split ExtendedKalmanFilter::updateSystemDynamics()
- Have a look at topic buffer sizes
- Logic for controller switching (zero position estimate etc.)
- Have a look at variable names for ros::Publisher/Subscribers
- Proper ROS parameter use
- Consider return by value for quaternion pd update functions
- Clean tests for controller (write nice method for service calling)