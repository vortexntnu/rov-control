# Uranus DP
The Uranus dynamic positioning system, a complete control system for our ROV.

## Future ideas
- Would be cool to implement an adaptive controller.

## Dependencies
- Depends on the Eigen library which can be downloaded as the package libeigen3-dev with apt-get (or any other package manager).

## Todo
- Add values values to model node
- Implement del_f_del_x in EKF
- Get values r_g, r_b, W, B for Quaternion controller
- Normalize quaternions everywhere
- Split ExtendedKalmanFilter::updateSystemDynamics()
- Have a look at topic buffer sizes