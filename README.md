# Uranus DP
The Uranus dynamic positioning system, a complete control system for our ROV.

## Future ideas
- Would be cool to implement an adaptive controller.
- Check if the Eigen conversion library is any good.

## Dependencies
- Depends on the Eigen library which can be downloaded as the package libeigen3-dev with apt-get (or any other package manager).

## Todo
*[ ] Implement model node (with get-services for model matrices) 
*[ ] Write sensorCallback for EKF
*[ ] Implement del_f_del_x in EKF
*[ ] Get values for K and T for Lagrange allocator
*[ ] Get values r_g, r_b, W, B for Quaternion controller
*[ ] Move skew function out of Quaternion controller class?
