#ifndef VORTEX_CONTROLLER_CONTROLLER_ROS_H
#define VORTEX_CONTROLLER_CONTROLLER_ROS_H

#include <Eigen/Dense>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <dynamic_reconfigure/server.h>

#include <vortex_controller/VortexControllerConfig.h>
#include "vortex/eigen_typedefs.h"
#include "vortex_controller/control_modes.h"
#include "vortex_msgs/PropulsionCommand.h"

#include "vortex_controller/state.h"
#include "vortex_controller/setpoints.h"
#include "vortex_controller/quaternion_pd_controller.h"

class Controller
{
public:
  explicit Controller(ros::NodeHandle nh);
  void commandCallback(const vortex_msgs::PropulsionCommand &msg);
  void stateCallback(const nav_msgs::Odometry &msg);
  void configCallback(const vortex_controller::VortexControllerConfig& config, uint32_t level);
  void spin();
private:
  ros::NodeHandle nh;
  ros::Subscriber command_sub;
  ros::Subscriber state_sub;
  ros::Publisher  wrench_pub;
  ros::Publisher  mode_pub;
  dynamic_reconfigure::Server<vortex_controller::VortexControllerConfig> dr_srv;

  ControlMode control_mode;
  int frequency;
  const double NORMALIZED_FORCE_DEADZONE = 0.01;
  const double MAX_QUAT_NORM_DEVIATION = 0.1;

  const uint8_t WRENCH_SURGE = 0;
  const uint8_t WRENCH_SWAY  = 1;
  const uint8_t WRENCH_HEAVE = 2;
  const uint8_t WRENCH_ROLL  = 3;
  const uint8_t WRENCH_PITCH = 4;
  const uint8_t WRENCH_YAW   = 5;

  const uint8_t POSITION_SURGE = 0;
  const uint8_t POSITION_SWAY  = 1;
  const uint8_t POSITION_HEAVE = 2;

  const uint8_t EULER_YAW   = 0;
  const uint8_t EULER_PITCH = 1;
  const uint8_t EULER_ROLL  = 2;

  State                  *state;
  Setpoints              *setpoints;
  QuaternionPdController *controller;

  ControlMode getControlMode(const vortex_msgs::PropulsionCommand &msg) const;
  void initSetpoints();
  void resetSetpoints();
  void initPositionHoldController();
  bool healthyMessage(const vortex_msgs::PropulsionCommand &msg);
  void publishControlMode();
  Eigen::Vector6d stayLevel(const Eigen::Quaterniond &orientation_state,
                            const Eigen::Vector6d &velocity_state);
  Eigen::Vector6d depthHold(const Eigen::Vector6d &tau_openloop,
                            const Eigen::Vector3d &position_state,
                            const Eigen::Quaterniond &orientation_state,
                            const Eigen::Vector6d &velocity_state,
                            const Eigen::Vector3d &position_setpoint);
  Eigen::Vector6d headingHold(const Eigen::Vector6d &tau_openloop,
                              const Eigen::Vector3d &position_state,
                              const Eigen::Quaterniond &orientation_state,
                              const Eigen::Vector6d &velocity_state,
                              const Eigen::Quaterniond &orientation_setpoint);
};

#endif  // VORTEX_CONTROLLER_CONTROLLER_ROS_H
