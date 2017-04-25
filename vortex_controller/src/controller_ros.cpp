#include "vortex_controller/controller_ros.h"

#include "vortex/eigen_helper.h"
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

#include "std_msgs/String.h"

#include <math.h>
#include <map>
#include <string>
#include <vector>

Controller::Controller(ros::NodeHandle nh) : nh(nh)
{
  command_sub = nh.subscribe("propulsion_command", 10, &Controller::commandCallback, this);
  state_sub   = nh.subscribe("state_estimate", 10, &Controller::stateCallback, this);
  wrench_pub  = nh.advertise<geometry_msgs::Wrench>("rov_forces", 10);
  mode_pub    = nh.advertise<std_msgs::String>("controller/mode", 10);

  control_mode = ControlModes::OPEN_LOOP;

  if (!nh.getParam("/controller/frequency", frequency))
  {
    ROS_WARN("Failed to read parameter controller frequency, defaulting to 10 Hz.");
    frequency = 10;
  }

  state = new State();
  initSetpoints();
  initPositionHoldController();

  // Set up a dynamic reconfigure server
  dynamic_reconfigure::Server<vortex_controller::VortexControllerConfig>::CallbackType cb;
  cb = boost::bind(&Controller::configCallback, this, _1, _2);
  dr_srv.setCallback(cb);

  ROS_INFO("Node initialized.");
}

void Controller::commandCallback(const vortex_msgs::PropulsionCommand& msg)
{
  if (!healthyMessage(msg))
  {
    ROS_WARN("Propulsion command message out of range, ignoring...");
    return;
  }

  ControlMode new_control_mode;
  {
    int i;
    for (i = 0; i < msg.control_mode.size(); ++i)
      if (msg.control_mode[i])
        break;
    new_control_mode = static_cast<ControlMode>(i);
  }

  if (new_control_mode != control_mode)
  {
    control_mode = new_control_mode;

    Eigen::Vector3d    position;
    Eigen::Quaterniond orientation;
    Eigen::Vector6d    velocity;

    // Reset setpoints to be equal to state
    state->get(&position, &orientation, &velocity);
    setpoints->set(position, orientation);

    ROS_INFO_STREAM("Changing mode to " << controlModeString(control_mode) << ".");
  }
  publishControlMode();

  double time = msg.header.stamp.toSec();
  Eigen::Vector6d command;
  for (int i = 0; i < 6; ++i)
    command(i) = msg.motion[i];
  setpoints->update(time, command);
}

void Controller::stateCallback(const nav_msgs::Odometry &msg)
{
  Eigen::Vector3d    position;
  Eigen::Quaterniond orientation;
  Eigen::Vector6d    velocity;

  tf::pointMsgToEigen(msg.pose.pose.position, position);
  tf::quaternionMsgToEigen(msg.pose.pose.orientation, orientation);
  tf::twistMsgToEigen(msg.twist.twist, velocity);

  const double MAX_QUAT_NORM_DEVIATION = 0.1;
  bool orientation_invalid = (abs(orientation.norm() - 1) > MAX_QUAT_NORM_DEVIATION);
  if (isFucked(position) || isFucked(velocity) || orientation_invalid)
  {
    ROS_WARN("Requested state not valid, ignoring...");
    return;
  }

  state->set(position, orientation, velocity);
}

void Controller::configCallback(const vortex_controller::VortexControllerConfig &config, uint32_t level)
{
  ROS_INFO_STREAM("Setting gains [a=" << config.a << ", b=" << config.b << ", c=" << config.c << "].");
  position_hold_controller->setGains(config.a, config.b, config.c);
}

void Controller::spin()
{
  Eigen::Vector6d    tau_command          = Eigen::VectorXd::Zero(6);
  Eigen::Vector6d    tau_openloop         = Eigen::VectorXd::Zero(6);
  Eigen::Vector6d    tau_sixdof           = Eigen::VectorXd::Zero(6);

  Eigen::Vector3d    position_state       = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_state    = Eigen::Quaterniond::Identity();
  Eigen::Vector6d    velocity_state       = Eigen::VectorXd::Zero(6);

  Eigen::Vector3d    position_setpoint    = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_setpoint = Eigen::Quaterniond::Identity();

  ros::Rate rate(frequency);
  while (ros::ok())
  {
    // TODO(mortenfyhn): check value of bool return from getters
    state->get(&position_state, &orientation_state, &velocity_state);
    setpoints->get(&position_setpoint, &orientation_setpoint);

    switch (control_mode)
    {
      case ControlModes::OPEN_LOOP:
      {
        setpoints->get(&tau_command);
        break;
      }

      case ControlModes::SIXDOF:
      {
        tau_command = position_hold_controller->compute(position_state,
                                                        orientation_state,
                                                        velocity_state,
                                                        position_setpoint,
                                                        orientation_setpoint);
        break;
      }

      case ControlModes::RPY_DEPTH:
      {
        // TODO(mortenfyhn): make this similar to depth hold
        tau_command(0) = tau_openloop(0);
        tau_command(1) = tau_openloop(1);
        tau_command(2) = tau_sixdof(2);
        tau_command(3) = tau_sixdof(3);
        tau_command(4) = tau_sixdof(4);
        tau_command(5) = tau_sixdof(5);
        break;
      }

      case ControlModes::DEPTH_HOLD:
      {
        setpoints->get(&tau_openloop);

        bool depth_change_commanded = abs(tau_openloop(2)) > FORCE_DEADZONE_LIMIT;
        if (depth_change_commanded)
        {
          tau_command = tau_openloop;
        }
        else
        {
          position_setpoint(0) = position_state(0);
          position_setpoint(1) = position_state(1);
          orientation_setpoint = orientation_state;
          tau_sixdof = position_hold_controller->compute(position_state,
                                                         orientation_state,
                                                         velocity_state,
                                                         position_setpoint,
                                                         orientation_setpoint);
          tau_command = tau_sixdof + tau_openloop;
        }
        break;
      }

      default:
      {
        ROS_ERROR("Default control mode reached.");
        break;
      }
    }

    geometry_msgs::Wrench msg;
    tf::wrenchEigenToMsg(tau_command, msg);
    wrench_pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }
}

void Controller::initSetpoints()
{
  std::vector<double> v;

  if (!nh.getParam("/propulsion/command/wrench/max", v))
    ROS_FATAL("Failed to read parameter max wrench command.");
  Eigen::Vector6d wrench_command_max = Eigen::Vector6d::Map(v.data(), v.size());

  if (!nh.getParam("/propulsion/command/wrench/scaling", v))
    ROS_FATAL("Failed to read parameter scaling wrench command.");
  Eigen::Vector6d wrench_command_scaling = Eigen::Vector6d::Map(v.data(), v.size());

  if (!nh.getParam("/propulsion/command/pose/rate", v))
    ROS_FATAL("Failed to read parameter pose command rate.");
  Eigen::Vector6d pose_command_rate = Eigen::Vector6d::Map(v.data(), v.size());

  setpoints = new Setpoints(wrench_command_scaling,
                            wrench_command_max,
                            pose_command_rate);
}

void Controller::initPositionHoldController()
{
  // Read controller gains from parameter server
  std::map<std::string, double> gains;
  if (!nh.getParam("/controller/gains", gains))
    ROS_FATAL("Failed to read parameter controller gains.");

  // Read center of gravity and buoyancy vectors
  std::vector<double> r_G_vec, r_B_vec;
  if (!nh.getParam("/physical/center_of_mass", r_G_vec))
    ROS_FATAL("Failed to read robot center of mass parameter.");
  if (!nh.getParam("/physical/center_of_buoyancy", r_B_vec))
    ROS_FATAL("Failed to read robot center of buoyancy parameter.");
  Eigen::Vector3d r_G(r_G_vec.data());
  Eigen::Vector3d r_B(r_B_vec.data());

  // Read and calculate ROV weight and buoyancy
  double mass, displacement, acceleration_of_gravity, density_of_water;
  if (!nh.getParam("/physical/mass_kg", mass))
    ROS_FATAL("Failed to read parameter mass.");
  if (!nh.getParam("/physical/displacement_m3", displacement))
    ROS_FATAL("Failed to read parameter displacement.");
  if (!nh.getParam("/gravity/acceleration", acceleration_of_gravity))
    ROS_FATAL("Failed to read parameter acceleration of gravity");
  if (!nh.getParam("/water/density", density_of_water))
    ROS_FATAL("Failed to read parameter density of water");
  double W = mass * acceleration_of_gravity;
  double B = density_of_water * displacement * acceleration_of_gravity;

  position_hold_controller = new QuaternionPdController(gains["a"],
                                                        gains["b"],
                                                        gains["c"],
                                                        W, B, r_G, r_B);
}

bool Controller::healthyMessage(const vortex_msgs::PropulsionCommand& msg)
{
  // Check that motion commands are in range
  for (int i = 0; i < msg.motion.size(); ++i)
  {
    if (msg.motion[i] > 1 || msg.motion[i] < -1)
    {
      ROS_WARN("Motion command out of range.");
      return false;
    }
  }

  // Check that exactly one control mode is requested
  int num_requested_modes = 0;
  for (int i = 0; i < msg.control_mode.size(); ++i)
    if (msg.control_mode[i])
      num_requested_modes++;

  if (num_requested_modes != 1)
  {
    ROS_WARN_STREAM("Invalid control mode. Attempt to set "
                    << num_requested_modes << " control modes at once.");
    return false;
  }

  return true;
}

void Controller::publishControlMode()
{
  std::string s = controlModeString(control_mode);
  std_msgs::String msg;
  msg.data = s;
  mode_pub.publish(msg);
}
