#include "controller_ros.h"

#include "uranus_dp/eigen_helper.h"
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

#include <math.h>

Controller::Controller(ros::NodeHandle nh) : nh(nh)
{
  command_sub = nh.subscribe("propulsion_command", 10, &Controller::commandCallback, this);
  state_sub   = nh.subscribe("state_estimate", 10, &Controller::stateCallback, this);
  wrench_pub  = nh.advertise<geometry_msgs::Wrench>("rov_forces", 10);

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
  dynamic_reconfigure::Server<uranus_dp::ControllerConfig>::CallbackType cb;
  cb = boost::bind(&Controller::configCallback, this, _1, _2);
  dr_srv.setCallback(cb);

  ROS_INFO("Controller: Initialized with dynamic reconfigure.");
}

void Controller::commandCallback(const vortex_msgs::PropulsionCommand& msg)
{
  if (!healthyMessage(msg))
  {
    ROS_WARN("Controller: Propulsion command message out of range, ignoring...");
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
    switch (control_mode)
    {
      case ControlModes::OPEN_LOOP:
      ROS_INFO("Controller: Changing mode to OPEN LOOP.");
      break;

      case ControlModes::POSITION_HOLD:
      ROS_INFO("Controller: Changing mode to POSITION HOLD.");
      Eigen::Vector3d    position;
      Eigen::Quaterniond orientation;
      Eigen::Vector6d    velocity;
      state->get(position, orientation, velocity);
      setpoints->setPose(position, orientation);
      break;
    }
  }

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
  // tf::twistMsgToEigen(msg.twist.twist, velocity);
  velocity.setZero();

  const double MAX_QUAT_NORM_DEVIATION = 0.1;
  bool orientation_invalid = (abs(orientation.norm() - 1) > MAX_QUAT_NORM_DEVIATION);
  if (isFucked(position) || isFucked(velocity) || orientation_invalid)
  {
    ROS_WARN("Controller: State not valid, ignoring...");
    return;
  }

  state->set(position, orientation, velocity);
}

void Controller::configCallback(uranus_dp::ControllerConfig &config, uint32_t level)
{
  ROS_INFO_STREAM("Setting quat pd gains:   a = " << config.a << ",   b = " << config.b << ",   c = " << config.c);
  position_hold_controller->setGains(config.a, config.b, config.c);
}

void Controller::spin()
{
  Eigen::Vector6d    tau                  = Eigen::VectorXd::Zero(6);
  Eigen::Vector3d    position_state       = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_state    = Eigen::Quaterniond::Identity();
  Eigen::Vector6d    velocity_state       = Eigen::VectorXd::Zero(6);
  Eigen::Vector3d    position_setpoint    = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_setpoint = Eigen::Quaterniond::Identity();

  ros::Rate rate(frequency);
  while (ros::ok())
  {
    switch (control_mode)
    {
      case ControlModes::POSITION_HOLD:
      state->get(position_state, orientation_state, velocity_state);
      setpoints->getPose(position_setpoint, orientation_setpoint);
      tau = position_hold_controller->compute(position_state,
                                              orientation_state,
                                              velocity_state,
                                              position_setpoint,
                                              orientation_setpoint);
      break;

      case ControlModes::OPEN_LOOP:
      setpoints->getWrench(tau);
      break;

      default:
      ROS_ERROR("Default control mode reached in Controller::spin().");
      break;
    }

    geometry_msgs::Wrench msg;
    tf::wrenchEigenToMsg(tau, msg);
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
  std::map<std::string,double> gains;
  if (!nh.getParam("/controller/gains", gains))
    ROS_ERROR("Failed to read parameter controller gains.");

  // Read center of gravity and buoyancy vectors
  std::vector<double> r_G_vec, r_B_vec;
  if (!nh.getParam("/physical/center_of_mass", r_G_vec))
    ROS_ERROR("Failed to read robot center of mass parameter.");
  if (!nh.getParam("/physical/center_of_buoyancy", r_B_vec))
    ROS_ERROR("Failed to read robot center of buoyancy parameter.");
  Eigen::Vector3d r_G(r_G_vec.data());
  Eigen::Vector3d r_B(r_B_vec.data());

  // Read and calculate ROV weight and buoyancy
  double mass, displacement, acceleration_of_gravity, density_of_water;
  if (!nh.getParam("/physical/mass_kg", mass))
    ROS_ERROR("Failed to read parameter mass.");
  if (!nh.getParam("/physical/displacement_m3", displacement))
    ROS_ERROR("Failed to read parameter displacement.");
  if (!nh.getParam("/gravity/acceleration", acceleration_of_gravity))
    ROS_ERROR("Failed to read parameter acceleration of gravity");
  if (!nh.getParam("/water/density", density_of_water))
    ROS_ERROR("Failed to read parameter density of water");
  double W = mass * acceleration_of_gravity;
  double B = density_of_water * displacement * acceleration_of_gravity;

  position_hold_controller = new QuaternionPdController(gains["a"], gains["b"], gains["c"], W, B, r_G, r_B);
}

bool Controller::healthyMessage(const vortex_msgs::PropulsionCommand& msg)
{
  // Check that motion commands are in range
  for (int i = 0; i < msg.motion.size(); ++i)
  {
    if (msg.motion[i] > 1 || msg.motion[i] < -1)
    {
      ROS_WARN("Controller: Motion command out of range.");
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
    ROS_WARN_STREAM("Controller: Invalid control mode. Attempt to set "
                    << num_requested_modes << " control modes at once.");
    return false;
  }

  return true;
}
