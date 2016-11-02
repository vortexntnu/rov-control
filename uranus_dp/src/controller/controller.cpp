#include "controller.h"

Controller::Controller()
{
  command_sub  = nh.subscribe("joystick_motion_command", 10, &Controller::commandCallback, this);
  state_sub    = nh.subscribe("state_estimate", 10, &Controller::stateCallback, this);
  wrench_pub   = nh.advertise<geometry_msgs::Wrench>("rov_forces", 10);

  control_mode = ControlModes::OPEN_LOOP;
  prev_time_valid = false;

  position_state.setZero();
  orientation_state.setIdentity();
  velocity_state.setZero();
  position_setpoint.setZero();
  orientation_setpoint.setIdentity();
  wrench_setpoint.setZero();

  getParams();

  // TODO: Read these parameters in a nicer way
  // Read controller gains from parameter server
  double a, b, c;
  if (!nh.getParam("/controller/gains/a", a))
    ROS_ERROR("Failed to read derivative controller gain (a).");
  if (!nh.getParam("/controller/gains/b", b))
    ROS_ERROR("Failed to read position controller gain (b).");
  if (!nh.getParam("/controller/gains/c", c))
    ROS_ERROR("Failed to read orientation controller gain (c).");

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

  position_hold_controller = new QuaternionPdController(a, b, c, r_G, r_B, W, B);
}

void Controller::commandCallback(const vortex_msgs::JoystickMotionCommand& msg)
{
  if (!healthyMessage(msg))
  {
    ROS_WARN("controller: Propulsion command message out of range, ignoring...");
    return;
  }

  if (msg.control_mode != control_mode)
  {
    control_mode = static_cast<ControlMode>(msg.control_mode);
    switch (control_mode)
    {
      case ControlModes::OPEN_LOOP:
      ROS_INFO("Changing control mode to OPEN LOOP.");
      break;

      case ControlModes::POSITION_HOLD:
      ROS_INFO("Changing control mode to POSITION HOLD.");
      position_setpoint    = position_state;
      orientation_setpoint = orientation_state;
      prev_time_valid = false;
      break;
    }
  }

  updateSetpoints(msg);
}

void Controller::stateCallback(const nav_msgs::Odometry &msg)
{
  tf::pointMsgToEigen(msg.pose.pose.position, position_state);
  tf::quaternionMsgToEigen(msg.pose.pose.orientation, orientation_state);
}

void Controller::spin()
{
  ros::Rate rate(frequency);
  while (ros::ok())
  {
    Eigen::Vector6d tau;
    switch (control_mode)
    {
      case ControlModes::POSITION_HOLD:
      tau = position_hold_controller->compute(position_state,
                                              orientation_state,
                                              velocity_state,
                                              position_setpoint,
                                              orientation_setpoint);
      break;

      case ControlModes::OPEN_LOOP:
      tau = wrench_setpoint;
      break;

      // TODO: Reimplement combined mode
    }

    geometry_msgs::Wrench msg;
    tf::wrenchEigenToMsg(tau, msg);
    wrench_pub.publish(msg);

    // TODO: should spinonce and sleep be at beginnig or end? does it matter?
    ros::spinOnce();
    rate.sleep();
  }
}

void Controller::updateSetpoints(const vortex_msgs::JoystickMotionCommand& msg)
{
  // Update wrench setpoints
  wrench_setpoint(0) = wrench_command_scaling[0] * wrench_command_max[0] * msg.forward;
  wrench_setpoint(1) = wrench_command_scaling[1] * wrench_command_max[1] * msg.right;
  wrench_setpoint(2) = wrench_command_scaling[2] * wrench_command_max[2] * msg.down;
  wrench_setpoint(3) = wrench_command_scaling[3] * wrench_command_max[3] * msg.roll_right;
  wrench_setpoint(4) = wrench_command_scaling[4] * wrench_command_max[4] * msg.tilt_up;
  wrench_setpoint(5) = wrench_command_scaling[5] * wrench_command_max[5] * msg.turn_right;

  // Update pose setpoints
  if (!prev_time_valid)
  {
    prev_time = msg.header.stamp;
    prev_time_valid = true;
    return;
  }

  // Calculate time difference
  ros::Time curr_time = msg.header.stamp;
  double dt = (curr_time - prev_time).toSec();
  prev_time = curr_time;

  if (dt == 0)
    ROS_WARN("Zero time difference between propulsion command messages.");

  // Increment position setpoints
  position_setpoint(0) += pose_command_rate[0] * dt * msg.forward;
  position_setpoint(1) += pose_command_rate[1] * dt * msg.right;
  position_setpoint(2) += pose_command_rate[2] * dt * msg.down;

  // Calc euler setpoints
  Eigen::Vector3d orientation_setpoint_euler;
  orientation_setpoint_euler = orientation_setpoint.toRotationMatrix().eulerAngles(2,1,0);
  // Increment euler setpoints
  orientation_setpoint_euler(0) += pose_command_rate[3] * dt * msg.roll_right;
  orientation_setpoint_euler(1) += pose_command_rate[4] * dt * msg.tilt_up;
  orientation_setpoint_euler(2) += pose_command_rate[5] * dt * msg.turn_right;
  // Calc incremented quat setpoints
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(orientation_setpoint_euler(0), Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(orientation_setpoint_euler(1), Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(orientation_setpoint_euler(2), Eigen::Vector3d::UnitX());
  Eigen::Quaterniond q(R);
  orientation_setpoint = q;
}

void Controller::getParams()
{
  // Read control command parameters
  if (!nh.getParam("/control/command/wrench/max", wrench_command_max))
    ROS_FATAL("Failed to read parameter max wrench command.");
  if (!nh.getParam("/control/command/wrench/scaling", wrench_command_scaling))
    ROS_FATAL("Failed to read parameter scaling wrench command.");
  if (!nh.getParam("/control/command/pose/rate", pose_command_rate))
    ROS_FATAL("Failed to read parameter pose command rate.");

  if (!nh.getParam("/controller/frequency", frequency))
  {
    ROS_WARN("Failed to read parameter controller frequency, defaulting to 10 Hz.");
    frequency = 10;
  }
}

bool Controller::healthyMessage(const vortex_msgs::JoystickMotionCommand& msg)
{
  if (abs(msg.forward) > 1)
  {
    ROS_WARN("controller: Forward motion command out of range");
    return false;
  }
  if (abs(msg.right) > 1)
  {
    ROS_WARN("controller: Right motion command out of range");
    return false;
  }
  if (abs(msg.down) > 1)
  {
    ROS_WARN("controller: Down motion command out of range.");
    return false;
  }
  if (abs(msg.tilt_up) > 1)
  {
    ROS_WARN("controller: Tilt up motion command out of range");
    return false;
  }
  if (abs(msg.turn_right) > 1)
  {
    ROS_WARN("controller: Turn right motion command out of range");
    return false;
  }

  bool validControlMode = (msg.control_mode == ControlModes::OPEN_LOOP || msg.control_mode == ControlModes::POSITION_HOLD);
  if (!validControlMode)
  {
    ROS_WARN("controller: Invalid control mode.");
    return false;
  }
  return true;
}
