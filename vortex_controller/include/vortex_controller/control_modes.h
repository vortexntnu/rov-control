#ifndef VORTEX_CONTROLLER_CONTROL_MODES_H
#define VORTEX_CONTROLLER_CONTROL_MODES_H

#include <string>

namespace ControlModes
{
enum ControlMode
{
  OPEN_LOOP  = 0,
  SIXDOF     = 1,
  RPY_DEPTH  = 2,
  DEPTH_HOLD = 3
};
}
typedef ControlModes::ControlMode ControlMode;

inline std::string controlModeString(ControlMode control_mode)
{
  std::string s;
  switch (control_mode)
  {
    case ControlModes::OPEN_LOOP:
    s = "Open Loop";
    break;

    case ControlModes::SIXDOF:
    s = "Six DOF";
    break;

    case ControlModes::RPY_DEPTH:
    s = "RPY Depth";
    break;

    case ControlModes::DEPTH_HOLD:
    s = "Depth Hold";
    break;

    default:
    s = "Invalid Control Mode";
    break;
  }
  return s;
}

#endif  // VORTEX_CONTROLLER_CONTROL_MODES_H
