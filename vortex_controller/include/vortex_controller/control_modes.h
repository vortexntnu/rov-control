#ifndef VORTEX_CONTROLLER_CONTROL_MODES_H
#define VORTEX_CONTROLLER_CONTROL_MODES_H

#include <string>

namespace ControlModes
{
enum ControlMode
{
  OPEN_LOOP           = 0,
  OPEN_LOOP_RESTORING = 1,
  STAY_LEVEL          = 2,
  DEPTH_HOLD          = 3,
  HEADING_HOLD        = 4,
  DEPTH_HEADING_HOLD  = 5
};
}  // namespace ControlModes
typedef ControlModes::ControlMode ControlMode;

inline std::string controlModeString(ControlMode control_mode)
{
  std::string s;
  switch (control_mode)
  {
    case ControlModes::OPEN_LOOP:
    s = "Open Loop";
    break;

    case ControlModes::OPEN_LOOP_RESTORING:
    s = "Open Loop Restoring";
    break;

    case ControlModes::STAY_LEVEL:
    s = "Stay Level";
    break;

    case ControlModes::DEPTH_HOLD:
    s = "Depth Hold";
    break;

    case ControlModes::HEADING_HOLD:
    s = "Heading Hold";
    break;

    default:
    s = "Invalid Control Mode";
    break;
  }
  return s;
}

#endif  // VORTEX_CONTROLLER_CONTROL_MODES_H
