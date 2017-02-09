#ifndef VORTEX_HELPER_H
#define VORTEX_HELPER_H

#include "uranus_dp/control_mode_enum.h"

inline std::string controlModeString(ControlMode control_mode)
{
  std::string s;
  switch(control_mode)
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

#endif
