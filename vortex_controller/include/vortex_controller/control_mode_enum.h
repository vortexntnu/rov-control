#ifndef CONTROL_MODE_ENUM_H
#define CONTROL_MODE_ENUM_H

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

#endif
