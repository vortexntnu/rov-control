#ifndef CONTROL_MODE_ENUM_H
#define CONTROL_MODE_ENUM_H

namespace ControlModes
{
enum ControlMode
{
  OPEN_LOOP = 0,
  POSITION_HOLD = 1
};
}
typedef ControlModes::ControlMode ControlMode;

#endif
