#include "ForceToPwmLookup.h"
#include <Arduino.h>

int ForceToPwm(float force)
{
  return MicroSecToPwmValue(ForceToMicroSec(force));
}

int ForceToMicroSec(float force)
{
  double us = PwmUpperMicroSec;
  for (int i = 0; i < ForceLookupSize; ++i)
  {
    if (force < ForceLookup[i])
    {
      double us_prev = PwmLowerMicroSec + PwmIncremetMicroSec*(i-1);
      double us_next = PwmLowerMicroSec + PwmIncremetMicroSec*i;
      double us_diff = us_next - us_prev;
      double force_diff = ForceLookup[i] - ForceLookup[i-1];
      double force_offset = force - ForceLookup[i-1];
      us = us_prev + us_diff*(force_offset/force_diff);
      break;
    }
  }
  return us;
}

int MicroSecToPwmValue(int us)
{
  return map(us, PwmLowerMicroSec, PwmUpperMicroSec, PwmLowerValue, PwmUpperValue);
}
