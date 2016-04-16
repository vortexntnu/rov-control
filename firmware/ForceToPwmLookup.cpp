#include "ForceToPwmLookup.h"
#include <Arduino.h>

int ForceToPwm(float force) {
  return MicroSecToPwmValue(ForceToMicroSec(force));
}


int ForceToMicroSec(float force) {

  int us = 0;

  
  for(int i = 0; i < FoceToPwmLookupTableSize; i++) {

    if( force < FoceToPwmLookupTable[i] ) {

      us = i * PwmIncremetMicroSec + PwmLowerMicroSec;
      
      break;
    }
    
  }
  
  return us;
  
}

int MicroSecToPwmValue(int us) {
  return map(us, PwmLowerMicroSec, PwmUpperMicroSec,
	     PwmLowerValue, PwmUpperValue);
}
