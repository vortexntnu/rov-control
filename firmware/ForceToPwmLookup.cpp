#include "ForceToPwmLookup.h"


int ForceToPwm(float force) {

  int pwmus = 0;

  
  for(int i = 0; i < FoceToPwmLookupTableSize; i++) {

    if( force < FoceToPwmLookupTable[i] ) {

      pwmus = i * PwmIncremetMicroSec + PwmLowerMicroSec;
      
      break;
    }
    
  }
  
  return pwmus;
  
}
