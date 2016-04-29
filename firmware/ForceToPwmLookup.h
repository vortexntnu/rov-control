#ifndef FORCETOPWMLOOKUP_H
#define FORCETOPWMLOOKUP_H

int ForceToPwm(float force);

int ForceToMicroSec(float force);
int MicroSecToPwmValue(int us);

const int PulseWidthMin = 1300;
const int PulseWidthMax = 1700;
const int PulseWidthIncrement = 10;

const int PwmLowerValue = 162;
const int PwmUpperValue = 214;

const int ForceLookupSize = 40;
  //kraft i Newton
const float ForceLookup[ForceLookupSize] = {
  -5.7056872745,
  -5.2153547745,
  -4.6358709118,
  -4.1901140882,
  -3.92266,
  -3.6106302255,
  -3.2094490882,
  -2.6745409118,
  -2.2733597745,
  -1.96133,
  -1.6493002255,
  -1.3818461373,
  -1.0252406863,
  -0.7132109118,
  -0.4903325,
  -0.3120297745,
  -0.133727049,
  -0.0891513627,
  0,
  0,
  0,
  0,
  0.0891513627,
  0.356605451,
  0.624059549,
  1.0252406863,
  1.4264218137,
  1.9167543137,
  2.4962381863,
  3.0311463627,
  3.6106302255,
  4.2346897745,
  4.7250222745,
  5.3490818137,
  6.017717049,
  6.730927951,
  7.221260451,
  7.8898956863,
  8.6476822745,
  9.450044549, //indeks 39
};

#endif /* FORCETOPWMLOOKUP_H  */
