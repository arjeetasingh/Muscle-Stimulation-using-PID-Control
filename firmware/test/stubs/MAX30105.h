#pragma once
#include <stdint.h>
#include "Wire.h"
#define I2C_SPEED_FAST 400000
class MAX30105 {
 public:
  bool begin(TwoWireStub&, uint32_t);
  void setup();
  void setPulseAmplitudeRed(uint8_t);
  void setPulseAmplitudeGreen(uint8_t);
  long getIR();
};
