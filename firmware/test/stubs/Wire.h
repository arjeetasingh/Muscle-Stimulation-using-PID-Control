#pragma once
#include <stdint.h>
struct TwoWireStub {
  void begin(int, int);
  void setClock(uint32_t);
  void beginTransmission(uint8_t);
  size_t write(uint8_t);
  uint8_t endTransmission();
};
extern TwoWireStub Wire;
