// Minimal Arduino/ESP32 stubs so the sketch can be syntax-checked on a PC (no hardware behaviour).
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdarg.h>
#include "freertos_stub.h"
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define ADC_11db 3
void pinMode(int, int);
void digitalWrite(int, int);
int digitalRead(int);
int analogRead(int);
void analogReadResolution(int);
void analogSetPinAttenuation(int, int);
uint32_t micros();
uint32_t millis();
void delay(uint32_t);
struct SerialStub {
  void begin(long);
  int available();
  int read();
  size_t write(const uint8_t*, size_t);
  void println(const char*);
};
extern SerialStub Serial;
