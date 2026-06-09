#pragma once
#include <Arduino.h>

extern volatile long encoderCount;
extern const float countsPerRadian;
extern portMUX_TYPE mux;

void initSensors();
float getPositionRad();