#include "sensors.h"
#include <math.h>

// =====================================================
// PINES ENCODER
// =====================================================
static const int ENC_A = 35;
static const int ENC_B = 34;

// =====================================================
// VARIABLES
// =====================================================
volatile long encoderCount = 0;
//const float countsPerRadian = 224.0f / (2.0f * PI);
const float countsPerRadian = (2*3.14159265359)/1800;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// =====================================================
// ISR ENCODER
// =====================================================
void IRAM_ATTR ISR_encoderA()
{
  portENTER_CRITICAL_ISR(&mux);

  bool A = digitalRead(ENC_A);
  bool B = digitalRead(ENC_B);

  if (A == B) encoderCount++;
  else        encoderCount--;

  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR ISR_encoderB()
{
  portENTER_CRITICAL_ISR(&mux);

  bool A = digitalRead(ENC_A);
  bool B = digitalRead(ENC_B);

  if (A != B) encoderCount++;
  else        encoderCount--;

  portEXIT_CRITICAL_ISR(&mux);
}

// =====================================================
// INIT
// =====================================================
void initSensors()
{
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), ISR_encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), ISR_encoderB, CHANGE);
}

// =====================================================
// POSICIÓN
// =====================================================
float getPositionRad()
{
  long count;

  portENTER_CRITICAL(&mux); 
  count = encoderCount;
  portEXIT_CRITICAL(&mux);

  return -((float)count) * countsPerRadian;
  //return count; 
}