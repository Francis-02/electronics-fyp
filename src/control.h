#pragma once
#include <Arduino.h>

extern volatile float refPos_rad;
extern volatile float refTarget_rad;

extern float g_velocity_rad_s;

extern volatile float g_position_rad;
extern volatile float g_error_rad;
extern volatile float g_pwm_percent;

void initControl();
void resetPID();
float controladorPID(float ref_rad, float pos_rad);
void setMotorPWM_percent(float pwm_percent);
float deadzoneInverse(float x_percent, float b_percent);
void controlTask(void *pvParameters);