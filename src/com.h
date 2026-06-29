#pragma once

#include <Arduino.h>

extern volatile float other_position_rad;

void initCom();

void conectarWiFi();
void reconnectMQTT();

void serialTask(void *pvParameters);
void mqttTask(void *pvParameters);