#pragma once
#include <Arduino.h>


extern volatile float other_position_rad;

void conectarWiFi();
void reconnectMQTT();
void mqttTask(void *pvParameters);
void serialTask(void *pvParameters);
void initCom();
void comTask(void *pvParameters);
bool communicationAlive();