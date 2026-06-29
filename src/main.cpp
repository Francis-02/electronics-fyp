//HAPTIC_LIDER: MAC: E0:8C:FE:60:D6:40
//HAPTIC SEGUIDOR_MAC: MAC: E0:8C:FE:60:F9:04



#include <Arduino.h>
#include "sensors.h"
#include "control.h"
#include "com.h"

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Iniciando ESP32 + Encoder + PID + MQTT");

  initSensors();
  initControl();
  initCom();

  resetPID();
  communicationAlive();

  xTaskCreatePinnedToCore(
    controlTask,
    "ControlTask",
    4096,
    NULL,
    2,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    serialTask,
    "SerialTask",
    4096,
    NULL,
    1,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    comTask,
    "comTask",
    4096,
    NULL,
    1,
    NULL,
    0
  );

  Serial.println("Sistema iniciado");
  Serial.println("Esperando mensajes en: ESP32/ref");
  Serial.println("Publicando posición en: ESP32/pos");
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(10));
}

/*
//MAC COODE
#include <Arduino.h>
#include <WiFi.h>

void setup()
{
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);

  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop()
{
}

*/

