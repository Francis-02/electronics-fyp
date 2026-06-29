
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

    // Inicialización
    initSensors();
    initControl();
    initCom();

    resetPID();

    // =====================================================
    // TAREA CONTROL
    // =====================================================

    xTaskCreatePinnedToCore(
        controlTask,
        "ControlTask",
        4096,
        NULL,
        2,
        NULL,
        1
    );

    // =====================================================
    // TAREA SERIAL
    // =====================================================

    xTaskCreatePinnedToCore(
        serialTask,
        "SerialTask",
        4096,
        NULL,
        1,
        NULL,
        0
    );

    // =====================================================
    // TAREA MQTT
    // =====================================================

    xTaskCreatePinnedToCore(
        mqttTask,
        "MQTTTask",
        4096,
        NULL,
        1,
        NULL,
        0
    );

    Serial.println("Sistema iniciado");
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(10));
}