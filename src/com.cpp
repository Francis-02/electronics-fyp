#include <WiFi.h>
#include <esp_now.h>

#include "com.h"
#include "control.h"
#include "sensors.h"

// =====================================================
// CONFIGURACIÓN
// =====================================================

#define MY_ID 1

// MAC DEL OTRO ESP32

uint8_t otherMAC[] = {0xE0, 0x8C, 0xFE, 0x60, 0xD6, 0x40}; // Para amaraillo
//uint8_t otherMAC[] = {0xE0, 0x8C, 0xFE, 0x60, 0xF9, 0x04}; // Para rojo

// =====================================================
// ESTRUCTURA DE COMUNICACIÓN
// =====================================================

typedef struct
{
  uint8_t id;
  float position_rad;
  float ref_rad;
  uint32_t counter;
} ComPacket;

// =====================================================
// VARIABLES TX/RX
// =====================================================

ComPacket txPacket;
ComPacket rxPacket;

volatile float other_position_rad = 0.0f;
volatile float other_ref_rad      = 0.0f;

volatile uint32_t last_rx_counter = 0;
volatile uint32_t last_rx_time_ms = 0;

// =====================================================
// CALLBACK RECEPCIÓN - CORE ESP32 2.x
// =====================================================

void onEspNowRecv(const uint8_t *mac,
                  const uint8_t *data,
                  int len)
{
  if (len != sizeof(ComPacket))
    return;

  memcpy(&rxPacket, data, sizeof(rxPacket));

  if (rxPacket.id == MY_ID)
    return;

  portENTER_CRITICAL(&mux);

  other_position_rad = rxPacket.position_rad;
  other_ref_rad      = rxPacket.ref_rad;

  last_rx_counter    = rxPacket.counter;
  last_rx_time_ms    = millis();

  portEXIT_CRITICAL(&mux);
}

// =====================================================
// CALLBACK ENVÍO - CORE ESP32 2.x
// =====================================================

void onEspNowSent(const uint8_t *mac_addr,
                  esp_now_send_status_t status)
{
  /*
  Serial.print("ESP-NOW -> ");

  if (status == ESP_NOW_SEND_SUCCESS)
    Serial.println("OK");
  else
    Serial.println("ERROR");

    */
  
}

// =====================================================
// INICIALIZACIÓN
// =====================================================

void initCom()
{
  WiFi.mode(WIFI_STA);

  Serial.print("Mi MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error iniciando ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);

  esp_now_peer_info_t peerInfo = {};

  memcpy(peerInfo.peer_addr, otherMAC, 6);

  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Error añadiendo peer");
    return;
  }

  Serial.println("ESP-NOW iniciado");
}

// =====================================================
// COMPROBAR COMUNICACIÓN
// =====================================================

bool communicationAlive()
{
  uint32_t lastTime;

  portENTER_CRITICAL(&mux);
  lastTime = last_rx_time_ms;
  portEXIT_CRITICAL(&mux);

  return ((millis() - lastTime) < 100);
}

// =====================================================
// TAREA COMUNICACIÓN
// =====================================================

void comTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);

  uint32_t counter = 0;

  for (;;)
  {
    float myPos;
    float myRef;

    portENTER_CRITICAL(&mux);

    myPos = g_position_rad;
    myRef = refTarget_rad;

    portEXIT_CRITICAL(&mux);

    txPacket.id           = MY_ID;
    txPacket.position_rad = myPos;
    txPacket.ref_rad      = myRef;
    txPacket.counter      = counter++;

    esp_now_send(otherMAC,
                 (uint8_t *)&txPacket,
                 sizeof(txPacket));

    vTaskDelayUntil(&xLastWakeTime,
                    xFrequency);
  }
}


void serialTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);

  float t = 0.0f;
  const float Ts = 0.01f;

  for (;;)
  {
    float pos, err, pwm, otherPos;
    bool alive;

    portENTER_CRITICAL(&mux);
    pos = g_position_rad;
    err = g_error_rad;
    pwm = g_pwm_percent;
    otherPos = other_position_rad;
    alive = ((millis() - last_rx_time_ms) < 100);
    portEXIT_CRITICAL(&mux);

    Serial.print(t);
    Serial.print(",");
    Serial.print(pwm);
    Serial.print(",");
    Serial.print(pos);
    Serial.print(",");
    Serial.print(otherPos);
    Serial.print(",");
    Serial.print(err);
    Serial.print(",");
    Serial.println(alive ? 1 : 0);

    t += Ts;

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
