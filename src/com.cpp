#include "com.h"
#include "control.h"
#include "sensors.h"

#include <WiFi.h>
#include <PubSubClient.h>

// =====================================================
// WIFI Y MQTT
// =====================================================

static const char* ssid = "OLIN_67373";
static const char* password = "umTsf4Bv";

static const char* mqtt_server = "192.168.18.2";
static const int mqtt_port = 1883;
static const char* mqtt_user = "";
static const char* mqtt_pass = "";

static const char* topic_ref = "ESP32/esp1";
static const char* topic_pos = "ESP32/esp2";

static WiFiClient espClient;
static PubSubClient client(espClient);

// Posición recibida desde el otro ESP32
volatile float other_position_rad = 0.0f;

// =====================================================
// CALLBACK MQTT
// =====================================================

static void mqttCallback(char* topic, byte* payload, unsigned int length)
{
    String mensaje = "";

    for (unsigned int i = 0; i < length; i++)
    {
        mensaje += (char)payload[i];
    }

    mensaje.trim();

    if (String(topic) == topic_ref)
    {
        float posRecibida = mensaje.toFloat();

        portENTER_CRITICAL(&mux);
        other_position_rad = posRecibida;
        portEXIT_CRITICAL(&mux);

        Serial.print("Pos externa recibida = ");
        Serial.println(posRecibida);
    }
}

// =====================================================
// INIT COM
// =====================================================

void initCom()
{
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqttCallback);
    conectarWiFi();
}

// =====================================================
// WIFI
// =====================================================

void conectarWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.print("Conectando a WiFi");

    int intentos = 0;

    while (WiFi.status() != WL_CONNECTED && intentos < 40)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print(".");
        intentos++;
    }

    Serial.println();

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.print("WiFi conectado. IP: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.print("No se pudo conectar al WiFi. Estado = ");
        Serial.println(WiFi.status());
    }
}

// =====================================================
// MQTT
// =====================================================

void reconnectMQTT()
{
    while (!client.connected())
    {
        Serial.print("Conectando a MQTT... ");

        String clientId = "ESP32Client-2";
        clientId += String((uint32_t)ESP.getEfuseMac(), HEX);

        bool ok;

        if (strlen(mqtt_user) > 0)
            ok = client.connect(clientId.c_str(), mqtt_user, mqtt_pass);
        else
            ok = client.connect(clientId.c_str());

        if (ok)
        {
            Serial.println("conectado");

            bool sub_ok = client.subscribe(topic_ref);
            Serial.print("Subscribe OK = ");
            Serial.println(sub_ok);

            Serial.print("Suscrito a: ");
            Serial.println(topic_ref);
        }
        else
        {
            Serial.print("fallo, rc=");
            Serial.print(client.state());
            Serial.println(" -> reintentando en 2 s");

            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}

// =====================================================
// TAREA SERIAL
// =====================================================

void serialTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    float t = 0.0f;
    const float Ts = 0.01f;

    for (;;)
    {
        float pos, otherPos, err, pwm;

        portENTER_CRITICAL(&mux);
        pos = g_position_rad;
        otherPos = other_position_rad;
        err = g_error_rad;
        pwm = g_pwm_percent;
        portEXIT_CRITICAL(&mux);

        Serial.print(t);
        Serial.print(",");
        Serial.print(pwm);
        Serial.print(",");
        Serial.print(pos);
        Serial.print(",");
        Serial.println(otherPos);


        t += Ts;

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// =====================================================
// TAREA MQTT
// =====================================================

void mqttTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    TickType_t lastPub = 0;
    char msg[32];

    conectarWiFi();

    for (;;)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            conectarWiFi();
        }

        if (WiFi.status() == WL_CONNECTED && !client.connected())
        {
            reconnectMQTT();
        }

        if (client.connected())
        {
            client.loop();

            if ((xTaskGetTickCount() - lastPub) >= pdMS_TO_TICKS(50))
            {
                float pos;

                portENTER_CRITICAL(&mux);
                pos = g_position_rad;
                portEXIT_CRITICAL(&mux);

                dtostrf(pos, 0, 4, msg);

                client.publish(topic_pos, msg);

                lastPub = xTaskGetTickCount();
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}