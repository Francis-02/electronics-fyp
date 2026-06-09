#include "control.h"
#include "sensors.h"
#include "com.h"
#include <math.h>

float g_velocity_rad_s;

// =====================================================
// PINES MOTOR
// =====================================================
static const int IN1 = 16;
static const int IN2 = 17;
static const int ENA = 4;

const float Vmax = 11.6;   // tensión máxima de armadura en voltios

// =====================================================
// LEDC ESP32
// =====================================================
static const uint8_t  PWM_CHANNEL = 0;
static const uint32_t PWM_FREQ    = 500;
static const uint8_t  PWM_RES     = 8;

static const float pwmMax_hw = 255.0f;
const float bDead_V = 2.9f;   // aprox. tensión mínima útil
//2.9
// =====================================================
// PID EN TENSION DE ARMADURA
// =====================================================
float Kp = 0.75f;
float Ki = 0.0f;
float Kd = 0.15f;

volatile float refPos_rad    = 1000;
volatile float refTarget_rad = 1000.0f;

static float I = 0.0f;
static float prevErr = 0.0f;
static float Df = 0.0f;

// =====================================================
// VARIABLES COMPARTIDASf
// =====================================================
volatile float g_position_rad = 0.0f;
volatile float g_error_rad    = 0.0f;
volatile float g_pwm_percent  = 0.0f;

// =====================================================
// MODELO VIRTUAL MASA-RESORTE-AMORTIGUADOR
// Entrada: posición angular real del haptic paddle
// Salida: referencia angular nueva para el PID
//
// Idea:
// El sistema toma como equilibrio la posición inicial.
// Si el usuario mueve el haptic respecto a esa posición,
// se genera una referencia en sentido contrario.
// =====================================================
float modelMRA(float theta_rad, float theta_dot_rad_s)
{

  const float kv_neg      = 0.01f;   // cancela amortiguamiento PID

  // Parámetros del oscilador virtual
  const float Ts          = 0.01f;
  const float M           = 0.05f;   // inercia virtual
  const float kp_mra      = 20.00f;    // 
  const float kv_muelle   = 0.0f;    // amortiguamiento interno del muelle
  const float k_coupling  = 0.0f;    // acoplamiento usuario-muelle

  static float theta_eq   = 0.0f;
  static float x_pos      = 0.0f;   // posición del muelle virtual
  static float x_vel      = 0.0f;   // velocidad del muelle virtual
  static bool  init       = false;

  if (!init)
  {
    theta_eq = theta_rad;
    x_pos    = theta_rad;
    x_vel    = 0.0f;
    init     = true;
  }
  return kp_mra * (theta_rad) + kv_muelle * (theta_dot_rad_s);
}

// =====================================================
// INIT
// =====================================================
void initControl()
{
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);

  refPos_rad = getPositionRad();
  refTarget_rad = 6.0f;
}

// =====================================================
// RESET PID
// =====================================================
void resetPID()
{
  I = 0.0f;
  prevErr = 0.0f;
  Df = 0.0f;

  refPos_rad = getPositionRad();
}

// =====================================================
// CONVERSION TENSION ARMADURA -> PWM HW 0-255
// =====================================================
int vArmToPWM(float Va)
{
  float pwm_percent = 0;
  
  if (Va < 0.0f) Va = -Va;

  if (Va > 10.6f) Va = 10.6f;

  /*

  if (Va >= bDead_V)
  {
    pwm_percent =
      0.5357f * Va * Va * Va
    - 8.9478f * Va * Va
    + 50.8605f * Va
    - 69.6389f;
  }
  else
  {
    pwm_percent = (Va / Vmax) * 100.0f;
  }*/

  pwm_percent = (Va / Vmax) * 100.0f;

  // Convertir PWM (%) a PWM 0-255
  float pwm = (pwm_percent / 100.0f) * 255.0f;

  if (pwm > 255.0f) pwm = 255.0f;
  if (pwm < 0.0f)   pwm = 0.0f;

  return (int)pwm;
}

// =====================================================
// MOTOR CON TENSION DE ARMADURA
// Va > 0  -> sentido positivo
// Va < 0  -> sentido negativo
// Va = 0  -> motor parado
// =====================================================
void setMotorVoltage(float Va)
{
  int pwm_hw = vArmToPWM(Va);

  if (Va > 0.0f)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM_CHANNEL, pwm_hw);
  }
  else if (Va < 0.0f)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(PWM_CHANNEL, pwm_hw);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM_CHANNEL, 0);
  }
}

// =====================================================
// COMPENSACIÓN INVERSA DE ZONA MUERTA
// =====================================================
float deadzoneInverseVoltage(float Va)
{
  const float epsilon = 0.08f;

  float aVa = fabs(Va);

  if (aVa < epsilon)
  {
    return bDead_V * sinf((PI * Va) / (2.0f * epsilon));
  }

  float t = (aVa - epsilon) / (Vmax - epsilon);
  float Va_comp = bDead_V + (Vmax - bDead_V) * t;

  if (Va > 0.0f) return Va_comp;
  else           return -Va_comp;
}

// =====================================================
// MOTOR PWM EN %
// Lo dejo por si quieres seguir haciendo ensayos manuales
// =====================================================
void setMotorPWM_percent(float pwm_percent)
{
  if (pwm_percent > 100.0f)  pwm_percent = 100.0f;
  if (pwm_percent < -100.0f) pwm_percent = -100.0f;

  int pwm_hw = (int)((fabs(pwm_percent) / 100.0f) * pwmMax_hw);

  if (pwm_percent > 0.0f)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM_CHANNEL, pwm_hw);
  }
  else if (pwm_percent < 0.0f)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(PWM_CHANNEL, pwm_hw);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM_CHANNEL, 0);
  }
}

// =====================================================
// CONTROLADOR PID
// ref_rad: referencia deseada en radianes
// pos_rad: posicion actual en radianes
// salida: tension de armadura Va en voltios
// =====================================================

float controladorPID(float ref_rad, float pos_rad)
{
  const float Ts = 0.01f;
  const float alpha = 0.99f;

  float err = ref_rad - pos_rad;

  I += err * Ts;

  if (I > 100.0f)  I = 100.0f;
  if (I < -100.0f) I = -100.0f;

  float D_raw = (err - prevErr) / Ts;
  Df = alpha * Df + (1.0f - alpha) * D_raw;

  prevErr = err;

  float Va = Kp * err + Ki * I + Kd * Df;

  if (Va > 10.7f)  Va = 10.7f;
  if (Va < -10.7f) Va = -10.7f;

  return Va;
}

// =====================================================
// TAREA DE CONTROL EN BUCLE CERRADO
// =====================================================
void controlTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);

  const float Ts = 0.01f;
  const float alpha = 0.98f;

  float position_rad_prev = getPositionRad();
  float velocity_raw = 0.0f;
  float velocity_f = 0.0f;

  for (;;)
  {
    float position_rad = getPositionRad();

    // ==============================
    // Cálculo de velocidad real
    // ==============================
    velocity_raw = (position_rad - position_rad_prev) / Ts;
    velocity_f = alpha * velocity_f + (1.0f - alpha) * velocity_raw;
    position_rad_prev = position_rad;

    // =====================================================
    // MODELO MASA-RESORTE-AMORTIGUADOR DESACTIVADO
    // Se deja comentado para mantener el código original,
    // pero ahora se usa un PID fijo con referencia fija.
    // =====================================================
    // float refMRA_rad = modelMRA(position_rad, velocity_f);

    // PID FIJO:
    float Va = controladorPID(refTarget_rad, position_rad);

    // ANTES CON MRA:
    // float Va = modelMRA(position_rad, -velocity_f);
    // =====================================================
    // Aplicación al motor con compensación de zona muerta
    // =====================================================
    float Va_comp = deadzoneInverseVoltage(Va);

    setMotorVoltage(Va_comp);

    // =====================================================
    // Cálculo del PWM realmente aplicado
    // =====================================================
    int pwm_hw = vArmToPWM(Va_comp);
    float pwm_percent = ((float)pwm_hw / 255.0f) * 100.0f;

    if (Va_comp < 0.0f)
      pwm_percent = -pwm_percent;

    // =====================================================
    // Variables compartidas
    // =====================================================
    portENTER_CRITICAL(&mux);
    g_position_rad   = position_rad;

    // PID FIJO:
    g_error_rad      = refTarget_rad - position_rad;

    // ANTES CON MRA:
    // g_error_rad      = refMRA_rad - position_rad;

    g_pwm_percent    = pwm_percent;
    g_velocity_rad_s = velocity_f;
    portEXIT_CRITICAL(&mux);


    refTarget_rad = other_position_rad;
/*
    if (Serial.available())
    {
        char tecla = Serial.read();

        switch (tecla)
        {
            case ' ':
                refTarget_rad += 5.0f;
                break;

            case 'b':
                refTarget_rad -= 5.0f;
                break;
        }

        Serial.print("Referencia: ");
        Serial.println(refTarget_rad);
    }
    */
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }

}

/*
void controlTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);
  const float Ts = 0.01f;   // 10 ms

  // ==============================
  // PARÁMETROS DE LOS ENSAYOS PWM
  // ==============================
  const float stepAmp   = 100.0f;   // % PWM para escalones
  const float rampMax   = 100.0f;   // % PWM máximo de la rampa
  const float rampDead  = 24.0f;    // % PWM mínimo de rampa (zona muerta)
  const float stepTime  = 1.0f;     // s
  const float rampTime  = 1.0f;     // s

  const float rampSlope = (rampMax - rampDead) / rampTime;   // %PWM/s

  // Filtro de velocidad
  const float alpha = 0.98f;

  enum TestType
  {
    STEP_POS = 0,
    STEP_NEG,
    RAMP_POS,
    RAMP_NEG
  };

  TestType test = STEP_POS;
  float tTest = 0.0f;
  float pwm_percent = 0.0f;

  float position_prev = getPositionRad();
  float velocity_raw = 0.0f;
  float velocity_f = 0.0f;

  for (;;)
  {
    float position_rad = getPositionRad();

    // ==============================
    // Cálculo de velocidad
    // ==============================
    velocity_raw = (position_rad - position_prev) / Ts;
    velocity_f = alpha * velocity_f + (1.0f - alpha) * velocity_raw;
    position_prev = position_rad;

    // ==============================
    // Generación de ensayos PWM
    // ==============================
    switch (test)
    {
      case STEP_POS:
        pwm_percent = stepAmp;
        if (tTest >= stepTime)
        {
          test = STEP_NEG;
          tTest = 0.0f;
        }
        break;

      case STEP_NEG:
        pwm_percent = -stepAmp;
        if (tTest >= stepTime)
        {
          test = RAMP_POS;
          tTest = 0.0f;
        }
        break;

      case RAMP_POS:
        pwm_percent = rampDead + rampSlope * tTest;
        if (pwm_percent > rampMax)
          pwm_percent = rampMax;

        if (tTest >= rampTime)
        {
          test = RAMP_NEG;
          tTest = 0.0f;
        }
        break;

      case RAMP_NEG:
        pwm_percent = -(rampDead + rampSlope * tTest);
        if (pwm_percent < -rampMax)
          pwm_percent = -rampMax;

        if (tTest >= rampTime)
        {
          test = STEP_POS;
          tTest = 0.0f;
        }
        break;
    }

    // Aplicar PWM directamente
    setMotorPWM_percent(100.0f);

    // Variables compartidas
    portENTER_CRITICAL(&mux);
    g_position_rad   = position_rad;
    g_error_rad      = 0.0f;
    g_pwm_percent    = pwm_percent;
    g_velocity_rad_s = velocity_f;
    portEXIT_CRITICAL(&mux);

    tTest += Ts;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
*/

/*
void controlTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100);

  for (;;)
  {
    float position_rad = getPositionRad();

    g_position_rad = position_rad;

    Serial.print("Posicion: ");
    Serial.println(position_rad);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
*/