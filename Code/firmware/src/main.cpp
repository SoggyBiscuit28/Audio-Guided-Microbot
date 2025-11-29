#include <Arduino.h>
#include <WiFi.h>
#include <driver/i2s.h>
#include <esp_task_wdt.h>
#include <math.h>
#include "I2SMicSampler.h"
// #include "ADCSampler.h"
#include "config.h"
#include "CommandDetector.h"
#include "CommandProcessor.h"
#include "ble_commands.h"
// ...existing code...
#include "neopixel_motorpin.h"

extern BleCommands BLECmd;
// #include "obstacle.h"
// #define TRIG_PIN 4
// #define ECHO_PIN 5

BleCommands BLECmd;
int flag = 1;
static volatile bool g_bleReady = false;
static bool g_controllerMode = false;
static bool g_pwmAttached = false;
static uint32_t g_lastVectorMs = 0;

constexpr uint8_t CONTROLLER_MODE_CODE = 6;
constexpr uint32_t CONTROLLER_TIMEOUT_MS = 400;
constexpr float JOYSTICK_DEADBAND = 0.08f;
constexpr uint8_t LEDC_LEFT_FWD_CHANNEL = 0;
constexpr uint8_t LEDC_LEFT_REV_CHANNEL = 1;
constexpr uint8_t LEDC_RIGHT_FWD_CHANNEL = 2;
constexpr uint8_t LEDC_RIGHT_REV_CHANNEL = 3;
constexpr uint32_t LEDC_PWM_FREQ = 20000;
constexpr uint8_t LEDC_PWM_RES_BITS = 8;

static void attachMotorPwm();
static void detachMotorPwm();
static void applyMotorSpeed(float value, uint8_t forwardChannel, uint8_t reverseChannel);
static void driveWithVector(float x, float y);
static void stopControllerMotion();
static void handleJoystickVector(float x, float y);

constexpr uint8_t SENSOR_PIN = 7; // GPIO7 / ADC1_CH6 on ESP32-S3 for voltage
constexpr float ADC_REF_VOLTAGE = 3.3f;
constexpr uint16_t ADC_FULL_SCALE = 4095;

// i2s config for reading from both channels of I2S
i2s_config_t i2sMemsConfigBothChannels = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_MIC_CHANNEL,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// i2s microphone pins
i2s_pin_config_t i2s_mic_pins = {
    .bck_io_num = I2S_MIC_SERIAL_CLOCK,
    .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SERIAL_DATA};

static void attachMotorPwm()
{
  if (g_pwmAttached)
  {
    return;
  }

  ledcSetup(LEDC_LEFT_FWD_CHANNEL, LEDC_PWM_FREQ, LEDC_PWM_RES_BITS);
  ledcSetup(LEDC_LEFT_REV_CHANNEL, LEDC_PWM_FREQ, LEDC_PWM_RES_BITS);
  ledcSetup(LEDC_RIGHT_FWD_CHANNEL, LEDC_PWM_FREQ, LEDC_PWM_RES_BITS);
  ledcSetup(LEDC_RIGHT_REV_CHANNEL, LEDC_PWM_FREQ, LEDC_PWM_RES_BITS);

  ledcAttachPin(IN1, LEDC_LEFT_FWD_CHANNEL);
  ledcAttachPin(IN2, LEDC_LEFT_REV_CHANNEL);
  ledcAttachPin(IN4, LEDC_RIGHT_FWD_CHANNEL);
  ledcAttachPin(IN3, LEDC_RIGHT_REV_CHANNEL);

  ledcWrite(LEDC_LEFT_FWD_CHANNEL, 0);
  ledcWrite(LEDC_LEFT_REV_CHANNEL, 0);
  ledcWrite(LEDC_RIGHT_FWD_CHANNEL, 0);
  ledcWrite(LEDC_RIGHT_REV_CHANNEL, 0);

  g_pwmAttached = true;
}

static void detachMotorPwm()
{
  if (!g_pwmAttached)
  {
    return;
  }

  ledcDetachPin(IN1);
  ledcDetachPin(IN2);
  ledcDetachPin(IN3);
  ledcDetachPin(IN4);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(DRV_EEP, LOW);

  g_pwmAttached = false;
}

static void applyMotorSpeed(float value, uint8_t forwardChannel, uint8_t reverseChannel)
{
  if (value > 1.0f)
  {
    value = 1.0f;
  }
  else if (value < -1.0f)
  {
    value = -1.0f;
  }

  float magnitude = fabsf(value);
  uint32_t duty = static_cast<uint32_t>(magnitude * ((1u << LEDC_PWM_RES_BITS) - 1u));

  if (value > 0.0f)
  {
    ledcWrite(forwardChannel, duty);
    ledcWrite(reverseChannel, 0);
  }
  else if (value < 0.0f)
  {
    ledcWrite(forwardChannel, 0);
    ledcWrite(reverseChannel, duty);
  }
  else
  {
    ledcWrite(forwardChannel, 0);
    ledcWrite(reverseChannel, 0);
  }
}

static float clampUnit(float value)
{
  if (value > 1.0f) return 1.0f;
  if (value < -1.0f) return -1.0f;
  return value;
}

static float applyJoystickDeadband(float value)
{
  return (fabsf(value) < JOYSTICK_DEADBAND) ? 0.0f : value;
}

static void driveWithVector(float x, float y)
{
  if (!g_controllerMode)
  {
    return;
  }

  if (!g_pwmAttached)
  {
    attachMotorPwm();
  }

  float xInput = applyJoystickDeadband(clampUnit(x));
  float yInput = applyJoystickDeadband(clampUnit(y));
  float forward = yInput;
  float turn = -xInput; // invert to align joystick X with robot's turning direction
  float left = forward + turn;
  float right = forward - turn;

  float limit = fmaxf(fabsf(left), fabsf(right));
  if (limit > 1.0f)
  {
    left /= limit;
    right /= limit;
  }

  float magnitude = clampUnit(sqrtf((xInput * xInput) + (yInput * yInput)));
  left *= magnitude;
  right *= magnitude;

  applyMotorSpeed(left, LEDC_LEFT_FWD_CHANNEL, LEDC_LEFT_REV_CHANNEL);
  applyMotorSpeed(right, LEDC_RIGHT_FWD_CHANNEL, LEDC_RIGHT_REV_CHANNEL);

  bool moving = (fabsf(left) > 0.001f) || (fabsf(right) > 0.001f);
  digitalWrite(DRV_EEP, moving ? HIGH : LOW);

  g_lastVectorMs = millis();
}

static void stopControllerMotion()
{
  if (g_pwmAttached)
  {
    ledcWrite(LEDC_LEFT_FWD_CHANNEL, 0);
    ledcWrite(LEDC_LEFT_REV_CHANNEL, 0);
    ledcWrite(LEDC_RIGHT_FWD_CHANNEL, 0);
    ledcWrite(LEDC_RIGHT_REV_CHANNEL, 0);
  }
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(DRV_EEP, LOW);

}

// This task does all the heavy lifting for our application
void applicationTask(void *param)
{
  CommandDetector *commandDetector = static_cast<CommandDetector *>(param);

  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);
  while (true)
  {
    // wait for some audio samples to arrive
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    if (ulNotificationValue > 0)
    {
      commandDetector->run();
    }
  }
}

static void handleJoystickVector(float x, float y)
{
  Serial.printf("[BLE] Joystick vector callback x=%.3f y=%.3f (controller %s)\n", x, y, g_controllerMode ? "ON" : "OFF");
  if (!g_controllerMode)
  {
    Serial.println("[BLE] Ignoring joystick vector because controller mode is OFF");
    return;
  }

  if (x < -1.0f) x = -1.0f;
  if (x > 1.0f) x = 1.0f;
  if (y < -1.0f) y = -1.0f;
  if (y > 1.0f) y = 1.0f;

  driveWithVector(x, y);
}

void handleCode(int code){
  Serial.printf("[RX] Received code: %d\n", code);

  if (code == CONTROLLER_MODE_CODE)
  {
    if (!g_controllerMode)
    {
      g_controllerMode = true;
      attachMotorPwm();
      stopControllerMotion();
      g_lastVectorMs = millis();
      flag = 0;
      BLECmd.notify("MODE:CONTROLLER_ON");
    }
    else
    {
      stopControllerMotion();
      detachMotorPwm();
      g_controllerMode = false;
      flag = 1;
      BLECmd.notify("MODE:CONTROLLER_OFF");
    }
    return;
  }

  if(code==0){
    if (g_controllerMode)
    {
      stopControllerMotion();
      detachMotorPwm();
      g_controllerMode = false;
      flag = 1;
      BLECmd.notify("MODE:CONTROLLER_OFF");
    }

    if(flag == 0){
      flag = 1;
    }
    else{
      flag = 0;
    }
    return;
  }

  if (g_controllerMode)
  {
    Serial.println("[RX] Ignoring direct motor command while joystick controller mode is active");
    return;
  }
  else if(code==1 && !flag){
    Serial.println("Forward");
    pinMode(4, OUTPUT);
    pinMode(5, INPUT);
    digitalWrite(4, LOW);
    delayMicroseconds(2);
    digitalWrite(4, HIGH);
    delayMicroseconds(10);
    digitalWrite(4, LOW);
    long duration = pulseIn(5, HIGH, 20000);
    float d = 0;
    if(duration == 0){
        d = -1;
    }
    else{
        d = duration * 0.034 / 2.0;
    }
    Serial.print("Distance: ");
    Serial.print(d, 2);
    Serial.println("cm");
    if(d > 0 && d < 6.0){
      Serial.println("Obstacle detected! Stopping.");
      BLECmd.notify("OBSTACLE");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      BLECmd.notify("IDLE");
      return;
    }
    else{
      BLECmd.notify("NO_OBSTACLE");
      digitalWrite(DRV_EEP, HIGH);
    }
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    // Obstacle detection in manual override
    digitalWrite(4, LOW);
    delayMicroseconds(2);
    digitalWrite(4, HIGH);
    delayMicroseconds(10);
    digitalWrite(4, LOW);
    for (int i = 0; i < 47; i++) {
      digitalWrite(4, LOW);
      delayMicroseconds(2);
      digitalWrite(4, HIGH);
      delayMicroseconds(10);
      digitalWrite(4, LOW);
      long duration = pulseIn(5, HIGH, 20000);
      float d = 0;
      if(duration == 0){
          d = -1;
      }
      else{
          d = duration * 0.034 / 2.0;
      }
      if(d > 0 && d < 6.0){
          Serial.println("Obstacle detected! Stopping.");
          BLECmd.notify("OBSTACLE");
          vTaskDelay(500 / portTICK_PERIOD_MS);
          break;
      }
      else{
        BLECmd.notify("NO_OBSTACLE");
      }
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
  else if(code==2 && !flag){
    digitalWrite(DRV_EEP, HIGH);
    Serial.println("Backward");
    BLECmd.notify("NO_OBSTACLE");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
  else if(code==3 && !flag){
    digitalWrite(DRV_EEP, HIGH);
    Serial.println("Left");
    BLECmd.notify("NO_OBSTACLE");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
  else if(code==4 && !flag){
    digitalWrite(DRV_EEP, HIGH);
    Serial.println("Right");
    BLECmd.notify("NO_OBSTACLE");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  BLECmd.notify("IDLE");
  digitalWrite(DRV_EEP, LOW);
  Serial.println("[RX] Completed manual command, robot idle");
}

void bleReceiverTask(void *param){
  Serial.println("[BLE] BLE Receiver Task started");
  BLECmd.begin("ESP32_Receiver");
  BLECmd.onCommand(handleCode);
  BLECmd.onVector(handleJoystickVector);
  g_bleReady = true;
  while (true) {
    BLECmd.poll();
    vTaskDelay(pdMS_TO_TICKS(5)); // Yield to other tasks
  }
}

void voltageMonitorTask(void *param){
  Serial.println("[ADC] Voltage monitor task started");
  while (true) {
    if (!g_bleReady) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    int raw = analogRead(SENSOR_PIN);
    float voltage = raw * (ADC_REF_VOLTAGE / static_cast<float>(ADC_FULL_SCALE));

    Serial.printf("[ADC] Raw: %d | Voltage: %.4f V\n", raw, voltage);

    char msg[32];
    snprintf(msg, sizeof(msg), "ADC:%d Voltage:%.4fV", raw, voltage);
    BLECmd.notify(msg);

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting up");
  // obstacle_init(TRIG_PIN, ECHO_PIN);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(SENSOR_PIN, INPUT);

  pinMode(DRV_EEP, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // make sure we don't get killed for our long running tasks
  esp_task_wdt_init(10, false);

  // start up the I2S input (from either an I2S microphone or Analogue microphone via the ADC)
  I2SSampler *i2s_sampler = new I2SMicSampler(i2s_mic_pins, false);
  // the command processor
  CommandProcessor *command_processor = new CommandProcessor();

  // create our application
  CommandDetector *commandDetector = new CommandDetector(i2s_sampler, command_processor);

  // set up the i2s sample writer task
  TaskHandle_t applicationTaskHandle;
  xTaskCreatePinnedToCore(applicationTask, "Command Detect", 8192, commandDetector, 1, &applicationTaskHandle, 0);

  // start sampling from i2s device - use I2S_NUM_0 as that's the one that supports the internal ADC
  i2s_sampler->start(I2S_NUM_0, i2sMemsConfigBothChannels, applicationTaskHandle);

  xTaskCreatePinnedToCore(
    bleReceiverTask, "BLE Receiver", 4096, NULL, 1, NULL, 1
  );

  xTaskCreatePinnedToCore(
    voltageMonitorTask, "Voltage Monitor", 3072, NULL, 1, NULL, 1
  );
}

void loop()
{
  if (g_controllerMode)
  {
    uint32_t now = millis();
    if (now - g_lastVectorMs > CONTROLLER_TIMEOUT_MS)
    {
      stopControllerMotion();
      g_lastVectorMs = now;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  else
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}