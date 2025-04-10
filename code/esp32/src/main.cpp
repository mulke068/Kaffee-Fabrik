#include <Arduino.h>
#include "MotorDriver.h"
#include "SensorController.h"
#include "LedController.h"
#include "CommandProcessor.h"
#include "TaskManager.h"
#include "Config.h"
#include "settings.h"

#define ESP_WIFI_MQTT
#define ESP_STOP_SWITCH
#define BOARD_HAS_PSRAM

Motor motor1(PWMA_1, AIN1_1, AIN2_1);
Motor motor2(PWMB_1, BIN1_1, BIN2_1);
Motor motor3(PWMA_2, AIN1_2, AIN2_2);
Motor motor4(PWMB_2, BIN1_2, BIN2_2);

Config config;
SensorController sensorController;
LedController ledController;
CommandProcessor commandProcessor;
TaskManager taskManager;

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);

  config.begin();
  config.loadSettings();

  sensorController.begin();
  ledController.begin();

  motor1.begin();
  motor2.begin();
  motor3.begin();
  motor4.begin();

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // TODO MotorDrivers Standby Checking
  Serial.println("Motors initialized");

  // Userbutton and ESP Lights
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(led17, OUTPUT);
  pinMode(led18, OUTPUT);
  digitalWrite(led17, HIGH);
  digitalWrite(led18, HIGH);

#ifdef ESP_STOP_SWITCH
  pinMode(config.stopPin1, INPUT);
  pinMode(config.stopPin2, INPUT);
#endif

#if defined(BOARD_HAS_PSRAM)
  if (psramFound())
  {
    Serial.println("PSRAM is initialized");
  }
  else
  {
    Serial.println("PSRAM failed to initialise");
  }
#endif

  commandProcessor.begin(&sensorController, &ledController, &config);

  Serial.println("\n===== Integrated System Controller (RTOS) =====");
  Serial.println("Type 'HELP' for available commands");
  commandProcessor.printHelp();


  taskManager.begin(&sensorController, &ledController, &config, &commandProcessor);

// #ifdef ESP_WIFI_MQTT
//   setupWiFi();

//   setupMQTT();

//   xTaskCreate(MQTTTask, "MQTTTask", 4096, NULL, 1, NULL);
// #endif
}

void loop() {
  vTaskDelete(NULL);
}