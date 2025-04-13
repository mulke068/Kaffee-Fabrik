// #include <Arduino.h>
#include <Wire.h>
#include "MotorDriver.h"
#include "SensorController.h"
#include "LedController.h"
#include "CommandProcessor.h"
#include "TaskManager.h"
#include "Config.h"
#include "HardwareConfig.h"
#include "GlobalConfig.h"

// Motor 1 & 3 Pins are reversed on the PCB
Motor motor1(Hardware::Motor1::PWMA, Hardware::Motor1::AIN2, Hardware::Motor1::AIN1);
Motor motor2(Hardware::Motor2::PWMB, Hardware::Motor2::BIN1, Hardware::Motor2::BIN2);
Motor motor3(Hardware::Motor3::PWMA, Hardware::Motor3::AIN2, Hardware::Motor3::AIN1);
Motor motor4(Hardware::Motor4::PWMB, Hardware::Motor4::BIN1, Hardware::Motor4::BIN2);

Config config;
SensorController sensorController;
LedController ledController;
CommandProcessor commandProcessor;
TaskManager taskManager;

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Wire.begin(Hardware::SDA, Hardware::SCL);

  Serial.write("Initializing...\n");
  Serial.printf("Version: %s\n", FIRMWARE_VERSION);

  config.begin();
  config.loadSettings();

  sensorController.begin();
  ledController.begin();

  motor1.begin();
  motor2.begin();
  motor3.begin();
  motor4.begin();

  pinMode(Hardware::STBY, OUTPUT);
  digitalWrite(Hardware::STBY, HIGH); // TODO MotorDrivers Standby Checking
  Serial.write("Motors initialized\n");

  // Userbutton and ESP Lights
  pinMode(Hardware::BUTTON_PIN, INPUT_PULLUP);

#ifdef ESP_STOP_SWITCH
  pinMode(config.stopPin1, INPUT_PULLUP);
  pinMode(config.stopPin2, INPUT_PULLUP);
#endif

// #if defined(BOARD_HAS_PSRAM)
//   delay(1000);

//   if (esp_spiram_is_initialized())
//   {
//     Serial.write("PSRAM is initialized\n");
//   }
//   else
//   {
//     Serial.write("PSRAM failed to initialise\n");
//   }
// #endif

  commandProcessor.begin(&sensorController, &ledController, &config);

  Serial.write("\n\n===== Integrated System Controller (RTOS) =====\n");
  Serial.write("Type 'HELP' for available commands\n");
  // commandProcessor.printHelp();

  taskManager.begin(&sensorController, &ledController, &config, &commandProcessor);

  Serial.write("Setup complete\n");

  // #ifdef ESP_WIFI_MQTT
  //   setupWiFi();

  //   setupMQTT();

  //   xTaskCreate(MQTTTask, "MQTTTask", 4096, NULL, 1, NULL);
  // #endif
}

void loop()
{
  vTaskDelete(NULL);
}
