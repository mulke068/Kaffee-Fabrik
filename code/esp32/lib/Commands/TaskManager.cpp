
#include "TaskManager.h"
#include "settings.h"

TaskManager::TaskManager()
{
  _sensorCtrl = nullptr;
  _ledCtrl = nullptr;
  _cmdProc = nullptr;
  _cfg = nullptr;
  _motorTaskHandle = nullptr;
  _sensorTaskHandle = nullptr;
  _commandTaskHandle = nullptr;
}

TaskManager::~TaskManager()
{
  // Destructor implementation (if needed)
}

void TaskManager::begin(SensorController *sensorCtrl, LedController *ledCtrl, Config *cfg, CommandProcessor *cmdProc)
{
  _sensorCtrl = sensorCtrl;
  _ledCtrl = ledCtrl;
  _cfg = cfg;
  _cmdProc = cmdProc;

  createMotorTask();
  createSensorTask();
  createCommandTask();
}

void TaskManager::createCommandTask()
{
  xTaskCreate(
      commandTaskFunction,
      "CommandTask",
      4096,
      this,
      1,
      &_commandTaskHandle);
}

void TaskManager::createMotorTask()
{
  xTaskCreate(
      motorTaskFunction,
      "MotorTask",
      2048,
      this,
      2,
      &_motorTaskHandle);
}

void TaskManager::createSensorTask()
{
  xTaskCreate(
      sensorTaskFunction,
      "SensorTask",
      2048,
      this,
      2,
      &_sensorTaskHandle);
}

void TaskManager::commandTaskFunction(void *pvParameters)
{
  TaskManager *taskManager = static_cast<TaskManager *>(pvParameters);
  while (1)
  {
    if (Serial.available())
    {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command.length() > 0)
      {
        taskManager->_cmdProc->processCommand(command);
      }
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

void TaskManager::motorTaskFunction(void *pvParameters)
{
  TaskManager *taskManager = static_cast<TaskManager *>(pvParameters);

  while (1)
  {

    extern Motor motor1, motor2, motor3, motor4;
    motor1.update();
    motor2.update();
    motor3.update();
    motor4.update();

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void TaskManager::sensorTaskFunction(void *pvParameters)
{
  TaskManager *taskManager = static_cast<TaskManager *>(pvParameters);

  while (1)
  {
    if (taskManager->_cfg->sensorConsolePrintingEnabled)
    {
      taskManager->_sensorCtrl->readINA219();
      taskManager->_sensorCtrl->readTMP102();

      vTaskDelay(pdMS_TO_TICKS(taskManager->_cfg->sensorConsolePrintingInterval));
    }
    if (!digitalRead(BUTTON_PIN))
    {
      taskManager->_ledCtrl->runLedPattern(1);
    }
#ifndef ESP_STOP_SWITCH
    vTaskDelay(pdMS_TO_TICKS(500));
#endif
#ifdef ESP_STOP_SWITCH
    if (digitalRead(taskManager->_cfg->stopPin1) || digitalRead(taskManager->_cfg->stopPin2))
    {
      extern Motor motor3;
      motor3.stop();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
#endif
  }
}

// #ifdef ESP_WIFI_MQTT
// void MQTTTask(void *pvParameters)
// {
//   (void)pvParameters;

//   while (1)
//   {
//     if (!mqttClient.connected())
//     {
//       reconnectMQTT();
//     }
//     mqttClient.loop();

//     static unsigned long lastPublish = 0;
//     if (sensorReadingEnabled && millis() - lastPublish > sensorInterval)
//     {
//       publishStatus();
//       lastPublish = millis();
//     }

//     vTaskDelay(pdMS_TO_TICKS(100));
//   }
// }
// #endif