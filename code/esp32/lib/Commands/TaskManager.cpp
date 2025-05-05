/**
 * @file TaskManager.cpp
 * @author Kevin Muller (@kevbcef.com)
 * @brief TaskManager class implementation for managing tasks in an ESP32 application.
 * This class handles the creation and management of tasks for motor control, sensor reading, and command processing.
 * @version 1.0
 * @date 2025-04-16
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "TaskManager.h"

/**
 * @brief Construct a new Task Manager:: Task Manager object
 *
 */
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

/**
 * @brief Destroy the Task Manager:: Task Manager object
 *
 */
TaskManager::~TaskManager()
{
  // Destructor implementation (if needed)
}

/**
 * @brief Initializes the TaskManager with the provided controllers and configuration.
 *
 * @param sensorCtrl Pointer to the SensorController instance.
 * @param ledCtrl Pointer to the LedController instance.
 * @param cfg Pointer to the Config instance.
 * @param cmdProc Pointer to the CommandProcessor instance.
 */
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

/**
 * @brief Creates the command task for processing commands.
 *
 * This task is responsible for reading commands from the serial input and processing them.
 *
 * @details The command task continuously checks for available serial input and processes
 * the commands received, allowing for dynamic control of the system.
 */
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

/**
 * @brief Creates the motor task for controlling motors.
 *
 * This task is responsible for updating the motor states at regular intervals.
 *
 * @details The motor task continuously updates the motor states, allowing for smooth
 * control of the motors in the system.
 */
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

/**
 * @brief Creates the sensor task for reading sensor data.
 *
 * This task is responsible for reading data from sensors at regular intervals and
 * printing it to the console if enabled in the configuration.
 *
 * @details The sensor task continuously reads data from the sensors and prints it to
 * the console if the corresponding configuration option is enabled.
 */
void TaskManager::createSensorTask()
{
  xTaskCreate(
      sensorTaskFunction,
      "SensorTask",
      2048, // 4069,
      this,
      2,
      &_sensorTaskHandle);
}

/**
 * @brief Command task function for processing commands.
 *
 * This function is executed in the command task and continuously checks for available
 * serial input to process commands.
 *
 * @param pvParameters Pointer to the TaskManager instance.
 */
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

/**
 * @brief Motor task function for updating motor states.
 *
 * This function is executed in the motor task and continuously updates the states of
 * the motors at regular intervals.
 *
 * @param pvParameters Pointer to the TaskManager instance.
 */
void TaskManager::motorTaskFunction(void *pvParameters)
{
  // TaskManager *taskManager = static_cast<TaskManager *>(pvParameters);

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

/**
 * @brief Sensor task function for reading sensor data.
 *
 * This function is executed in the sensor task and continuously reads data from the
 * sensors at regular intervals, printing it to the console if enabled in the configuration.
 *
 * @param pvParameters Pointer to the TaskManager instance.
 */
void TaskManager::sensorTaskFunction(void *pvParameters)
{
  TaskManager *taskManager = static_cast<TaskManager *>(pvParameters);
  uint32_t start_time = millis();
  bool timerFinished = true;
#ifdef ESP_STOP_SWITCH
  bool motorStopped = false;
#endif

  while (1)
  {
    if (taskManager->_cfg->sensorConsolePrintingEnabled)
    {
      if (!timerFinished)
      {
        uint32_t current_time = millis();
        uint32_t elapsed = current_time - start_time;
        timerFinished = elapsed > taskManager->_cfg->sensorConsolePrintingInterval;
      }
      else
      {
        timerFinished = false;
        start_time = millis();
        taskManager->_sensorCtrl->readINA219();
        taskManager->_sensorCtrl->readTMP102();
      }
    }
    if (!digitalRead(Hardware::BUTTON_PIN))
    {
      taskManager->_ledCtrl->runLedPattern(1);
    }
#ifndef ESP_STOP_SWITCH
    vTaskDelay(pdMS_TO_TICKS(500));
#endif
#ifdef ESP_STOP_SWITCH
    if (digitalRead(taskManager->_cfg->stopPin1) || digitalRead(taskManager->_cfg->stopPin2))
    // if (!digitalRead(taskManager->_cfg->stopPin1) || !digitalRead(taskManager->_cfg->stopPin2))
    {
      extern Motor motor4;
      if (!motorStopped) {
        Serial.println("STOP SWITCH TRIGGERED!");
        motor4.stop();
        motorStopped = true;
      }
      
    } else {
      motorStopped = false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
#endif
  }
}