/**
 * @file TaskManager.h
 * @author Kevin Muller (@kevbcef.com)
 * @brief Task manager for handling various tasks in the system.
 * @version 1.0
 * @date 2025-04-16
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef TaskManager_h
#define TaskManager_h

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "MotorDriver.h"
#include "LedController.h"
#include "SensorController.h"
#include "CommandProcessor.h"
#include "Config.h"
#include "HardwareConfig.h"
#include "GlobalConfig.h"

class TaskManager
{
public:
    TaskManager();
    ~TaskManager();

    void begin(SensorController *sensorCtrl, LedController *ledCtrl, Config *cfg, CommandProcessor *cmdProc);

    void createMotorTask();
    void createSensorTask();
    void createCommandTask();

    TaskHandle_t _motorTaskHandle;
    TaskHandle_t _sensorTaskHandle;
    TaskHandle_t _commandTaskHandle;

private:
    static void motorTaskFunction(void *pvParameters);
    static void sensorTaskFunction(void *pvParameters);
    static void commandTaskFunction(void *pvParameters);

    SensorController *_sensorCtrl;
    LedController *_ledCtrl;
    Config *_cfg;
    CommandProcessor *_cmdProc;
};

#endif // TaskManager_h