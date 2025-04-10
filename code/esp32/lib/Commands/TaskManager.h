
#ifndef TaskManager_h
#define TaskManager_h

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "MotorDriver.h"
#include "LedController.h"
#include "SensorController.h"
#include "CommandProcessor.h"
#include "Config.h"
#include "settings.h"

#ifdef ESP_WIFI_MQTT
#include "WiFiManager.h"
#include "MQTTManager.h"
#endif

class TaskManager
{
public:
    TaskManager();
    ~TaskManager();

    void begin(SensorController *sensorCtrl, LedController *ledCtrl, Config *cfg, CommandProcessor *cmdProc);

    void createMotorTask();
    void createSensorTask();
    void createCommandTask();

#ifdef ESP_WIFI_MQTT
    void createMqttWifiTask();
#endif

private:
    static void motorTaskFunction(void *pvParameters);
    static void sensorTaskFunction(void *pvParameters);
    static void commandTaskFunction(void *pvParameters);

#ifdef ESP_WIFI_MQTT
    static void mqttWifiTaskFunction(void *pvParameters);
#endif

    SensorController *_sensorCtrl;
    LedController *_ledCtrl;
    Config *_cfg;
    CommandProcessor *_cmdProc;

    TaskHandle_t _motorTaskHandle;
    TaskHandle_t _sensorTaskHandle;
    TaskHandle_t _commandTaskHandle;

#ifdef ESP_WIFI_MQTT
    TaskHandle_t _mqttWifiTaskHandle;
    WiFiManager *_wifiManager;
    MQTTManager *_mqttManager;
#endif
};

#endif