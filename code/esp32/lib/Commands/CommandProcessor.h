
#ifndef CommandProcessor_h
#define CommandProcessor_h

#include "MotorDriver.h"
#include "LedController.h"
#include "SensorController.h"
#include "Config.h"
// #include <esp_heap_caps.h>
// #include <Arduino.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>


class CommandProcessor
{
public:
    CommandProcessor();
    ~CommandProcessor();

    void begin(SensorController *sensorCtrl, LedController *ledCtrl, Config *cfg);

    void processCommand(const String &command);

    void printHelp();
    void printStatus();
    void printSystemMonitor();

private:
    Motor *_motor1;
    Motor *_motor2;
    Motor *_motor3;
    Motor *_motor4;

    SensorController *_sensorCtrl;
    LedController *_ledCtrl;
    Config *_cfg;

    void processMotorCommand(const String &motorIndex, const String &action, const String &value);
    void processLedCommand(const String &ledIndex, const String &action);
    void processSensorCommand(const String &action, const String &sensor, const String &value);

#ifdef ESP_STOP_SWITCH
    void processStopSwitchCommand(const String &action, const String &value);
#endif
};

#endif