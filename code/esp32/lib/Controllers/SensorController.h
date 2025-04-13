
#ifndef SensorController_h
#define SensorController_h

#include <Adafruit_INA219.h>
#include <Wire.h>
#include "Config.h"
#include "HardwareConfig.h"
#include "GlobalConfig.h"

class SensorController
{
public:
    SensorController();
    ~SensorController();

    void begin();

    void readINA219();
    void readTMP102();

    float getBusVoltage() const { return _busVoltage; };
    float getShuntVoltage() const { return _shuntVoltage; };
    float getLoadVoltage() const { return _loadVoltage; };
    float getCurrentMA() const { return _current_mA; };
    float getPowerMW() const { return _power_mW; };
    float getMotorDriverTemp() const { return _motorDriverTemp; };
    float getPowerUnitTemp() const { return _powerUnitTemp; };
    bool getMotorDriverAlert();
    bool getPowerUnitAlert();

private:
    void initINA219();
    void initTMP102();
    float readTMP102Temp(uint8_t address);

    Adafruit_INA219 _ina219;

    float _busVoltage;
    float _shuntVoltage;
    float _loadVoltage;
    float _current_mA;
    float _power_mW;
    float _motorDriverTemp;
    float _powerUnitTemp;
};

#endif