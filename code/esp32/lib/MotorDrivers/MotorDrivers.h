
#ifndef MotorDrivers_h
#define MotorDrivers_h


#include "Arduino.h"

enum MotorDirection {
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2
};


class Motor {
public:
    Motor(uint8_t enablePin, uint8_t in1, uint8_t in2);
    virtual ~Motor();
    void begin();
    void setDirection(MotorDirection dir);
    void setSpeed(int speed, bool immediate = false);
    void update();
    void stop();
    bool isActive();
    int getSpeed();
    MotorDirection getDirection();

private:

    uint8_t _enPin;
    uint8_t _in1Pin;
    uint8_t _in2Pin;
    int _currentSpeed = 0;
    int _targetSpeed = 0;
    int _rampStep = 5;
    bool _isRunning = false;
    MotorDirection _direction = STOP;

    void updatePins();
    void updateSpeed();
};

#endif