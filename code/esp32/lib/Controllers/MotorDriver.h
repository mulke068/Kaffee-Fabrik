
#ifndef MotorDriver_h
#define MotorDriver_h

#include <Arduino.h> // TODO: remove this dependency
#include "GlobalConfig.h"

enum MotorDirection
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2
};

class Motor
{
public:
  Motor(uint8_t enablePin, uint8_t in1, uint8_t in2);
  // virtual ~Motor();
  ~Motor();
  void begin();
  void updateDirection(MotorDirection dir);
  void updateSpeed(int speed, bool immediate = false);
  void update();
  void stop();
  bool isActive();
  int getSpeed();
  MotorDirection getDirection();

  void printStatus();

private:
  void setDirection();
  void setSpeed();

  uint8_t _enPin;
  uint8_t _in1Pin;
  uint8_t _in2Pin;

  int _currentSpeed;
  int _targetSpeed;

  bool _pendingDirectionChange;
  MotorDirection _pendingDirection;
  int _orginalTargetSpeed;

  int _rampStep;
  bool _isRunning;
  MotorDirection _direction;
};

#endif