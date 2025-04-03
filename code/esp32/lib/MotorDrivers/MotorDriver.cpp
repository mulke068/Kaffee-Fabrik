
#include <MotorDrivers.h>

Motor::Motor(uint8_t enablePin, uint8_t in1, uint8_t in2)
{
    _enPin = enablePin;
    _in1Pin = in1;
    _in2Pin = in2;
}

Motor::~Motor(){
    return; 
}

void Motor::begin()
{
    pinMode(_enPin, OUTPUT);
    pinMode(_in1Pin, OUTPUT);
    pinMode(_in2Pin, OUTPUT);
    stop();
}

void Motor::setDirection(MotorDirection dir)
{
    if (_direction != dir)
    {
        _direction = dir;
        updateSpeed();
        updatePins();
    }
}

void Motor::setSpeed(int speed, bool immediate)
{
    speed = constrain(map(speed, 0, 100, 0, 255), 0, 255);
    _targetSpeed = speed;
    if (immediate)
    {
        _currentSpeed = _targetSpeed;
        updateSpeed();
    }
}

void Motor::update()
{
    if (_currentSpeed != _targetSpeed)
    {
        int step = _rampStep;
        if (abs(_targetSpeed - _currentSpeed) < _rampStep)
        {
            step = abs(_targetSpeed - _currentSpeed);
        }
        _currentSpeed += (_targetSpeed > _currentSpeed) ? step : -step;
        updateSpeed();
    }
}

void Motor::stop()
{
    setDirection(STOP);
    setSpeed(0, true);
}

bool Motor::isActive()
{
    return _isRunning;
}

int Motor::getSpeed() 
{
    return map(_currentSpeed, 0, 255, 0, 100);
}

MotorDirection Motor::getDirection() 
{
    return _direction;
}

void Motor::updatePins()
{
    switch (_direction)
    {
    case FORWARD:
        digitalWrite(_in1Pin, HIGH);
        digitalWrite(_in2Pin, LOW);
        break;
    case BACKWARD:
        digitalWrite(_in1Pin, LOW);
        digitalWrite(_in2Pin, HIGH);
        break;
    default:
        digitalWrite(_in1Pin, LOW);
        digitalWrite(_in2Pin, LOW);
    }
    _isRunning = (_direction != STOP);
}

void Motor::updateSpeed()
{
    analogWrite(_enPin, _currentSpeed);
}