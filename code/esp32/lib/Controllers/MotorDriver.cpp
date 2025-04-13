#include <MotorDriver.h>

Motor::Motor(uint8_t enablePin, uint8_t in1, uint8_t in2)
{
    _enPin = enablePin;
    _in1Pin = in1;
    _in2Pin = in2;

    _currentSpeed = 0;
    _targetSpeed = 0;
    _direction = STOP;

    _pendingDirectionChange = false;
    _pendingDirection = STOP;
    _orginalTargetSpeed = 0;

    _rampStep = 2;
    _isRunning = false;
}

Motor::~Motor()
{
    // Destructor implementation (if needed)
}

void Motor::begin()
{
    pinMode(_enPin, OUTPUT);
    pinMode(_in1Pin, OUTPUT);
    pinMode(_in2Pin, OUTPUT);
    stop();
}

void Motor::updateDirection(MotorDirection dir)
{
    if (_direction != dir)
    {
        if (_currentSpeed > 0)
        {
            // Serial.println("Motor direction change requested, current speed: " + String(_currentSpeed) + ", target speed: " + String(_targetSpeed));
            _pendingDirectionChange = true;
            _pendingDirection = dir;
            _orginalTargetSpeed = _targetSpeed;
            updateSpeed(0, false);
        }
        else
        {
            _direction = dir;
            setDirection();
            // Serial.println("Motor direction changed to " + String(_pendingDirection));
        }
        return;
    }
}

void Motor::updateSpeed(int speed, bool immediate)
{
    speed = constrain(map(speed, 0, 100, 0, 255), 0, 255);
    if (immediate)
    {
        _targetSpeed = speed;
        _currentSpeed = speed;
        setSpeed();
        return;
    }
    else if (_pendingDirectionChange)
    {
        _targetSpeed = 0;
        _orginalTargetSpeed = speed;
        // Serial.println("Motor pending direction change, speed set to " + String(_orginalTargetSpeed) + " (target: " + String(_targetSpeed) + ")");
        return;
    }
    else
    {
        _targetSpeed = speed;
        // Serial.println("Motor speed set to " + String(_targetSpeed) + " (target: " + String(_targetSpeed) + ")");
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
        setSpeed();
    }

    if (_pendingDirectionChange && _currentSpeed <= 5)
    {

        _currentSpeed = 0;
        setSpeed();

        vTaskDelay(pdMS_TO_TICKS(50));

        _direction = _pendingDirection;
        setDirection();

        _pendingDirectionChange = false;
        _targetSpeed = _orginalTargetSpeed;

        // Serial.println("Direction change completed to " + String(_direction == FORWARD ? "FORWARD" : (_direction == BACKWARD ? "BACKWARD" : "STOP")) + ", resuming to speed: " + String(map(_targetSpeed, 0, 255, 0, 100)) + "%");
    }
}

void Motor::stop()
{
    updateDirection(STOP);
    updateSpeed(0, true);
    return;
}

void Motor::setDirection()
{
    // Serial.println("Setting physical direction to: " + String(_direction == FORWARD ? "FORWARD" : (_direction == BACKWARD ? "BACKWARD" : "STOP")));
    switch (_direction)
    {
    case FORWARD:
        digitalWrite(_in1Pin, HIGH);
        digitalWrite(_in2Pin, LOW);
        // Serial.println("Setting pins: IN1=HIGH, IN2=LOW");
        break;
    case BACKWARD:
        digitalWrite(_in1Pin, LOW);
        digitalWrite(_in2Pin, HIGH);
        // Serial.println("Setting pins: IN1=LOW, IN2=HIGH");
        break;
    default:
        digitalWrite(_in1Pin, LOW);
        digitalWrite(_in2Pin, LOW);
        // Serial.println("Setting pins: IN1=LOW, IN2=LOW");
    }
    _isRunning = (_direction != STOP);
    return;
}

void Motor::setSpeed()
{
    analogWrite(_enPin, _currentSpeed);
    return;
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

void Motor::printStatus()
{
    char dirStr[9] = {0};
    switch (getDirection())
    {
    case FORWARD:
        strcpy(dirStr, "FORWARD");
        break;
    case BACKWARD:
        strcpy(dirStr, "BACKWARD");
        break;
    case STOP:
        strcpy(dirStr, "STOPPED");
        break;
    default:
        strcpy(dirStr, "Unknown");
        break;
    }
    int speedPercent = map(_currentSpeed, 0, 255, 0, 100);
    Serial.print("Motor status: Direction= ");
    Serial.print(dirStr);
    Serial.print(", Speed=");
    Serial.print(speedPercent);
    Serial.print("%\n");
    dirStr[0] = '\0';
    return;
}