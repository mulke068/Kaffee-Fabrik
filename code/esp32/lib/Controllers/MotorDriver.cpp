/**
 * @file MotorDriver.cpp
 * @author Kevin Muller (@kevbchef.com)
 * @brief Motor driver class for controlling DC motors using PWM and direction pins.
 * @version 1.0
 * @date 2025-04-16
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "MotorDriver.h"

/**
 * @brief Construct a new Motor:: Motor object
 *
 * @param enablePin Pin for enabling the motor (PWM pin)
 * @param in1 Pin for direction control (input 1)
 * @param in2 Pin for direction control (input 2)
 */
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

/**
 * @brief Destroy the Motor:: Motor object
 *
 */
Motor::~Motor()
{
    // Destructor implementation (if needed)
}

/**
 * @brief Initialize the motor driver and stop the motor at startup.
 *
 */
void Motor::begin()
{
    pinMode(_enPin, OUTPUT);
    pinMode(_in1Pin, OUTPUT);
    pinMode(_in2Pin, OUTPUT);
    stop();
}

/**
 * @brief Update the motor direction and speed.
 *
 * @param dir The desired motor direction (FORWARD, BACKWARD, or STOP).
 */
void Motor::updateDirection(MotorDirection dir)
{
    if (_direction != dir)
    {
        if (_currentSpeed > 0)
        {
            _pendingDirectionChange = true;
            _pendingDirection = dir;
            _orginalTargetSpeed = _targetSpeed;
            updateSpeed(0, false);
        }
        else
        {
            _direction = dir;
            setDirection();
        }
        return;
    }
}

/**
 * @brief Update the motor speed.
 *
 * @param speed The desired motor speed (0 to 100%).
 * @param immediate If true, set the speed immediately without ramping.
 */
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
        return;
    }
    else
    {
        _targetSpeed = speed;
    }
}

/**
 * @brief Update the motor state (speed and direction) based on the current settings.
 *
 * This function adjusts the motor's speed and direction according to the current settings and handles any pending direction changes.
 *
 * This function is called periodically to ensure the motor operates smoothly and responds to any changes in speed or direction.
 *
 */
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
    }
}

/**
 * @brief Stop the motor and set its direction to STOP.
 *
 * This function sets the motor's speed to 0 and updates its direction to STOP.
 *
 */
void Motor::stop()
{
    updateDirection(STOP);
    updateSpeed(0, true);
    return;
}

/**
 * @brief Set the motor direction based on the current direction setting.
 *
 * This function sets the motor's direction pins (IN1 and IN2) according to the current direction setting.
 *
 */
void Motor::setDirection()
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
    return;
}

/**
 * @brief Set the motor speed using PWM.
 *
 * This function sets the motor's speed using PWM on the enable pin (EN).
 *
 */
void Motor::setSpeed()
{
    analogWrite(_enPin, _currentSpeed);
    return;
}

/**
 * @brief Get the current state of the motor (running or stopped).
 *
 * @return true if the motor is running, false if it is stopped.
 */
bool Motor::isActive()
{
    return _isRunning;
}

/**
 * @brief Get the current speed of the motor as a percentage.
 *
 * @return The current speed of the motor (0 to 100%).
 */
int Motor::getSpeed()
{
    return map(_currentSpeed, 0, 255, 0, 100);
}

/**
 * @brief Get the current direction of the motor.
 *
 * @return The current direction of the motor (FORWARD, BACKWARD, or STOP).
 */
MotorDirection Motor::getDirection()
{
    return _direction;
}

/**
 * @brief Print the current status of the motor, including direction and speed.
 *
 */
void Motor::printStatus()
{
    char dirStr[9] = {0};
    switch (_direction)
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