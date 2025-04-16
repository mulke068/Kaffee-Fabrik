/**
 * @file SensorController.cpp
 * @author Kevin Muller (@kevbcef.com)
 * @brief This source file implements the SensorController class for managing sensor operations.
 * @version 1.0
 * @date 2025-04-16
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "SensorController.h"

/**
 * @brief Construct a new Sensor Controller:: Sensor Controller object
 *
 */
SensorController::SensorController()
{
  _busVoltage = 0.0;
  _shuntVoltage = 0.0;
  _loadVoltage = 0.0;
  _current_mA = 0.0;
  _power_mW = 0.0;
  _motorDriverTemp = 0.0;
  _powerUnitTemp = 0.0;
  _ina219 = Adafruit_INA219(SensorAdress::INA219);
}

/**
 * @brief Destroy the Sensor Controller:: Sensor Controller object
 *
 */
SensorController::~SensorController()
{
  _ina219.~Adafruit_INA219();
}

/**
 * @brief Initializes the INA219 and TMP102 sensors.
 *
 * This function initializes the INA219 sensor and sets up the TMP102 sensors with specific configurations.
 * It also sets the alert pins for the TMP102 sensors to input mode with pull-up resistors.
 */
void SensorController::begin()
{
  initINA219();
  initTMP102();
  Serial.write("SensorController initialized\n");
}

/**
 * @brief Initializes the INA219 sensor.
 *
 * This function checks if the INA219 sensor is connected and initializes it.
 * If the sensor is not found, an error message is printed to the serial monitor.
 */
void SensorController::initINA219()
{
  if (!_ina219.begin())
  {
    Serial.write("Failed to find INA219 chip\n");
  }
  else
  {
    Serial.write("INA219 initialized successfully\n");
  }
}

/**
 * @brief Initializes the TMP102 sensors.
 *
 * This function sets the alert pins for the TMP102 sensors to input mode with pull-up resistors.
 * It also configures the temperature thresholds for the sensors.
 */
void SensorController::initTMP102()
{
  // Alert pins for TMP102 sensors
  pinMode(SensorAdress::Alert::TMP102_IC20, INPUT_PULLUP);
  pinMode(SensorAdress::Alert::TMP102_IC8, INPUT_PULLUP);

  // Motor Driver Temp Sensor: Set T_HIGH (70°C) and T_LOW (0°C)
  Wire.beginTransmission(SensorAdress::TMP102_IC20);
  Wire.write(0x03);
  Wire.write(0x46);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(SensorAdress::TMP102_IC20);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();

  // Power Unit Temp Sensor: Set T_HIGH (60°C) and T_LOW (5°C)
  Wire.beginTransmission(SensorAdress::TMP102_IC8);
  Wire.write(0x03);
  Wire.write(0x3C);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(SensorAdress::TMP102_IC8);
  Wire.write(0x02);
  Wire.write(0x08);
  Wire.write(0x00);
  Wire.endTransmission();

  Serial.write("TMP102 sensors initialized\n");
}

/**
 * @brief Reads the INA219 sensor values and prints them to the serial monitor.
 *
 * This function reads the bus voltage, shunt voltage, load voltage, current, and power from the INA219 sensor.
 * It then prints these values to the serial monitor.
 */
void SensorController::readINA219()
{
  _shuntVoltage = _ina219.getShuntVoltage_mV();
  _busVoltage = _ina219.getBusVoltage_V();
  _current_mA = _ina219.getCurrent_mA();
  _power_mW = _ina219.getPower_mW();
  _loadVoltage = _busVoltage + (_shuntVoltage / 1000);

  Serial.write("===== Power Readings =====\n");
  Serial.print("Bus Voltage:    ");
  Serial.print(_busVoltage, 2);
  Serial.print(" V\n");
  Serial.print("Shunt Voltage:  ");
  Serial.print(_shuntVoltage, 2);
  Serial.print(" mV\n");
  Serial.print("Load Voltage:   ");
  Serial.print(_loadVoltage, 2);
  Serial.print(" V\n");
  Serial.print("Current:        ");
  Serial.print(_current_mA, 2);
  Serial.print(" mA\n");
  Serial.print("Power:          ");
  Serial.print(_power_mW, 2);
  Serial.print(" mW\n");
}

/**
 * @brief Reads the temperature from the TMP102 sensors and prints them to the serial monitor.
 *
 * This function reads the temperature from the motor driver and power unit TMP102 sensors.
 * It then prints these values along with their alert status to the serial monitor.
 */
void SensorController::readTMP102()
{
  _motorDriverTemp = readTMP102Temp(SensorAdress::TMP102_IC20);
  _powerUnitTemp = readTMP102Temp(SensorAdress::TMP102_IC8);

  Serial.write("===== Temperature Readings =====\n");
  Serial.print("Motor Driver Temp: ");
  Serial.print(_motorDriverTemp, 2);
  Serial.print(" °C (Alert: ");
  Serial.print(getMotorDriverAlert() ? "YES" : "NO");
  Serial.print(")\n");

  Serial.print("Power Unit Temp:   ");
  Serial.print(_powerUnitTemp, 2);
  Serial.print(" °C (Alert: ");
  Serial.print(getPowerUnitAlert() ? "YES" : "NO");
  Serial.print(")\n");
}

/**
 * @brief Reads the temperature from a TMP102 sensor at the specified address.
 *
 * @param address The I2C address of the TMP102 sensor.
 * @return float The temperature in degrees Celsius.
 */
float SensorController::readTMP102Temp(uint8_t address)
{
  Wire.beginTransmission(address);
  Wire.write(0x00);
  Wire.endTransmission();
  // Needed to set inside the static_cast<uint8_t> to avoid ambiguity between int and uint8_t
  Wire.requestFrom(static_cast<uint8_t>(address), static_cast<uint8_t>(2));

  if (Wire.available() == 2)
  {
    byte msb = Wire.read();
    byte lsb = Wire.read();
    int temperature = ((msb << 8) | lsb) >> 4;
    return temperature * 0.0625;
  }
  return 0.0;
}

bool SensorController::getMotorDriverAlert()
{
  return !digitalRead(SensorAdress::Alert::TMP102_IC20);
}

/**
 * @brief Checks if the power unit TMP102 sensor is in alert state.
 *
 * This function reads the alert pin of the power unit TMP102 sensor and returns
 * true if the sensor is in alert state, indicating a temperature issue.
 *
 * @return true if the power unit TMP102 sensor is in alert state, false otherwise.
 */
bool SensorController::getPowerUnitAlert()
{
  return !digitalRead(SensorAdress::Alert::TMP102_IC8);
}