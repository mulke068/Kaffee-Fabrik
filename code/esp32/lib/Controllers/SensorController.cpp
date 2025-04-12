
#include "SensorController.h"

SensorController::SensorController()
{
  // Constructor implementation (if needed)
  _busVoltage = 0.0;
  _shuntVoltage = 0.0;
  _loadVoltage = 0.0;
  _current_mA = 0.0;
  _power_mW = 0.0;
  _motorDriverTemp = 0.0;
  _powerUnitTemp = 0.0;
  _ina219 = Adafruit_INA219(INA219_ADDR);
}

SensorController::~SensorController()
{
  // Destructor implementation (if needed)
  _ina219.~Adafruit_INA219();
}

void SensorController::begin()
{
  initINA219();
  initTMP102();
  Serial.println("SensorController initialized");
}

void SensorController::initINA219()
{
  if (!_ina219.begin())
  {
    Serial.println("Failed to find INA219 chip");
  }
  else
  {
    Serial.println("INA219 initialized successfully");
  }
}

void SensorController::initTMP102()
{
  // Alert pins for TMP102 sensors
  pinMode(TMP102_IC20_ALERT, INPUT_PULLUP);
  pinMode(TMP102_IC8_ALERT, INPUT_PULLUP);
  
  // Motor Driver Temp Sensor: Set T_HIGH (70°C) and T_LOW (0°C)
  Wire.beginTransmission(TMP102_IC20_ADDR);
  Wire.write(0x03);
  Wire.write(0x46);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(TMP102_IC20_ADDR);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();

  // Power Unit Temp Sensor: Set T_HIGH (60°C) and T_LOW (5°C)
  Wire.beginTransmission(TMP102_IC8_ADDR);
  Wire.write(0x03);
  Wire.write(0x3C);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(TMP102_IC8_ADDR);
  Wire.write(0x02);
  Wire.write(0x08);
  Wire.write(0x00);
  Wire.endTransmission();

  Serial.println("TMP102 sensors initialized");
}

void SensorController::readINA219()
{
  _shuntVoltage = _ina219.getShuntVoltage_mV();
  _busVoltage = _ina219.getBusVoltage_V();
  _current_mA = _ina219.getCurrent_mA();
  _power_mW = _ina219.getPower_mW();
  _loadVoltage = _busVoltage + (_shuntVoltage / 1000);

  Serial.println("===== Power Readings =====");
  Serial.print("Bus Voltage:   ");
  Serial.print(_busVoltage);
  Serial.println(" V");
  Serial.print("Shunt Voltage: ");
  Serial.print(_shuntVoltage);
  Serial.println(" mV");
  Serial.print("Load Voltage:  ");
  Serial.print(_loadVoltage);
  Serial.println(" V");
  Serial.print("Current:       ");
  Serial.print(_current_mA);
  Serial.println(" mA");
  Serial.print("Power:         ");
  Serial.print(_power_mW);
  Serial.println(" mW");
}

void SensorController::readTMP102()
{
  _motorDriverTemp = readTMP102Temp(TMP102_IC20_ADDR);
  _powerUnitTemp = readTMP102Temp(TMP102_IC8_ADDR);

  Serial.println("===== Temperature Readings =====");
    Serial.print("Motor Driver Temp: ");
    Serial.print(_motorDriverTemp);
    Serial.print("°C (Alert: ");
    Serial.print(getMotorDriverAlert() ? "Yes" : "NO");
    Serial.println(")");

    Serial.print("Power Unit Temp:   ");
    Serial.print(_powerUnitTemp);
    Serial.print("°C (Alert: ");
    Serial.print(getPowerUnitAlert() ? "YES" : "NO");
    Serial.println(")");
}

float SensorController::readTMP102Temp(uint8_t address) {
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
  return !digitalRead(TMP102_IC20_ALERT);
}

bool SensorController::getPowerUnitAlert()
{
  return !digitalRead(TMP102_IC8_ALERT);
}