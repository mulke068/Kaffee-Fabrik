
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
  _ina219 = Adafruit_INA219(SensorAdress::INA219);
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
  Serial.write("SensorController initialized\n");
}

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

bool SensorController::getPowerUnitAlert()
{
  return !digitalRead(SensorAdress::Alert::TMP102_IC8);
}