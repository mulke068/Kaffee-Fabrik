
#include "LedController.h"

LedController::LedController()
{
  // Constructor implementation (if needed)
}

LedController::~LedController()
{
  // Destructor implementation (if needed)
}

/**
 * @brief Initialize the LED controller
 **/
void LedController::begin()
{
  Wire.beginTransmission(SensorAdress::PCA9555);
  Wire.write(0b00000110);
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(SensorAdress::PCA9555);
  Wire.write(0b00000111);
  Wire.write(0b00000000);
  Wire.endTransmission();

  setLeds(0b00000000, 0b00000000);
  pinMode(LedAdress::led17, OUTPUT);
  pinMode(LedAdress::led18, OUTPUT);
  digitalWrite(LedAdress::led17, HIGH);
  digitalWrite(LedAdress::led18, HIGH);
  Serial.write("PCA9555 initialized\n");
}

/**
 * @brief Set the state of the LEDs
 * @param newPort0State  New state for port 0 (first 8 LEDs)
 * @param newPort1State  New state for port 1 (last 8 LEDs)
 **/
void LedController::setLeds(byte newPort0State, byte newPort1State)
{
  _port0State = newPort0State;
  _port1State = newPort1State;

  Wire.beginTransmission(SensorAdress::PCA9555);
  Wire.write(0b00000010);
  Wire.write(~_port0State);
  Wire.endTransmission();

  Wire.beginTransmission(SensorAdress::PCA9555);
  Wire.write(0b00000011);
  Wire.write(~_port1State);
  Wire.endTransmission();
}

/**
 * @brief Run a specific LED pattern
 * @param patternNum  Pattern number (1-3)
 **/
void LedController::runLedPattern(int patternNum)
{
  switch (patternNum)
  {
  case 1: // Same Color ON OFF Pattern
    setLeds(LedAdress::led1 | LedAdress::led4, LedAdress::led10 | LedAdress::led7);
    vTaskDelay(pdMS_TO_TICKS(1000));
    setLeds(LedAdress::led2 | LedAdress::led5, LedAdress::led11 | LedAdress::led8);
    vTaskDelay(pdMS_TO_TICKS(1000));
    setLeds(LedAdress::led3 | LedAdress::led6, LedAdress::led12 | LedAdress::led9);
    vTaskDelay(pdMS_TO_TICKS(1000));
    setLeds(0b00000000, 0b00000000);
    break;

  case 2: // Sequential Pattern
    for (int i = 0; i < 6; i++)
    {
      setLeds(1 << i, 0);
      vTaskDelay(pdMS_TO_TICKS(300));
    }
    for (int i = 0; i < 6; i++)
    {
      setLeds(0, 1 << i);
      vTaskDelay(pdMS_TO_TICKS(300));
    }
    setLeds(0b00000000, 0b00000000);
    break;

  case 3: // All on then off
    setLeds(0xFF, 0xFF);
    vTaskDelay(pdMS_TO_TICKS(1000));
    setLeds(0b00000000, 0b00000000);
    break;

  default:
    break;
  }
}

/**
 * @brief Turn on all LEDs
 **/
void LedController::setAllOn()
{
  setLeds(0xFF, 0xFF);
  digitalWrite(LedAdress::led17, LOW);
  digitalWrite(LedAdress::led18, LOW);
  Serial.write("All LEDs on\n");
}

/**
 * @brief  Turn off all LEDs
 **/
void LedController::setAllOff()
{
  setLeds(0b00000000, 0b00000000);
  digitalWrite(LedAdress::led17, HIGH);
  digitalWrite(LedAdress::led18, HIGH);
  Serial.write("All LEDs off\n");
}

/**
 * @brief  Toggle the state of a specific LED
 * @param ledNum  LED number (1-12, 17, or 18)
 **/
void LedController::toggleLed(int ledNum)
{
  byte ledmask;
  byte *portState = nullptr;
  if (getLedMaskAndPort(ledNum, ledmask, portState))
  {
    *portState ^= ledmask;
    setLeds(_port0State, _port1State);
    Serial.print("LED ");
    Serial.print(ledNum);
    Serial.print(" toggled to ");
    Serial.println((*portState & ledmask) ? "ON" : "OFF");
  }
  else if (ledNum == 17 || ledNum == 18)
  {
    Serial.print("LED ");
    Serial.print(ledNum);
    Serial.print(" toggled to ");
    Serial.println(digitalRead(ledNum) == HIGH ? "OFF" : "ON");
    switch (ledNum)
    {
    case 17:
      ledNum = LedAdress::led17;
      break;
    case 18:
      ledNum = LedAdress::led18;
      break;
    }
    digitalWrite(ledNum, digitalRead(ledNum) == HIGH ? LOW : HIGH);
  }
  else
  {
    Serial.write("Invalid LED number. Must be between 1 and 12, or 17 or 18.\n");
    return;
  }
}

/**
 * @brief  Set the state of a specific LED
 * @param ledNum  LED number (1-12, 17, or 18)
 * @param state  State of the LED (true for ON, false for OFF)
 **/
void LedController::setLed(int ledNum, bool state)
{
  byte ledmask;
  byte *portState = nullptr;
  if (getLedMaskAndPort(ledNum, ledmask, portState))
  {
    if (state)
    {
      *portState |= ledmask;
      Serial.print("LED ");
      Serial.print(ledNum);
      Serial.print(" turned ON\n");
    }
    else
    {
      *portState &= ~ledmask;
      Serial.print("LED ");
      Serial.print(ledNum);
      Serial.print(" turned OFF\n");
    }
    setLeds(_port0State, _port1State);
  }
  else if (ledNum == 17 || ledNum == 18)
  {
    switch (ledNum)
    {
    case 17:
      ledNum = LedAdress::led17;
      break;
    case 18:
      ledNum = LedAdress::led18;
      break;
    }
    digitalWrite(ledNum, state ? LOW : HIGH);
    Serial.print("LED ");
    Serial.print(ledNum);
    Serial.print(" turned ");
    Serial.println(state ? "ON" : "OFF");
  }
  else
  {
    Serial.write("Invalid LED number. Must be between 1 and 12, or 17 or 18.\n");
    return;
  }
}

/**
 * @brief  Get the LED mask and port state for a specific LED number
 * @param ledNum  LED number (1-12, 17, or 18)
 * @param ledmask  Reference to store the LED mask
 * @param portState  Reference to store the port state pointer
 * @return true if valid LED number, false otherwise
 **/
bool LedController::getLedMaskAndPort(int ledNum, byte &ledmask, byte *&portState)
{
  // ledNum is 1-12, state is true or false but also 18 and 17 are valid ledNum
  if (ledNum >= 1 && ledNum <= 12)
  {
    if (ledNum <= 6)
    {
      // Why do we need to shift by 1? Because led1 is 0b00000001, led2 is 0b00000010, led3 is 0b00000100, etc.
      ledmask = 1 << (ledNum - 1);
      portState = &_port0State;
    }
    else if (ledNum >= 7)
    {
      ledmask = 1 << (12 - ledNum);
      portState = &_port1State;
    }
    return true;
  }
  else
  {
    return false;
  }
}