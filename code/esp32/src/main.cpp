#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Preferences.h>
#include "MotorDrivers.h"
#include "settings.h"

#define ESP_WIFI_MQTT
#define ESP_STOP_SWITCH

#ifdef ESP_WIFI_MQTT

#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient espClient;
PubSubClient mqttClient(espClient);

#endif

Motor motor1(PWMA_1, AIN1_1, AIN2_1);
Motor motor2(PWMB_1, BIN1_1, BIN2_1);
Motor motor3(PWMA_2, AIN1_2, AIN2_2);
Motor motor4(PWMB_2, BIN1_2, BIN2_2);
Preferences preferences;

void initINA219();
void initTMP102();
void initPCA9555();
void initMotors();
void readINA219();
void readTMP102();
void setLeds(byte port0, byte port1);
void runLedPattern(int patternNum);
void processCommand(String command);
void printHelp();
void printStatus();
void loadSettings();
void saveSettings();
void setupWiFi();
void setupMQTT();
void reconnectMQTT();
void publishStatus();
void mqttCallback(char *topic, byte *payload, unsigned int length);
void processWiFiCommand(String command);
void processMQTTCommand(String command);
void printNetworkSettings();

void MotorTask(void *pvParameters)
{
  (void)pvParameters;
  while (1)
  {

    motor1.update();
    motor2.update();
    motor3.update();
    motor4.update();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void SensorTask(void *pvParameters)
{
  (void)pvParameters;
  while (1)
  {
    if (sensorReadingEnabled && autoUpdateEnabled)
    {
      readINA219();
      readTMP102();
      vTaskDelay(pdMS_TO_TICKS(sensorInterval));
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(200));
    }

  }
}

void SerialTask(void *pvParameters)
{
  (void)pvParameters;
  while (1)
  {
    if (Serial.available())
    {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command.length() > 0)
      {
        processCommand(command);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void LedTask(void *pvParameters)
{
  (void)pvParameters;
  while (1)
  {
    #ifdef ESP_STOP_SWITCH
    {
      if (digitalRead(stopPin1) || digitalRead(stopPin2)) {
        motor3.stop();
      }
    }
    #endif
    if (!digitalRead(BUTTON_PIN))
    {
      runLedPattern(1);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

#ifdef ESP_WIFI_MQTT
void MQTTTask(void *pvParameters)
{
  (void)pvParameters;

  while (1)
  {
    if (!mqttClient.connected())
    {
      reconnectMQTT();
    }
    mqttClient.loop();

    static unsigned long lastPublish = 0;
    if (sensorReadingEnabled && millis() - lastPublish > sensorInterval)
    {
      publishStatus();
      lastPublish = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
#endif

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);

  loadSettings();

  initINA219();
  initTMP102();
  initPCA9555();
  initMotors();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_17, OUTPUT);
  pinMode(LED_18, OUTPUT);
  digitalWrite(LED_17, HIGH);
  digitalWrite(LED_18, HIGH);

#ifdef ESP_STOP_SWITCH
  pinMode(stopPin1, INPUT);
  pinMode(stopPin2, INPUT);
#endif

#if defined(BOARD_HAS_PSRAM)
  if (psramFound())
  {
    Serial.println("PSRAM is initialized");
  }
  else
  {
    Serial.println("PSRAM failed to initialise");
  }
#endif

  Serial.println("\n===== Integrated System Controller (RTOS) =====");
  Serial.println("Type 'HELP' for available commands");
  printHelp();

  xTaskCreate(MotorTask, "MotorTask", 2048, NULL, 2, NULL);
  xTaskCreate(SensorTask, "SensorTask", 2048, NULL, 2, NULL);
  xTaskCreate(SerialTask, "SerialTask", 4096, NULL, 2, NULL);
  xTaskCreate(LedTask, "LedTask", 2048, NULL, 2, NULL);

#ifdef ESP_WIFI_MQTT
  setupWiFi();

  setupMQTT();

  xTaskCreate(MQTTTask, "MQTTTask", 4096, NULL, 1, NULL);
#endif
}

void loop()
{
  vTaskDelete(NULL);
}

void initINA219()
{
  if (!ina219.begin())
  {
    Serial.println("Failed to find INA219 chip");
  }
  else
  {
    Serial.println("INA219 initialized successfully");
  }
}

void loadSettings()
{
  preferences.begin("controller", false); // Read-only mode

#ifdef ESP_WIFI_MQTT
  // WiFi settings
  ssid = preferences.getString("ssid", ssid);
  password = preferences.getString("password", password);
  useStaticIP = preferences.getBool("useStaticIP", useStaticIP);

  // Read IP settings if static IP is enabled
  if (useStaticIP)
  {
    staticIP.fromString(preferences.getString("staticIP", "0.0.0.0"));
    gateway.fromString(preferences.getString("gateway", "0.0.0.0"));
    subnet.fromString(preferences.getString("subnet", "255.255.255.0"));
    dns1.fromString(preferences.getString("dns1", "8.8.8.8"));
    dns2.fromString(preferences.getString("dns2", "8.8.4.4"));
  }

  // MQTT settings
  mqttServer = preferences.getString("mqttServer", mqttServer);
  mqttPort = preferences.getInt("mqttPort", mqttPort);
  mqttUser = preferences.getString("mqttUser", mqttUser);
  mqttPassword = preferences.getString("mqttPassword", mqttPassword);
  mqttClientId = preferences.getString("mqttClientId", mqttClientId);
  mqttBaseTopic = preferences.getString("mqttBaseTopic", mqttBaseTopic);

#endif
#ifdef ESP_STOP_SWITCH
  stopPin1 = preferences.getInt("stopPin1", stopPin1);
  stopPin2 = preferences.getInt("stopPin2", stopPin2);
#endif

  preferences.end();

  Serial.println("Settings loaded from preferences");
}

void saveSettings()
{
  preferences.begin("controller", false);

#ifdef ESP_WIFI_MQTT
  // WiFi settings
  preferences.putString("ssid", ssid);
  preferences.putString("password", password);
  preferences.putBool("useStaticIP", useStaticIP);

  // IP settings
  if (useStaticIP)
  {
    preferences.putString("staticIP", staticIP.toString());
    preferences.putString("gateway", gateway.toString());
    preferences.putString("subnet", subnet.toString());
    preferences.putString("dns1", dns1.toString());
    preferences.putString("dns2", dns2.toString());
  }

  // MQTT settings
  preferences.putString("mqttServer", mqttServer);
  preferences.putInt("mqttPort", mqttPort);
  preferences.putString("mqttUser", mqttUser);
  preferences.putString("mqttPassword", mqttPassword);
  preferences.putString("mqttClientId", mqttClientId);
  preferences.putString("mqttBaseTopic", mqttBaseTopic);
#endif
#ifdef ESP_STOP_SWITCH
  preferences.putInt("stopPin1", stopPin1);
  preferences.putInt("stopPin2", stopPin2);
#endif

  preferences.end();

  Serial.println("Settings saved to preferences");
}

void initTMP102()
{
  pinMode(TMP102_IC20_ALERT, INPUT_PULLUP);
  pinMode(TMP102_IC8_ALERT, INPUT_PULLUP);

  // Motor Driver sensor: Set T_HIGH (70°C) and T_LOW (0°C)
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

  // Power Unit sensor: Set T_HIGH (60°C) and T_LOW (5°C)
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

void initPCA9555()
{
  // Configure PCA9555 ports as outputs
  Wire.beginTransmission(PCA9555_ADDR);
  Wire.write(0b00000110);
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(PCA9555_ADDR);
  Wire.write(0b00000111);
  Wire.write(0b00000000);
  Wire.endTransmission();

  setLeds(0, 0);
  Serial.println("PCA9555 initialized");
}

void initMotors()
{
  const uint8_t controlPins[] = {
      PWMA_1, PWMB_1, PWMA_2, PWMB_2,
      AIN1_1, AIN2_1, BIN1_1, BIN2_1,
      AIN1_2, AIN2_2, BIN1_2, BIN2_2,
      STBY};
  for (uint8_t pin : controlPins)
  {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  motor1.begin();
  motor2.begin();
  motor3.begin();
  motor4.begin();

  digitalWrite(STBY, HIGH);
  Serial.println("Motors initialized");
}

void readINA219()
{
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();
  float loadvoltage = busvoltage + (shuntvoltage / 1000);

  Serial.println("===== Power Readings =====");
  Serial.print("Bus Voltage:   ");
  Serial.print(busvoltage);
  Serial.println(" V");
  Serial.print("Shunt Voltage: ");
  Serial.print(shuntvoltage);
  Serial.println(" mV");
  Serial.print("Load Voltage:  ");
  Serial.print(loadvoltage);
  Serial.println(" V");
  Serial.print("Current:       ");
  Serial.print(current_mA);
  Serial.println(" mA");
  Serial.print("Power:         ");
  Serial.print(power_mW);
  Serial.println(" mW");
}

void readTMP102()
{
  // Motor Driver sensor
  Wire.beginTransmission(TMP102_IC20_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(static_cast<uint8_t>(TMP102_IC20_ADDR), static_cast<uint8_t>(2));

  Serial.println("===== Temperature Readings =====");
  if (Wire.available() == 2)
  {
    byte msb = Wire.read();
    byte lsb = Wire.read();
    int temperature = ((msb << 8) | lsb) >> 4;
    float tempC = temperature * 0.0625;
    Serial.print("Motor Driver Temp: ");
    Serial.print(tempC);
    Serial.print("°C (Alert: ");
    Serial.print(digitalRead(TMP102_IC20_ALERT) ? "No" : "YES");
    Serial.println(")");
  }

  // Power Unit sensor
  Wire.beginTransmission(TMP102_IC8_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(static_cast<uint8_t>(TMP102_IC8_ADDR), static_cast<uint8_t>(2));
  if (Wire.available() == 2)
  {
    byte msb = Wire.read();
    byte lsb = Wire.read();
    int temperature = ((msb << 8) | lsb) >> 4;
    float tempC = temperature * 0.0625;
    Serial.print("Power Unit Temp:   ");
    Serial.print(tempC);
    Serial.print("°C (Alert: ");
    Serial.print(digitalRead(TMP102_IC8_ALERT) ? "No" : "YES");
    Serial.println(")");
  }
}

void setLeds(byte newPort0, byte newPort1)
{
  port0State = newPort0;
  port1State = newPort1;

  Wire.beginTransmission(PCA9555_ADDR);
  Wire.write(0b00000010);
  Wire.write(~port0State);
  Wire.endTransmission();

  Wire.beginTransmission(PCA9555_ADDR);
  Wire.write(0b00000011);
  Wire.write(~port1State);
  Wire.endTransmission();
}

void runLedPattern(int patternNum)
{
  switch (patternNum)
  {
  case 1: // Basic pattern
    setLeds(led1 | led4, led10 | led7);
    vTaskDelay(pdMS_TO_TICKS(1000));
    setLeds(led2 | led5, led11 | led8);
    vTaskDelay(pdMS_TO_TICKS(1000));
    setLeds(led3 | led6, led9 | led12);
    vTaskDelay(pdMS_TO_TICKS(1000));
    setLeds(0, 0);
    break;

  case 2: // Sequential pattern
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
    setLeds(0, 0);
    break;

  case 3: // All on then off
    setLeds(0xFF, 0xFF);
    vTaskDelay(pdMS_TO_TICKS(1000));
    setLeds(0, 0);
    break;

  default:
    break;
  }
}

void processCommand(String command)
{
  command.toUpperCase();
  String parts[4];
  int count = 0;
  int lastIndex = 0;
  for (int i = 0; i < command.length() && count < 4; i++)
  {
    if (command[i] == ':')
    {
      parts[count++] = command.substring(lastIndex, i);
      lastIndex = i + 1;
    }
  }
  if (lastIndex < command.length())
  {
    parts[count++] = command.substring(lastIndex);
  }

#ifdef ESP_WIFI_MQTT
  // Handle WiFi commands
  if (parts[0] == "WIFI")
  {
    processWiFiCommand(command);
    return;
  }

  // Handle MQTT commands
  if (parts[0] == "MQTT")
  {
    processMQTTCommand(command);
    return;
  }
#endif
#ifdef ESP_STOP_SWITCH
  // Handle stop switch commands
  if (parts[0] == "STOP") {
    if (parts[1] == "PIN1") {
      stopPin1 = parts[2].toInt();
      saveSettings();
      Serial.println("Stop switch pin 1 set to " + String(stopPin1));
    } else if (parts[1] == "PIN2") {
      stopPin2 = parts[2].toInt();
      saveSettings();
      Serial.println("Stop switch pin 2 set to " + String(stopPin2));
    } else if (parts[1] == "STATUS") {
      Serial.println("Stop switch pin 1: " + String(stopPin1) );
      Serial.println("Stop switch pin 2: " + String(stopPin2) );
    }
  }
#endif

  // Motor commands
  if (parts[0] == "M1" || parts[0] == "M2" || parts[0] == "M3" || parts[0] == "M4")
  {
    Motor *targetMotor = nullptr;
    if (parts[0] == "M1")
      targetMotor = &motor1;
    else if (parts[0] == "M2")
      targetMotor = &motor2;
    else if (parts[0] == "M3")
      targetMotor = &motor3;
    else if (parts[0] == "M4")
      targetMotor = &motor4;

    if (targetMotor != nullptr)
    {
      if (parts[1] == "RL" || parts[1] == "FWD")
      {
        targetMotor->setDirection(FORWARD);
        if (count > 2 && parts[2].length() > 0)
          targetMotor->setSpeed(parts[2].toInt());
        Serial.println(parts[0] + " set to FORWARD at speed " + String(targetMotor->getSpeed()) + "%");
      }
      else if (parts[1] == "LL" || parts[1] == "REV" || parts[1] == "BWD")
      {
        targetMotor->setDirection(BACKWARD);
        if (count > 2 && parts[2].length() > 0)
          targetMotor->setSpeed(parts[2].toInt());
        Serial.println(parts[0] + " set to BACKWARD at speed " + String(targetMotor->getSpeed()) + "%");
      }
      else if (parts[1] == "STOP")
      {
        targetMotor->stop();
        Serial.println(parts[0] + " stopped");
      }
      else if (parts[1] == "SPD")
      {
        if (count > 2)
        {
          targetMotor->setSpeed(parts[2].toInt());
          Serial.println(parts[0] + " speed set to " + parts[2] + "%");
        }
      }
      else if (parts[1] == "STATUS")
      {
        String dirStr = "STOPPED";
        if (targetMotor->getDirection() == FORWARD)
          dirStr = "FORWARD";
        else if (targetMotor->getDirection() == BACKWARD)
          dirStr = "BACKWARD";
        Serial.println(parts[0] + " status: Direction=" + dirStr + ", Speed=" + String(targetMotor->getSpeed()) + "%");
      }
    }
    return;
  }

  // LED commands
  if (parts[0] == "LED")
  {
    if (parts[1] == "ALL")
    {
      if (parts[2] == "ON")
      {
        setLeds(0xFF, 0xFF);
        digitalWrite(LED_17, LOW);
        digitalWrite(LED_18, LOW);
        Serial.println("All LEDs turned ON");
      }
      else if (parts[2] == "OFF")
      {
        setLeds(0, 0);
        digitalWrite(LED_17, HIGH);
        digitalWrite(LED_18, HIGH);
        Serial.println("All LEDs turned OFF");
      }
    }
    else if (parts[1] == "PATTERN")
    {
      int patternNum = parts[2].toInt();
      Serial.println("Running LED pattern " + String(patternNum));
      runLedPattern(patternNum);
    }
    else
    {
      int ledNum = parts[1].toInt();
      if (ledNum >= 1 && ledNum <= 12)
      {
        byte ledMask;
        byte *portState;
        if (ledNum <= 6)
        {
          ledMask = 1 << (ledNum - 1);
          portState = &port0State;
        }
        else
        {
          ledMask = 1 << (12 - ledNum);
          portState = &port1State;
        }
        if (parts[2] == "ON")
        {
          *portState |= ledMask;
          Serial.println("LED " + String(ledNum) + " turned ON");
        }
        else if (parts[2] == "OFF")
        {
          *portState &= ~ledMask;
          Serial.println("LED " + String(ledNum) + " turned OFF");
        }
        else if (parts[2] == "TOGGLE")
        {
          *portState ^= ledMask;
          Serial.println("LED " + String(ledNum) + " toggled");
        }
        setLeds(port0State, port1State);
      }
      else if (ledNum >= 17 && ledNum <= 18)
      {
        if (parts[2] == "ON")
        {
          if (ledNum == 17)
          {
            digitalWrite(LED_17, LOW); // Assuming active-low configuration
          }
          else if (ledNum == 18)
          {
            digitalWrite(LED_18, LOW);
          }
        }
        else if (parts[2] == "OFF")
        {
          if (ledNum == 17)
          {
            digitalWrite(LED_17, HIGH);
          }
          else if (ledNum == 18)
          {
            digitalWrite(LED_18, HIGH);
          }
        }
      }
    }
    return;
  }

  // Sensor commands
  if (parts[0] == "SENSOR")
  {
    if (parts[1] == "READ")
    {
      if (parts[2] == "ALL" || parts[2] == "")
      {
        readINA219();
        readTMP102();
      }
      else if (parts[2] == "POWER")
      {
        readINA219();
      }
      else if (parts[2] == "TEMP")
      {
        readTMP102();
      }
    }
    else if (parts[1] == "AUTO")
    {
      if (parts[2] == "ON")
      {
        sensorReadingEnabled = true;
        if (count > 3)
        {
          sensorInterval = parts[3].toInt() * 1000;
        }
        Serial.println("Auto sensor readings enabled every " + String(sensorInterval / 1000) + " seconds");
      }
      else if (parts[2] == "OFF")
      {
        sensorReadingEnabled = false;
        Serial.println("Auto sensor readings disabled");
      }
    }
    return;
  }

  // System commands
  if (command == "STOP" || command == "ALERT")
  {
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
    setLeds(0, 0);
    Serial.println("ALERT STOP: All motors stopped, all LEDs off");
    return;
  }

  if (command == "STATUS")
  {
    printStatus();
    return;
  }

  if (command == "HELP")
  {
    printHelp();
    return;
  }

  Serial.println("Unknown command: " + command);
  Serial.println("Type HELP for available commands");
}

void printHelp()
{
  Serial.println("\n===== Available Commands =====");
  Serial.println("MOTOR CONTROL:");
  Serial.println("  Mx:RL:y    - Set motor x forward at y% speed (x=1-4, y=0-100)");
  Serial.println("  Mx:LL:y    - Set motor x backward at y% speed");
  Serial.println("  Mx:STOP     - Stop motor x");
  Serial.println("  Mx:SPD:y    - Set motor x speed to y%");
  Serial.println("  Mx:STATUS   - Get motor x status");

  Serial.println("\nLED CONTROL:");
  Serial.println("  LED:ALL:ON   - Turn on all LEDs");
  Serial.println("  LED:ALL:OFF  - Turn off all LEDs");
  Serial.println("  LED:n:ON     - Turn on LED n (n=1-12)");
  Serial.println("  LED:n:OFF    - Turn off LED n");
  Serial.println("  LED:n:TOGGLE - Toggle LED n");
  Serial.println("  LED:PATTERN:n - Run LED pattern n (n=1-3)");

  Serial.println("\nSENSOR COMMANDS:");
  Serial.println("  SENSOR:READ:ALL  - Read all sensors");
  Serial.println("  SENSOR:READ:POWER - Read power sensor (INA219)");
  Serial.println("  SENSOR:READ:TEMP - Read temperature sensors (TMP102)");
  Serial.println("  SENSOR:AUTO:ON:n - Enable auto sensor readings every n seconds");
  Serial.println("  SENSOR:AUTO:OFF  - Disable auto sensor readings");

#ifdef ESP_WIFI_MQTT
  Serial.println("\nWIFI COMMANDS:");
  Serial.println("  WIFI:SSID:name    - Set WiFi SSID");
  Serial.println("  WIFI:PASSWORD:pwd - Set WiFi password");
  Serial.println("  WIFI:STATIC:ON    - Enable static IP");
  Serial.println("  WIFI:STATIC:OFF   - Disable static IP (use DHCP)");
  Serial.println("  WIFI:IP:x.x.x.x   - Set static IP address");
  Serial.println("  WIFI:GATEWAY:x.x.x.x - Set gateway address");
  Serial.println("  WIFI:SUBNET:x.x.x.x  - Set subnet mask");
  Serial.println("  WIFI:CONNECT      - Connect to WiFi using current settings");
  Serial.println("  WIFI:STATUS       - Show WiFi settings and status");

  Serial.println("\nMQTT COMMANDS:");
  Serial.println("  MQTT:SERVER:host  - Set MQTT server hostname/IP");
  Serial.println("  MQTT:PORT:n       - Set MQTT server port");
  Serial.println("  MQTT:USER:name    - Set MQTT username");
  Serial.println("  MQTT:PASSWORD:pwd - Set MQTT password");
  Serial.println("  MQTT:ID:id        - Set MQTT client ID");
  Serial.println("  MQTT:TOPIC:topic  - Set MQTT base topic");
  Serial.println("  MQTT:CONNECT      - Connect to MQTT server");
  Serial.println("  MQTT:PUBLISH      - Publish current status to MQTT");
  Serial.println("  MQTT:STATUS       - Show MQTT settings and status");
#endif
#ifdef ESP_STOP_SWITCH
  Serial.println("\nSTOP SWITCH COMMANDS:");
  Serial.println("  STOP:PIN1:n      - Set stop switch pin 1 (default: 15)");
  Serial.println("  STOP:PIN2:n      - Set stop switch pin 2 (default: 16)");
  Serial.println("  STOP:STATUS       - Show stop switch settings");
#endif

  Serial.println("\nSYSTEM COMMANDS:");
  Serial.println("  STATUS      - Print system status");
  Serial.println("  STOP        - Emergency stop (all motors and LEDs off)");
  Serial.println("  HELP        - Show this help message");
}

void printStatus()
{
  Serial.println("\n===== System Status =====");
  Serial.println("MOTORS:");
  for (int i = 1; i <= 4; i++)
  {
    Motor *m;
    switch (i)
    {
    case 1:
      m = &motor1;
      break;
    case 2:
      m = &motor2;
      break;
    case 3:
      m = &motor3;
      break;
    case 4:
      m = &motor4;
      break;
    }
    String dirStr = "STOPPED";
    if (m->getDirection() == FORWARD)
      dirStr = "FORWARD";
    else if (m->getDirection() == BACKWARD)
      dirStr = "BACKWARD";
    Serial.println("  M" + String(i) + ": " + dirStr + " at " + String(m->getSpeed()) + "%");
  }
  // Serial.println("LEDs:");
  // Serial.println("  Port 0: 0b" + String(port0State, BIN));
  // Serial.println("  Port 1: 0b" + String(port1State, BIN));
  Serial.println("TEMPERATURE ALERTS:");
  Serial.println("  Motor Driver: " + String(digitalRead(TMP102_IC20_ALERT) ? "Normal" : "ALERT!"));
  Serial.println("  Power Unit: " + String(digitalRead(TMP102_IC8_ALERT) ? "Normal" : "ALERT!"));
  Serial.println("AUTO SENSOR READINGS: " + String(sensorReadingEnabled ? "Enabled" : "Disabled"));
  if (sensorReadingEnabled)
  {
    Serial.println("  Interval: " + String(sensorInterval / 1000) + " seconds");
  }
#ifdef ESP_STOP_SWITCH
  Serial.println("STOP SWITCH PINS:");
  Serial.println("  Pin 1 Triggert: "+ String(digitalRead(stopPin1) ? "YES" : "NO"));
  Serial.println("  Pin 2 Triggert: "+ String(digitalRead(stopPin2) ? "YES" : "NO"));
#endif
  readINA219();
  readTMP102();
#ifdef ESP_WIFI_MQTT
    printNetworkSettings();
#endif
}

#ifdef ESP_WIFI_MQTT


void setupWiFi()
{
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);

  if (useStaticIP && staticIP != IPAddress(0, 0, 0, 0))
  {
    if (!WiFi.config(staticIP, gateway, subnet, dns1, dns2))
    {
      Serial.println("Static IP configuration failed");
    }
  }

  WiFi.begin(ssid.c_str(), password.c_str());

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20)
  {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("\nFailed to connect to WiFi");
  }
}

void setupMQTT()
{
  mqttClient.setServer(mqttServer.c_str(), mqttPort);
  mqttClient.setCallback(mqttCallback);
  reconnectMQTT();
}

void reconnectMQTT()
{
  // Attempt to connect only if WiFi is connected
  if (WiFi.status() != WL_CONNECTED)
    return;

  if (!mqttClient.connected())
  {
    Serial.print("Connecting to MQTT broker...");

    bool connected = false;
    if (mqttUser.length() > 0)
    {
      connected = mqttClient.connect(mqttClientId.c_str(), mqttUser.c_str(), mqttPassword.c_str());
    }
    else
    {
      connected = mqttClient.connect(mqttClientId.c_str());
    }

    if (connected)
    {
      Serial.println("connected");

      // Subscribe to command topic
      String commandTopic = mqttBaseTopic + "command";
      mqttClient.subscribe(commandTopic.c_str());

      // Publish online status
      String statusTopic = mqttBaseTopic + "status";
      mqttClient.publish(statusTopic.c_str(), "online", true);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.println(mqttClient.state());
    }
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  String topicStr = String(topic);
  String message = "";

  for (unsigned int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }

  Serial.println("MQTT message received [" + topicStr + "]: " + message);

  // Process the received command
  if (topicStr == mqttBaseTopic + "command")
  {
    processCommand(message);
  }
}

float readTMP102Temperature(uint8_t address)
{
  Wire.beginTransmission(address);
  Wire.write(0x00);
  Wire.endTransmission();
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

String getMotorDirectionString(Motor &motor)
{
  if (motor.getDirection() == FORWARD)
    return "FORWARD";
  else if (motor.getDirection() == BACKWARD)
    return "BACKWARD";
  return "STOPPED";
}


void publishStatus()
{
  if (!mqttClient.connected() || WiFi.status() != WL_CONNECTED)
    return;

  // Publish all readings as JSON
  String powerTopic = mqttBaseTopic + "power";
  String tempTopic = mqttBaseTopic + "temperature";
  String motorsTopic = mqttBaseTopic + "motors";

  // Power readings
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();
  float loadvoltage = busvoltage + (shuntvoltage / 1000);

  String powerJson = "{\"busvoltage\":" + String(busvoltage) +
                     ",\"shuntvoltage\":" + String(shuntvoltage) +
                     ",\"loadvoltage\":" + String(loadvoltage) +
                     ",\"current_mA\":" + String(current_mA) +
                     ",\"power_mW\":" + String(power_mW) + "}";

  mqttClient.publish(powerTopic.c_str(), powerJson.c_str());

  // Temperature readings
  float motorTemp = readTMP102Temperature(TMP102_IC20_ADDR);
  float powerTemp = readTMP102Temperature(TMP102_IC8_ADDR);
  bool motorAlert = !digitalRead(TMP102_IC20_ALERT);
  bool powerAlert = !digitalRead(TMP102_IC8_ALERT);

  String tempJson = "{\"motorDriver\":{\"temp\":" + String(motorTemp) +
                    ",\"alert\":" + String(motorAlert ? "true" : "false") +
                    "},\"powerUnit\":{\"temp\":" + String(powerTemp) +
                    ",\"alert\":" + String(powerAlert ? "true" : "false") + "}}";

  mqttClient.publish(tempTopic.c_str(), tempJson.c_str());

  // Motor status
  String motorsJson = "{\"m1\":{\"direction\":\"" + getMotorDirectionString(motor1) +
                      "\",\"speed\":" + String(motor1.getSpeed()) +
                      "},\"m2\":{\"direction\":\"" + getMotorDirectionString(motor2) +
                      "\",\"speed\":" + String(motor2.getSpeed()) +
                      "},\"m3\":{\"direction\":\"" + getMotorDirectionString(motor3) +
                      "\",\"speed\":" + String(motor3.getSpeed()) +
                      "},\"m4\":{\"direction\":\"" + getMotorDirectionString(motor4) +
                      "\",\"speed\":" + String(motor4.getSpeed()) + "}}";

  mqttClient.publish(motorsTopic.c_str(), motorsJson.c_str());
}

void processWiFiCommand(String command)
{
  String parts[4];
  int count = 0;
  int lastIndex = 0;

  for (int i = 0; i < command.length() && count < 4; i++)
  {
    if (command[i] == ':')
    {
      parts[count++] = command.substring(lastIndex, i);
      lastIndex = i + 1;
    }
  }

  if (lastIndex < command.length())
  {
    parts[count++] = command.substring(lastIndex);
  }

  if (parts[1] == "SSID" && count > 2)
  {
    ssid = parts[2];
    Serial.println("WiFi SSID set to: " + ssid);
    saveSettings();
  }
  else if (parts[1] == "PASSWORD" && count > 2)
  {
    password = parts[2];
    Serial.println("WiFi password updated");
    saveSettings();
  }
  else if (parts[1] == "STATIC" && count > 2)
  {
    if (parts[2] == "ON")
    {
      useStaticIP = true;
      Serial.println("Static IP enabled");
      saveSettings();
    }
    else if (parts[2] == "OFF")
    {
      useStaticIP = false;
      Serial.println("Static IP disabled (using DHCP)");
      saveSettings();
    }
  }
  else if (parts[1] == "IP" && count > 2)
  {
    if (staticIP.fromString(parts[2]))
    {
      Serial.println("Static IP set to: " + staticIP.toString());
      saveSettings();
    }
    else
    {
      Serial.println("Invalid IP format");
    }
  }
  else if (parts[1] == "GATEWAY" && count > 2)
  {
    if (gateway.fromString(parts[2]))
    {
      Serial.println("Gateway set to: " + gateway.toString());
      saveSettings();
    }
    else
    {
      Serial.println("Invalid gateway format");
    }
  }
  else if (parts[1] == "SUBNET" && count > 2)
  {
    if (subnet.fromString(parts[2]))
    {
      Serial.println("Subnet mask set to: " + subnet.toString());
      saveSettings();
    }
    else
    {
      Serial.println("Invalid subnet mask format");
    }
  }
  else if (parts[1] == "CONNECT")
  {
    setupWiFi();
  }
  else if (parts[1] == "STATUS")
  {
    printNetworkSettings();
  }
  else
  {
    Serial.println("Unknown WiFi command");
  }
}

void processMQTTCommand(String command)
{
  String parts[4];
  int count = 0;
  int lastIndex = 0;

  for (int i = 0; i < command.length() && count < 4; i++)
  {
    if (command[i] == ':')
    {
      parts[count++] = command.substring(lastIndex, i);
      lastIndex = i + 1;
    }
  }

  if (lastIndex < command.length())
  {
    parts[count++] = command.substring(lastIndex);
  }

  if (parts[1] == "SERVER" && count > 2)
  {
    mqttServer = parts[2];
    Serial.println("MQTT server set to: " + mqttServer);
    saveSettings();
  }
  else if (parts[1] == "PORT" && count > 2)
  {
    mqttPort = parts[2].toInt();
    Serial.println("MQTT port set to: " + String(mqttPort));
    saveSettings();
  }
  else if (parts[1] == "USER" && count > 2)
  {
    mqttUser = parts[2];
    Serial.println("MQTT username set to: " + mqttUser);
    saveSettings();
  }
  else if (parts[1] == "PASSWORD" && count > 2)
  {
    mqttPassword = parts[2];
    Serial.println("MQTT password updated");
    saveSettings();
  }
  else if (parts[1] == "ID" && count > 2)
  {
    mqttClientId = parts[2];
    Serial.println("MQTT client ID set to: " + mqttClientId);
    saveSettings();
  }
  else if (parts[1] == "TOPIC" && count > 2)
  {
    mqttBaseTopic = parts[2];
    if (!mqttBaseTopic.endsWith("/"))
    {
      mqttBaseTopic += "/";
    }
    Serial.println("MQTT base topic set to: " + mqttBaseTopic);
    saveSettings();
  }
  else if (parts[1] == "CONNECT")
  {
    setupMQTT();
  }
  else if (parts[1] == "STATUS")
  {
    printNetworkSettings();
  }
  else if (parts[1] == "PUBLISH")
  {
    publishStatus();
    Serial.println("Published current status to MQTT");
  }
  else
  {
    Serial.println("Unknown MQTT command");
  }
}

void printNetworkSettings()
{
  Serial.println("\n===== Network Settings =====");
  Serial.println("WiFi:");
  Serial.println("  SSID: " + ssid);
  Serial.println("  Connected: " + String(WiFi.status() == WL_CONNECTED ? "Yes" : "No"));
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("  IP: " + WiFi.localIP().toString());
    Serial.println("  Gateway: " + WiFi.gatewayIP().toString());
    Serial.println("  Subnet: " + WiFi.subnetMask().toString());
  }
  Serial.println("  Static IP: " + String(useStaticIP ? "Enabled" : "Disabled"));
  if (useStaticIP)
  {
    Serial.println("    IP: " + staticIP.toString());
    Serial.println("    Gateway: " + gateway.toString());
    Serial.println("    Subnet: " + subnet.toString());
  }

  Serial.println("\nMQTT:");
  Serial.println("  Server: " + mqttServer);
  Serial.println("  Port: " + String(mqttPort));
  Serial.println("  Client ID: " + mqttClientId);
  Serial.println("  User: " + mqttUser);
  Serial.println("  Base Topic: " + mqttBaseTopic);
  Serial.println("  Connected: " + String(mqttClient.connected() ? "Yes" : "No"));
}
#endif
