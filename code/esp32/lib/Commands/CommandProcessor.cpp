
#include "CommandProcessor.h"

CommandProcessor::CommandProcessor()
{
  _motor1 = nullptr;
  _motor2 = nullptr;
  _motor3 = nullptr;
  _motor4 = nullptr;
  _sensorCtrl = nullptr;
  _ledCtrl = nullptr;
  _cfg = nullptr;
}

CommandProcessor::~CommandProcessor()
{
  // Destructor implementation (if needed)
}

void CommandProcessor::begin(SensorController *sensorCtr, LedController *ledCtr, Config *cfg)
{
  _sensorCtrl = sensorCtr;
  _ledCtrl = ledCtr;
  _cfg = cfg;

  extern Motor motor1, motor2, motor3, motor4;

  _motor1 = &motor1;
  _motor2 = &motor2;
  _motor3 = &motor3;
  _motor4 = &motor4;

  Serial.println("CommandProcessor initialized.");
}

void CommandProcessor::processCommand(const String &command)
{
  String cmd = command;
  cmd.toUpperCase();

  String parts[4];

  int count = 0;
  int lastIndex = 0;

  for (int i = 0; i < cmd.length() && count < 4; i++)
  {
    if (cmd[i] == ':')
    {
      parts[count++] = cmd.substring(lastIndex, i);
      lastIndex = i + 1;
    }
  }
  if (lastIndex < cmd.length())
  {
    parts[count++] = cmd.substring(lastIndex);
  }

#ifdef ESP_WIFI_MQTT

  if (parts[0] == "WIFI")
  {
    processWiFiCommand(cmd);
    return;
  }
  if (parts[0] == "MQTT")
  {
    processMQTTCommand(cmd);
    return;
  }
#endif
#ifdef ESP_STOP_SWITCH
  if (parts[0] == "STOP")
  {
    processStopSwitchCommand(parts[1], parts[2]);
  }
#endif

  if (parts[0] == "M1" || parts[0] == "M2" || parts[0] == "M3" || parts[0] == "M4")
  {
    parts[2] = count < 2 ? parts[2] : "";
    processMotorCommand(parts[0], parts[1], parts[2]);
  }

  if (parts[0] == "LED")
  {
    processLedCommand(parts[1], parts[2]);
  }

  if (parts[0] == "SENSOR")
  {
    parts[3] = count < 3 ? parts[3] : ""; // Wenn keine value vorhanden sende lehrer string und nicht nichts
    processSensorCommand(parts[1], parts[2], parts[3]);
  }

  if (cmd == "STOP" || cmd == "ALERT")
  {
    _motor1->stop();
    _motor2->stop();
    _motor3->stop();
    _motor4->stop();

    _ledCtrl->setLeds(0b00000000, 0b00000000);
    Serial.println("ALERT STOP: All motors stopped, all LEDs off");
  }

  if (cmd == "STATUS")
  {
    printStatus();
  }

  if (cmd == "HELP")
  {
    printHelp();
  }

  Serial.println("Unknown command: " + cmd);
  Serial.println("Type HELP for available commands");
}

#ifdef ESP_STOP_SWITCH
void CommandProcessor::processStopSwitchCommand(const String &action, const String &value)
{
  if (action == "PIN1")
  {
    _cfg->stopPin1 = value.toInt();
    _cfg->saveSettings();
    Serial.println("Stop switch pin 1 set to " + String(_cfg->stopPin1));
    return;
  }
  else if (action == "PIN2")
  {
    _cfg->stopPin2 = value.toInt();
    _cfg->saveSettings();
    Serial.println("Stop switch pin 2 set to " + String(_cfg->stopPin2));
    return;
  }
  else if (action == "STATUS")
  {
    Serial.println("\n===== Stop Switch Status =====");
    Serial.println("Stop switch pin 1: " + String(_cfg->stopPin1));
    Serial.println(" Pin 1 Triggert: " + String(digitalRead(_cfg->stopPin1) ? "YES" : "NO"));
    Serial.println("Stop switch pin 2: " + String(_cfg->stopPin2));
    Serial.println(" Pin 2 Triggert: " + String(digitalRead(_cfg->stopPin2) ? "YES" : "NO"));
    return;
  }
}
#endif

void CommandProcessor::processMotorCommand(const String &motor, const String &action, const String &value)
{
  Motor *targetMotor = nullptr;
  if (motor == "M1")
  {
    targetMotor = _motor1;
  }
  else if (motor == "M2")
  {
    targetMotor = _motor2;
  }
  else if (motor == "M3")
  {
    targetMotor = _motor3;
  }
  else if (motor == "M4")
  {
    targetMotor = _motor4;
  }

  if (targetMotor != nullptr)
  {
    if (action == "RL" || action == "FWD")
    {
      targetMotor->setDirection(FORWARD);
      if (value.length() > 0)
      {
        targetMotor->setSpeed(value.toInt());
      }
      Serial.println(String(motor) + " set FORWARD at speed " + String(value.length() > 0 ? value.toInt() : targetMotor->getSpeed()) + "%");
      return;
    }
    else if (action == "LL" || action == "REV" || action == "BWD")
    {
      targetMotor->setDirection(BACKWARD);
      if (value.length() > 0)
      {
        targetMotor->setSpeed(value.toInt());
        Serial.println(String(motor) + " set BACKWARD at speed " + String(value) + "%");
        return;
      }
    }
    else if (action == "STOP")
    {
      targetMotor->setDirection(STOP);
      Serial.println(String(motor) + " stopped");
      return;
    }
    else if (action == "SPD")
    {
      if (value.length() > 0)
      {
        targetMotor->setSpeed(value.toInt());
        Serial.println(String(motor) + " speed set to " + String(value) + "%");
        return;
      }
      else
      {
        Serial.println(String(motor) + " speed not set");
        return;
      }
    }
    else if (action == "STATUS")
    {
      String dirStr = "Unknown";
      switch (targetMotor->getDirection())
      {
      case FORWARD:
        dirStr = "FORWARD";
        break;
      case BACKWARD:
        dirStr = "BACKWARD";
        break;
      case STOP:
        dirStr = "STOPPED";
        break;
      default:
        dirStr = "Unknown";
        break;
      }
      Serial.println(String(motor) + " status: Direction=" + dirStr + ", Speed=" + String(targetMotor->getSpeed()) + "%");
      return;
    }
    else
    {
      Serial.println("Unknown motor command: " + action);
      return;
    }
  }
}

void CommandProcessor::processLedCommand(const String &ledIndex, const String &action)
{
  if (ledIndex == "ALL")
  {
    if (action == "ON")
    {
      _ledCtrl->setLeds(0xFF, 0xFF);
      Serial.println("All LEDs turned ON");
      return;
    }
    else if (action == "OFF")
    {
      _ledCtrl->setLeds(0b00000000, 0b00000000);
      Serial.println("All LEDs turned OFF");
      return;
    }
  }
  else if (ledIndex == "PATTERN")
  {
    _ledCtrl->runLedPattern(action.toInt());
    return;
  }
  else
  {
    int ledNum = ledIndex.toInt();
    if (action == "ON")
    {
      _ledCtrl->setLed(ledNum, true);
      return;
    }
    else if (action == "OFF")
    {
      _ledCtrl->setLed(ledNum, false);
      return;
    }
    else if (action == "TOGGLE")
    {
      _ledCtrl->toggleLed(ledNum);
      return;
    }
  }
}

void CommandProcessor::processSensorCommand(const String &action, const String &sensor, const String &value)
{
  if (action == "READ")
  {
    if (sensor == "ALL" || sensor == "")
    {
      _sensorCtrl->readINA219();
      _sensorCtrl->readTMP102();
      return;
    }
    else if (sensor == "POWER")
    {
      _sensorCtrl->readINA219();
      return;
    }
    else if (sensor == "TEMP")
    {
      _sensorCtrl->readTMP102();
      return;
    }
  }
  else if (action == "AUTO")
  {
    if (sensor == "ON")
    {
      _cfg->sensorConsolePrintingEnabled = true;
      if (value.length() > 0)
      {
        _cfg->sensorConsolePrintingInterval = value.toInt() * 1000;
      }
      _cfg->saveSettings();
      Serial.println("Auto sensor readings enabled every " + String(_cfg->sensorConsolePrintingInterval / 1000) + " seconds");
      return;
    }
    else if (sensor == "OFF")
    {
      _cfg->sensorConsolePrintingEnabled = false;
      _cfg->saveSettings();
      Serial.println("Auto sensor readings disabled");
      return;
    }
  }
  else
  {
    Serial.println("Unknown sensor command: " + action);
    return;
  }
}

// }

/*
                          __        _____ _____ ___      _____ _   _
                          \ \      / /_ _|  ___|_ _|    |  ___| \ | |
                           \ \ /\ / / | || |_   | |_____| |_  |  \| |
                            \ V  V /  | ||  _|  | |_____|  _| | |\  |
                             \_/\_/  |___|_|   |___|    |_|   |_| \_|
*/

// void processWiFiCommand(String command)
// {
//   String parts[4];
//   int count = 0;
//   int lastIndex = 0;

//   for (int i = 0; i < command.length() && count < 4; i++)
//   {
//     if (command[i] == ':')
//     {
//       parts[count++] = command.substring(lastIndex, i);
//       lastIndex = i + 1;
//     }
//   }

//   if (lastIndex < command.length())
//   {
//     parts[count++] = command.substring(lastIndex);
//   }

//   if (parts[1] == "SSID" && count > 2)
//   {
//     ssid = parts[2];
//     Serial.println("WiFi SSID set to: " + ssid);
//     saveSettings();
//   }
//   else if (parts[1] == "PASSWORD" && count > 2)
//   {
//     password = parts[2];
//     Serial.println("WiFi password updated");
//     saveSettings();
//   }
//   else if (parts[1] == "STATIC" && count > 2)
//   {
//     if (parts[2] == "ON")
//     {
//       useStaticIP = true;
//       Serial.println("Static IP enabled");
//       saveSettings();
//     }
//     else if (parts[2] == "OFF")
//     {
//       useStaticIP = false;
//       Serial.println("Static IP disabled (using DHCP)");
//       saveSettings();
//     }
//   }
//   else if (parts[1] == "IP" && count > 2)
//   {
//     if (staticIP.fromString(parts[2]))
//     {
//       Serial.println("Static IP set to: " + staticIP.toString());
//       saveSettings();
//     }
//     else
//     {
//       Serial.println("Invalid IP format");
//     }
//   }
//   else if (parts[1] == "GATEWAY" && count > 2)
//   {
//     if (gateway.fromString(parts[2]))
//     {
//       Serial.println("Gateway set to: " + gateway.toString());
//       saveSettings();
//     }
//     else
//     {
//       Serial.println("Invalid gateway format");
//     }
//   }
//   else if (parts[1] == "SUBNET" && count > 2)
//   {
//     if (subnet.fromString(parts[2]))
//     {
//       Serial.println("Subnet mask set to: " + subnet.toString());
//       saveSettings();
//     }
//     else
//     {
//       Serial.println("Invalid subnet mask format");
//     }
//   }
//   else if (parts[1] == "CONNECT")
//   {
//     setupWiFi();
//   }
//   else if (parts[1] == "STATUS")
//   {
//     printNetworkSettings();
//   }
//   else
//   {
//     Serial.println("Unknown WiFi command");
//   }
// }

/*
                               __        _____ _____ ___      _____ _   _
                               \ \      / /_ _|  ___|_ _|    |  ___| \ | |
                                \ \ /\ / / | || |_   | |_____| |_  |  \| |
                                 \ V  V /  | ||  _|  | |_____|  _| | |\  |
                                  \_/\_/  |___|_|   |___|    |_|   |_| \_|
*/

// void processMQTTCommand(String command)
// {
//   String parts[4];
//   int count = 0;
//   int lastIndex = 0;

//   for (int i = 0; i < command.length() && count < 4; i++)
//   {
//     if (command[i] == ':')
//     {
//       parts[count++] = command.substring(lastIndex, i);
//       lastIndex = i + 1;
//     }
//   }

//   if (lastIndex < command.length())
//   {
//     parts[count++] = command.substring(lastIndex);
//   }

//   if (parts[1] == "SERVER" && count > 2)
//   {
//     mqttServer = parts[2];
//     Serial.println("MQTT server set to: " + mqttServer);
//     saveSettings();
//   }
//   else if (parts[1] == "PORT" && count > 2)
//   {
//     mqttPort = parts[2].toInt();
//     Serial.println("MQTT port set to: " + String(mqttPort));
//     saveSettings();
//   }
//   else if (parts[1] == "USER" && count > 2)
//   {
//     mqttUser = parts[2];
//     Serial.println("MQTT username set to: " + mqttUser);
//     saveSettings();
//   }
//   else if (parts[1] == "PASSWORD" && count > 2)
//   {
//     mqttPassword = parts[2];
//     Serial.println("MQTT password updated");
//     saveSettings();
//   }
//   else if (parts[1] == "ID" && count > 2)
//   {
//     mqttClientId = parts[2];
//     Serial.println("MQTT client ID set to: " + mqttClientId);
//     saveSettings();
//   }
//   else if (parts[1] == "TOPIC" && count > 2)
//   {
//     mqttBaseTopic = parts[2];
//     if (!mqttBaseTopic.endsWith("/"))
//     {
//       mqttBaseTopic += "/";
//     }
//     Serial.println("MQTT base topic set to: " + mqttBaseTopic);
//     saveSettings();
//   }
//   else if (parts[1] == "CONNECT")
//   {
//     setupMQTT();
//   }
//   else if (parts[1] == "STATUS")
//   {
//     printNetworkSettings();
//   }
//   else if (parts[1] == "PUBLISH")
//   {
//     publishStatus();
//     Serial.println("Published current status to MQTT");
//   }
//   else
//   {
//     Serial.println("Unknown MQTT command");
//   }
// }

/*

                                   ____       _       _        _____
                                  |  _ \ _ __(_)_ __ | |_     |  ___| __
                                  | |_) | '__| | '_ \| __|____| |_ | '_ \
                                  |  __/| |  | | | | | ||_____|  _|| | | |
                                  |_|   |_|  |_|_| |_|\__|    |_|  |_| |_|


*/

void CommandProcessor::printStatus()
{
  Serial.println("\n===== System Status =====");

  Serial.println("MOTORS:");
  for (int i = 1; i <= 4; i++)
  {
    String dirStr = "STOPPED";
    Motor *targetMotor = nullptr;
    ;
    switch (i)
    {
    case 1:
      targetMotor = _motor1;
      break;
    case 2:
      targetMotor = _motor2;
      break;
    case 3:
      targetMotor = _motor3;
      break;
    case 4:
      targetMotor = _motor4;
      break;
    }
    if (targetMotor != nullptr)
    {
      dirStr = "STOPPED";
    }
    else if (targetMotor->getDirection() == FORWARD)
    {
      dirStr = "FORWARD";
    }
    else if (targetMotor->getDirection() == BACKWARD)
    {
      dirStr = "BACKWARD";
    }
    else
    {
      Serial.println("Null pointer error in Motor " + String(i));
      break;
    }
    Serial.println("  M" + String(i) + ": " + dirStr + " at " + String(targetMotor->getSpeed()) + "%");
  }

  Serial.println("LEDs:");
  Serial.println("  Port 0: 0b" + String(_ledCtrl->getPort0State(), BIN));
  Serial.println("  Port 1: 0b" + String(_ledCtrl->getPort1State(), BIN));

  Serial.println("\n===== Sensor Status =====");
  Serial.println(" Temperature Sensors");
  _sensorCtrl->readTMP102();
  Serial.println("  Motor Driver: " + String(_sensorCtrl->getMotorDriverAlert() ? "ALERT" : "Normal"));
  Serial.println("  Power Unit: " + String(_sensorCtrl->getPowerUnitAlert() ? "ALERT" : "Normal"));

  Serial.println(" Power Sensor");
  _sensorCtrl->readINA219();

  Serial.println("\n===== Sensor Configuration =====");
  Serial.println("AUTO SENSOR READINGS: " + String(_cfg->sensorConsolePrintingEnabled ? "Enabled" : "Disabled"));
  if (_cfg->sensorConsolePrintingEnabled)
    Serial.println("  Interval: " + String(_cfg->sensorConsolePrintingInterval / 1000) + " seconds");
  Serial.println("SENSOR UPDATE: " + String(_cfg->sensorUpdateEnabled ? "Enabled" : "Disabled"));
  Serial.println("  Interval: " + String(_cfg->sensorUpdateInterval / 1000) + " seconds");

#ifdef ESP_STOP_SWITCH
  Serial.println("\n===== Stop Switch Status =====");
  Serial.println("  Pin 1 Triggert: " + String(digitalRead(_cfg->stopPin1) ? "YES" : "NO"));
  Serial.println("  Pin 2 Triggert: " + String(digitalRead(_cfg->stopPin2) ? "YES" : "NO"));
#endif

#ifdef ESP_WIFI_MQTT
  Serial.println("\n===== Network Status =====");
  Serial.println("WiFi:");
  if (_wifiManager)
  {
    _wifiManager->printNetworkSettings();
  }
  //   Serial.println("WiFi:");
  //   Serial.println("  Connected: " + String(WiFi.status() == WL_CONNECTED ? "Yes" : "No"));
  //   if (WiFi.status() == WL_CONNECTED)
  //   {
  //     Serial.println("  IP: " + WiFi.localIP().toString());
  //     Serial.println("  Gateway: " + WiFi.gatewayIP().toString());
  //     Serial.println("  Subnet: " + WiFi.subnetMask().toString());
  //   }
  //   Serial.println("\nMQTT:");
  //   Serial.println("  Connected: " + String(mqttClient.connected() ? "Yes" : "No"));

#endif
}

void CommandProcessor::printHelp()
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
  Serial.println("  STOP:PIN1:n      - Set stop switch pin 1 default: 15");
  Serial.println("  STOP:PIN2:n      - Set stop switch pin 2 default: 16");
  Serial.println("  STOP:STATUS       - Show stop switch settings");
#endif

  Serial.println("\nSYSTEM COMMANDS:");
  Serial.println("  STATUS      - Print system status");
  Serial.println("  STOP        - Emergency stop all motors and LEDs off");
  Serial.println("  HELP        - Show this help message");
}
