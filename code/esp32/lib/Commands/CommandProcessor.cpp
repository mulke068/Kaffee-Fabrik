/**
 * @file CommandProcessor.cpp
 * @author Kevin Muller (@kevbchef.com)
 * @brief Command Processor for ESP32-based robot
 * @details This class processes commands received from the serial interface and executes the corresponding actions on motors, LEDs, and sensors.
 * @version 0.1
 * @date 2025-04-16
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "CommandProcessor.h"

/**
 * @brief Construct a new Command Processor:: Command Processor object
 * @details Initializes the motor and sensor pointers to nullptr.
 *
 */
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

/**
 * @brief Destroy the Command Processor:: Command Processor object
 *
 */
CommandProcessor::~CommandProcessor()
{
  // Destructor implementation (if needed)
}

/**
 * @brief Initializes the Command Processor with the provided sensor and LED controllers.
 * @param sensorCtr Pointer to the SensorController object.
 * @param ledCtr Pointer to the LedController object.
 * @param cfg Pointer to the Config object.
 */
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

  Serial.write("CommandProcessor initialized.");
}

/**
 * @brief Processes the received command string.
 * @param command The command string to process.
 * @details This function parses the command string and executes the corresponding action.
 *          It supports motor control, LED control, sensor reading, and system commands.
 */
void CommandProcessor::processCommand(const String &command)
{
  String cmd = command;
  cmd.toUpperCase();

  String parts[4];

  int count = 0;
  int lastIndex = 0;

  for (int i = 0; i < cmd.length() && count < 3; i++)
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

  // #ifdef ESP_WIFI_MQTT

  //   if (parts[0] == "WIFI")
  //   {
  //     processWiFiCommand(cmd);
  //     return;
  //   }
  //   if (parts[0] == "MQTT")
  //   {
  //     processMQTTCommand(cmd);
  //     return;
  //   }
  // #endif

  if (parts[0] == "M1" || parts[0] == "M2" || parts[0] == "M3" || parts[0] == "M4")
  {
    parts[2] = count < 3 ? "" : parts[2];
    processMotorCommand(parts[0], parts[1], parts[2]);
    return;
  }
  else if (parts[0] == "LED")
  {
    processLedCommand(parts[1], parts[2]);
    return;
  }
  else if (parts[0] == "SENSOR")
  {
    parts[3] = count < 4 ? "" : parts[3];
    // Serial.printf("SENSOR: %s:%s:%s\n", parts[1].c_str(), parts[2].c_str(), parts[3].c_str());
    processSensorCommand(parts[1], parts[2], parts[3]);
    return;
  }

#ifdef ESP_STOP_SWITCH
  else if (parts[0] == "SWITCH")
  {
    processStopSwitchCommand(parts[1], parts[2]);
  }
#endif // ESP_STOP_SWITCH

  else if (cmd == "STOP" || cmd == "ALERT")
  {
    _motor1->stop();
    _motor2->stop();
    _motor3->stop();
    _motor4->stop();

    _ledCtrl->setLeds(0b00000000, 0b00000000);
    Serial.write("ALERT STOP: All motors stopped, all LEDs off\n");
    return;
  }
  else if (cmd == "STATUS")
  {
    printStatus();
    return;
  }
#ifdef MONITORING_ENABLED
  else if (cmd == "MONITOR")
  {
    printSystemMonitor();
    return;
  }
#endif
  else if (cmd == "CLEAR")
  {
    Serial.write("\033[2J\033[H"); // Clear the console
    return;
  }
  else if (cmd == "HELP")
  {
    printHelp();
    return;
  }
  else if (cmd == "SAVE")
  {
    _cfg->saveSettings();
    Serial.write("Settings saved\n");
    return;
  }
  else if (cmd == "REBOOT")
  {
    ESP.restart();
    return;
  }
  else
  {
    Serial.print("Unknown command: ");
    Serial.println(cmd);
    Serial.write("Type HELP for available commands\n");
  }
}

/*
                ____  _             ____          _   _      _           _____ _   _
               / ___|| |_ ___  _ __/ ___|_      _| |_(_) ___| |__       |  ___| \ | |
               \___ \| __/ _ \| '_ \___ \ \ /\ / / __| |/ __| '_ \ _____| |_  |  \| |
                ___) | || (_) | |_) |__) \ V  V /| |_| | (__| | | |_____|  _| | |\  |
               |____/ \__\___/| .__/____/ \_/\_/  \__|_|\___|_| |_|     |_|   |_| \_|
                              |_|
*/

#ifdef ESP_STOP_SWITCH
/**
 * @brief Processes the stop switch command.
 * @param action The action to perform (PIN1, PIN2, STATUS).
 * @param value The value associated with the action.
 * @details This function sets the stop switch pins or prints their status.
 */
void CommandProcessor::processStopSwitchCommand(const String &action, const String &value)
{
  if (action == "PIN1")
  {
    _cfg->stopPin1 = value.toInt();
    Serial.print("Stop switch pin 1 set to ");
    Serial.println(_cfg->stopPin1);
    return;
  }
  else if (action == "PIN2")
  {
    _cfg->stopPin2 = value.toInt();
    Serial.print("Stop switch pin 2 set to ");
    Serial.println(_cfg->stopPin2);
    return;
  }
  else if (action == "STATUS")
  {
    printStopSwitch();
    return;
  }
  else
  {
    Serial.write("Unknown stop switch action\n");
    return;
  }
}
/**
 * @brief Prints the status of the stop switch pins.
 * @details This function prints the current status of the stop switch pins to the serial console.
 */
void CommandProcessor::printStopSwitch()
{
  Serial.write("\n===== Stop Switch Status =====\n");
  Serial.print("Stop switch pin 1: ");
  Serial.println(_cfg->stopPin1);
  Serial.print("  Pin 1 Triggered: ");
  Serial.println(digitalRead(_cfg->stopPin1) ? "YES" : "NO");
  Serial.print("Stop switch pin 2: ");
  Serial.println(_cfg->stopPin2);
  Serial.print("  Pin 2 Triggered: ");
  Serial.println(digitalRead(_cfg->stopPin2) ? "YES" : "NO");
  return;
}
#endif // ESP_STOP_SWITCH
/*
                      __  __       _                  _____ _   _
                     |  \/  | ___ | |_ ___  _ __     |  ___| \ | |
                     | |\/| |/ _ \| __/ _ \| '__|____| |_  |  \| |
                     | |  | | (_) | || (_) | | |_____|  _| | |\  |
                     |_|  |_|\___/ \__\___/|_|       |_|   |_| \_|
*/

/**
 * @brief Processes the motor command.
 * @param motor The motor to control (M1, M2, M3, M4).
 * @param action The action to perform (RL, LL, STOP, SPD, STATUS).
 * @param value The value associated with the action (speed).
 * @details This function updates the motor's direction and speed based on the received command.
 */
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
      targetMotor->updateDirection(FORWARD);
      if (value.length() > 0)
      {
        targetMotor->updateSpeed(value.toInt());
      }
      Serial.print(motor);
      Serial.print(" set FORWARD at speed ");
      Serial.print(value.length() > 0 ? value.toInt() : targetMotor->getSpeed());
      Serial.print("%\n");
      return;
    }
    else if (action == "LL" || action == "BWD")
    {
      targetMotor->updateDirection(BACKWARD);
      if (value.length() > 0)
      {
        targetMotor->updateSpeed(value.toInt());
      }
      Serial.print(motor);
      Serial.print(" set BACKWARD at speed ");
      Serial.print(value.length() > 0 ? value.toInt() : targetMotor->getSpeed());
      Serial.print("%\n");
      return;
    }
    else if (action == "STOP")
    {
      targetMotor->updateDirection(STOP);
      Serial.print(motor);
      Serial.print(" stopped\n");
      return;
    }
    else if (action == "SPD")
    {
      if (value.length() > 0)
      {
        targetMotor->updateSpeed(value.toInt());
        Serial.print(motor);
        Serial.print(" speed set to ");
        Serial.print(value);
        Serial.print("%\n");
        return;
      }
      else
      {
        Serial.print(motor);
        Serial.print(" speed not set\n");
        return;
      }
    }
    else if (action == "STATUS")
    {
      targetMotor->printStatus();
      return;
    }
    else
    {
      Serial.print("Unknown motor command: ");
      Serial.println(action);

      return;
    }
  }
}

/*
                            _             _       _____ _   _
                           | |    ___  __| |     |  ___| \ | |
                           | |   / _ \/ _` |_____| |_  |  \| |
                           | |__|  __/ (_| |_____|  _| | |\  |
                           |_____\___|\__,_|     |_|   |_| \_|
*/

/**
 * @brief Processes the LED command.
 * @param ledIndex The index of the LED (ALL, n, PATTERN).
 * @param action The action to perform (ON, OFF, TOGGLE).
 * @details This function updates the state of the specified LED or all LEDs based on the received command.
 */
void CommandProcessor::processLedCommand(const String &ledIndex, const String &action)
{
  if (ledIndex == "ALL")
  {
    if (action == "ON")
    {
      _ledCtrl->setAllOn();
      Serial.write("All LEDs turned ON\n");
      return;
    }
    else if (action == "OFF")
    {
      _ledCtrl->setAllOff();
      Serial.write("All LEDs turned OFF\n");
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

/*
                       ____                                 _____ _   _
                      / ___|  ___ _ __  ___  ___  _ __     |  ___| \ | |
                      \___ \ / _ \ '_ \/ __|/ _ \| '__|____| |_  |  \| |
                       ___) |  __/ | | \__ \ (_) | | |_____|  _| | |\  |
                      |____/ \___|_| |_|___/\___/|_|       |_|   |_| \_|
*/

/**
 * @brief Processes the sensor command.
 * @param action The action to perform (READ, AUTO).
 * @param sensor The sensor to read (ALL, POWER, TEMP).
 * @param value The value associated with the action (interval).
 * @details This function reads the specified sensor or enables/disables auto sensor readings based on the received command.
 */
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
      _cfg->sensorConsolePrintingInterval = value.toInt() * 1000;
      Serial.print("Auto sensor readings enabled every :");
      Serial.print(_cfg->sensorConsolePrintingInterval / 1000);
      Serial.print(" seconds\n");
      return;
    }
    else if (sensor == "OFF")
    {
      _cfg->sensorConsolePrintingEnabled = false;
      Serial.write("Auto sensor readings disabled\n");
      return;
    }
  }
  else
  {
    Serial.print("Unknown sensor command: ");
    Serial.println(action);
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

/**
 * @brief Prints the current status of the system.
 * @details This function prints the status of motors, LEDs, and sensors to the serial console.
 */
void CommandProcessor::printStatus()
{
  Serial.write("\n===== System Status =====\n");
  Serial.write("MOTORS:\n");
  for (int i = 1; i <= 4; i++)
  {
    Motor *targetMotor = nullptr;
    const char *dirStr = "UNKNOWN";
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

    if (!targetMotor)
    {
      Serial.print("Motor");
      Serial.print(i);
      Serial.print(" Null pointer Error\n");
      continue;
    }

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
      dirStr = "UNKNOWN";
      break;
    }

    Serial.print("  M");
    Serial.print(i);
    Serial.print(": ");
    Serial.print("Direction: ");
    Serial.print(dirStr);
    Serial.print("  Speed: ");
    Serial.print(targetMotor->getSpeed());
    Serial.print("%\n");
  }

  Serial.write("LEDs:\n");
  Serial.print("  Port 0: 0b");
  Serial.println(_ledCtrl->getPort0State(), BIN);
  Serial.print("  Port 1: 0b");
  Serial.println(_ledCtrl->getPort1State(), BIN);

  // Serial.println("\n===== Sensor Status =====");
  _sensorCtrl->readTMP102();
  _sensorCtrl->readINA219();

  Serial.write("\n===== Sensor Configuration =====\n");
  Serial.print("AUTO SENSOR READINGS: ");
  Serial.println(_cfg->sensorConsolePrintingEnabled ? "Enabled" : "Disabled");
  if (_cfg->sensorConsolePrintingEnabled)
  {
    Serial.print("  Interval: ");
    Serial.print(_cfg->sensorConsolePrintingInterval / 1000);
    Serial.print(" seconds\n");
  }

  Serial.print("SENSOR UPDATE: ");
  Serial.println(_cfg->sensorUpdateEnabled ? "Enabled" : "Disabled");
  if (_cfg->sensorUpdateEnabled)
  {
    Serial.print("  Interval: ");
    Serial.print(_cfg->sensorUpdateInterval / 1000);
    Serial.print(" seconds\n");
  }

#ifdef ESP_STOP_SWITCH
  Serial.write("\n===== Stop Switch Status =====\n");
  Serial.print("  Pin 1 Triggered: ");
  Serial.println(digitalRead(_cfg->stopPin1) ? "Yes" : "No");
  Serial.print("  Pin 2 Triggered: ");
  Serial.println(digitalRead(_cfg->stopPin2) ? "Yes" : "No");
#endif // ESP_STOP_SWITCH

  // #ifdef ESP_WIFI_MQTT
  // Serial.write("\n===== Network Status =====\n");
  // Serial.write("WiFi:\n");
  // if (_wifiManager)
  // {
  //   _wifiManager->printNetworkSettings();
  // }
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

  // #endif
}
/*
                                   ____       _       _        _____
                                  |  _ \ _ __(_)_ __ | |_     |  ___| __
                                  | |_) | '__| | '_ \| __|____| |_ | '_ \
                                  |  __/| |  | | | | | ||_____|  _|| | | |
                                  |_|   |_|  |_|_| |_|\__|    |_|  |_| |_|


*/

// NOT MY CODE!
// at that point i cant help you anymore
// hope if it works if not in GlobalConfig disable the monitoring
#ifdef MONITORING_ENABLED
#include "TaskManager.h"
#include <map>
#include <esp_timer.h>
#include <esp_clk.h> // for esp_clk_cpu_freq()

extern TaskManager taskManager;

/**
 * @brief Prints the system monitor information.
 * @details This function prints the system monitor information, including CPU usage, RAM usage, and task status.
 */
void CommandProcessor::printSystemMonitor()
{
  static uint64_t lastPrintTime = esp_timer_get_time(); // µs
  static std::map<std::string, uint32_t> prevTaskRuntime;
  static uint32_t prevTotalRuntime = 0;

  size_t freeHeap = xPortGetFreeHeapSize();
  size_t minFreeHeap = xPortGetMinimumEverFreeHeapSize();
  size_t totalHeap = ESP.getHeapSize();

#ifdef BOARD_HAS_PSRAM
  size_t freePsram = ESP.getFreePsram();
  size_t totalPsram = ESP.getPsramSize();
#endif // BOARD_HAS_PSRAM

  float temp = temperatureRead();

  UBaseType_t taskCount = uxTaskGetNumberOfTasks();
  TaskStatus_t *taskStatus = (TaskStatus_t *)pvPortMalloc(taskCount * sizeof(TaskStatus_t));
  if (!taskStatus)
  {
    Serial.println("Memory allocation failed!");
    return;
  }

  uint32_t totalRuntime = 0;
  UBaseType_t capturedTasks = uxTaskGetSystemState(taskStatus, taskCount, nullptr);
  for (UBaseType_t i = 0; i < capturedTasks; i++)
  {
    totalRuntime += taskStatus[i].ulRunTimeCounter;
  }

  uint64_t currentTime = esp_timer_get_time(); // µs
  float timeDeltaSec = (currentTime - lastPrintTime) / 1e6f;
  lastPrintTime = currentTime;

  uint32_t runtimeDelta = totalRuntime - prevTotalRuntime;
  prevTotalRuntime = totalRuntime;

  // Uptime
  uint64_t uptimeSec = currentTime / 1000000;
  int hrs = uptimeSec / 3600;
  int mins = (uptimeSec % 3600) / 60;
  int secs = uptimeSec % 60;

  // Get actual CPU frequency
  uint32_t cpuFreqHz = esp_clk_cpu_freq(); // e.g., 240000000
  float cpuTotalPercent = (runtimeDelta / (float)(cpuFreqHz * timeDeltaSec)) * 100.0f;

  // Header
  Serial.write("\n=== System Monitor ===\n");
  Serial.printf("Temperature: %.1f°\n", temp);
  Serial.printf("Uptime-Format: %02d:%02d:%02d\n", hrs, mins, secs);
  Serial.print("Uptime:       ");
  Serial.println(uptimeSec);
  Serial.printf("CPU Usage (Total): %.1f%%\n", cpuTotalPercent);

  Serial.printf("RAM:   Free: %.1fkB | Min: %.1fkB | Total: %.1fkB\n",
                freeHeap / 1024.0f, minFreeHeap / 1024.0f, totalHeap / 1024.0f);

#ifdef BOARD_HAS_PSRAM
  Serial.printf("PSRAM: Free: %.1fkB | Total: %.1fkB\n",
                freePsram / 1024.0f, totalPsram / 1024.0f);
#endif // BOARD_HAS_PSRAM

  Serial.write("\nTask Name           State   CPU%    Stack Free\n");
  Serial.write("------------------------------------------------------\n");

  for (UBaseType_t i = 0; i < capturedTasks; i++)
  {
    const char *state;
    switch (taskStatus[i].eCurrentState)
    {
    case eRunning:
      state = "Run";
      break;
    case eReady:
      state = "Rdy";
      break;
    case eBlocked:
      state = "Blk";
      break;
    case eSuspended:
      state = "Sus";
      break;
    case eDeleted:
      state = "Del";
      break;
    default:
      state = "Unk";
      break;
    }

    std::string name(taskStatus[i].pcTaskName);
    uint32_t prevRuntime = prevTaskRuntime[name];
    uint32_t currentRuntime = taskStatus[i].ulRunTimeCounter;
    prevTaskRuntime[name] = currentRuntime;

    float taskDelta = (float)(currentRuntime - prevRuntime);
    float taskPercent = (runtimeDelta > 0) ? (taskDelta / runtimeDelta) * 100.0f : 0.0f;

    size_t stackFree = taskStatus[i].usStackHighWaterMark * sizeof(StackType_t);

    Serial.printf("%-18s %-6s %6.1f%%   %6d B\n",
                  taskStatus[i].pcTaskName,
                  state,
                  taskPercent,
                  stackFree);
  }

  vPortFree(taskStatus);

  // Applications Section
  Serial.write("\n== Applications ==\n");
  Serial.write("- Motor Controller\n");
  Serial.write("- Sensor Processor\n");
  Serial.write("- Command Processor\n");
}
#endif

/**
 * @brief Prints the help message with available commands.
 * @details This function prints a list of available commands and their descriptions to the serial console.
 */
void CommandProcessor::printHelp()
{
  Serial.write("\n===== Available Commands =====\n");
  Serial.write("MOTOR CONTROL:\n");
  Serial.write("  Mx:RL:y     - Set motor x forward at y% speed (x=1-4, y=0-100)\n");
  Serial.write("  Mx:LL:y     - Set motor x backward at y% speed\n");
  Serial.write("  Mx:STOP     - Stop motor x\n");
  Serial.write("  Mx:SPD:y    - Set motor x speed to y%\n");
  Serial.write("  Mx:STATUS   - Get motor x status\n");

  Serial.write("\nLED CONTROL:\n");
  Serial.write("  LED:ALL:ON     - Turn on all LEDs\n");
  Serial.write("  LED:ALL:OFF    - Turn off all LEDs\n");
  Serial.write("  LED:n:ON       - Turn on LED n (n=1-12)\n");
  Serial.write("  LED:n:OFF      - Turn off LED n\n");
  Serial.write("  LED:n:TOGGLE   - Toggle LED n\n");
  Serial.write("  LED:PATTERN:n  - Run LED pattern n (n=1-3)\n");

  Serial.write("\nSENSOR COMMANDS:\n");
  Serial.write("  SENSOR:READ:ALL   - Read all sensors\n");
  Serial.write("  SENSOR:READ:POWER - Read power sensor (INA219)\n");
  Serial.write("  SENSOR:READ:TEMP  - Read temperature sensors (TMP102)\n");
  Serial.write("  SENSOR:AUTO:ON:n  - Enable auto sensor readings every n seconds\n");
  Serial.write("  SENSOR:AUTO:OFF   - Disable auto sensor readings\n");

#ifdef ESP_WIFI_MQTT
  Serial.write("\nWIFI COMMANDS:\n");
  Serial.write("  WIFI:SSID:name    - Set WiFi SSID\n");
  Serial.write("  WIFI:PASSWORD:pwd - Set WiFi password\n");
  Serial.write("  WIFI:STATIC:ON    - Enable static IP\n");
  Serial.write("  WIFI:STATIC:OFF   - Disable static IP (use DHCP)\n");
  Serial.write("  WIFI:IP:x.x.x.x   - Set static IP address\n");
  Serial.write("  WIFI:GATEWAY:x.x.x.x - Set gateway address\n");
  Serial.write("  WIFI:SUBNET:x.x.x.x  - Set subnet mask\n");
  Serial.write("  WIFI:CONNECT      - Connect to WiFi using current settings\n");
  Serial.write("  WIFI:STATUS       - Show WiFi settings and status");

  Serial.write("\nMQTT COMMANDS:\n");
  Serial.write("  MQTT:SERVER:host  - Set MQTT server hostname/IP\n");
  Serial.write("  MQTT:PORT:n       - Set MQTT server port\n");
  Serial.write("  MQTT:USER:name    - Set MQTT username\n");
  Serial.write("  MQTT:PASSWORD:pwd - Set MQTT password\n");
  Serial.write("  MQTT:ID:id        - Set MQTT client ID\n");
  Serial.write("  MQTT:TOPIC:topic  - Set MQTT base topic\n");
  Serial.write("  MQTT:CONNECT      - Connect to MQTT server\n");
  Serial.write("  MQTT:PUBLISH      - Publish current status to MQTT\n");
  Serial.write("  MQTT:STATUS       - Show MQTT settings and status\n");
#endif
#ifdef ESP_STOP_SWITCH
  Serial.write("\nSTOP SWITCH COMMANDS:\n");
  Serial.write("  SWITCH:PIN1:n      - Set stop switch pin 1 default: 15\n");
  Serial.write("  SWITCH:PIN2:n      - Set stop switch pin 2 default: 16\n");
  Serial.write("  SWITCH:STATUS      - Show stop switch settings\n");
#endif

  Serial.write("\nSYSTEM COMMANDS:\n");
  Serial.write("  CLEAR        - Clear console\n");
  Serial.write("  REBOOT       - Reboot the system\n");
  Serial.write("  SAVE         - Save settings to flash\n");
  Serial.write("  STOP / Alert - Emergency stop all motors and LEDs off\n");
  Serial.write("  MONITOR      - Print system monitor information\n");
  Serial.write("  STATUS       - Print system status\n");
  Serial.write("  HELP         - Show this help message\n");
}
