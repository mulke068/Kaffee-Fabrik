/**
 * @file GlobalConfig.h
 * @author Kevin Muller (@kevbcef.com)
 * @brief Global configuration file for the ESP32 project.
 * This file contains the configuration settings for the project.
 * @version 1.0
 * @date 2025-04-16
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef GLOBAL_CONFIG_H
#define GLOBAL_CONFIG_H

// #define ESP_WIFI_MQTT
#define ESP_STOP_SWITCH
// #define BOARD_HAS_PSRAM

#define MONITORING_ENABLED

#define FIRMWARE_VERSION "1.0.0"

// #define DEBUG_ENABLED

#ifdef DEBUG_ENABLED
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(x, ...) Serial.printf(x, __VA_ARGS__)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(x, ...)
#endif // DEBUG_ENABLED

#endif // GLOBAL_CONFIG_H