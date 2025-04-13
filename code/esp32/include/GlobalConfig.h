#ifndef GLOBAL_CONFIG_H
#define GLOBAL_CONFIG_H

// #define ESP_WIFI_MQTT
#define ESP_STOP_SWITCH
#define BOARD_HAS_PSRAM

#define FIRMWARE_VERSION "0.1.5"

// #define DEBUG_ENABLED

#ifdef DEBUG_ENABLED
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(x, ...) Serial.printf(x, __VA_ARGS__)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(x, ...)
#endif

#endif