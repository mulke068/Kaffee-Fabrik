#ifndef LOGGER_H
#define LOGGER_H

#include "GlobalConfig.h"
#include <Arduino.h>

enum LogLevel {
  LOG_NONE = 0,
  LOG_ERROR,
  LOG_WARNING,
  LOG_INFO,
  LOG_DEBUG,
  LOG_VERBOSE
};

class Logger {
public:
  static void begin(unsigned long baudRate = 115200);
  static void setLogLevel(LogLevel level);
  
  static void error(const char* module, const char* message, ...);
  static void warning(const char* module, const char* message, ...);
  static void info(const char* module, const char* message, ...);
  static void debug(const char* module, const char* message, ...);
  static void verbose(const char* module, const char* message, ...);
  
private:
  static LogLevel _currentLevel;
  static bool _initialized;
  static void log(LogLevel level, const char* levelStr, const char* module, const char* format, va_list args);
};

// Convenience macros
#define LOG_ERROR(module, message, ...) Logger::error(module, message, ##__VA_ARGS__)
#define LOG_WARNING(module, message, ...) Logger::warning(module, message, ##__VA_ARGS__)
#define LOG_INFO(module, message, ...) Logger::info(module, message, ##__VA_ARGS__)
#define LOG_DEBUG(module, message, ...) Logger::debug(module, message, ##__VA_ARGS__)
#define LOG_VERBOSE(module, message, ...) Logger::verbose(module, message, ##__VA_ARGS__)

#endif