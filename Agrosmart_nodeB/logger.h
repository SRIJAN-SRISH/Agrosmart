#pragma once
#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include "config.h"

#define LOG_LEVEL_DEBUG 0
#define LOG_LEVEL_INFO  1
#define LOG_LEVEL_WARN  2
#define LOG_LEVEL_ERROR 3

// Default log level
#ifndef GLOBAL_LOG_LEVEL
#ifdef DEVELOPMENT_MODE
#define GLOBAL_LOG_LEVEL LOG_LEVEL_DEBUG
#else
#define GLOBAL_LOG_LEVEL LOG_LEVEL_INFO
#endif
#endif

// Base macro for logging
// Format: [TIME_MS ] [MODULE  ] [LEVEL] Message
#define LOG_FORMAT(level_str, module, format, ...) \
    Serial.printf("[%08lu] [%-8s] [%-5s] " format "\r\n", (unsigned long)millis(), module, level_str, ##__VA_ARGS__)

#if GLOBAL_LOG_LEVEL <= LOG_LEVEL_DEBUG
    #define LOG_DEBUG(module, format, ...) LOG_FORMAT("DEBUG", module, format, ##__VA_ARGS__)
#else
    #define LOG_DEBUG(module, format, ...)
#endif

#if GLOBAL_LOG_LEVEL <= LOG_LEVEL_INFO
    #define LOG_INFO(module, format, ...)  LOG_FORMAT("INFO", module, format, ##__VA_ARGS__)
#else
    #define LOG_INFO(module, format, ...)
#endif

#if GLOBAL_LOG_LEVEL <= LOG_LEVEL_WARN
    #define LOG_WARN(module, format, ...)  LOG_FORMAT("WARN", module, format, ##__VA_ARGS__)
#else
    #define LOG_WARN(module, format, ...)
#endif

#if GLOBAL_LOG_LEVEL <= LOG_LEVEL_ERROR
    #define LOG_ERROR(module, format, ...) LOG_FORMAT("ERROR", module, format, ##__VA_ARGS__)
#else
    #define LOG_ERROR(module, format, ...)
#endif

#endif // LOGGER_H
