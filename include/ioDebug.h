#ifndef IO_DEBUG_H
#define IO_DEBUG_H

#include <PrintEx.h>

// StreamEx mySerial = Serial; // <---- DECLARE THIS IN main.cpp

// Enable or disable debug output
#define DEBUG_ENABLED

// Debug macro
#ifdef DEBUG_ENABLED
  // Define the debug level
  #define DEBUG_LEVEL 1  // DEBUG_LEVEL: VERBOSE 0, INFO 1, WARNING 2, ERROR 3

  #define DPRINTF(level, ...) \
    (level >= DEBUG_LEVEL) ? mySerial.printf(__VA_ARGS__) : 0
#else
  #define DPRINTF(level, ...)
#endif

#endif
