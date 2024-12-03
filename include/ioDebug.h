/**
 * @file ioDebug.h
 *
 * ioDebug is an Arduino debug library that leverages printf statements for streamlined debugging.
 *
 * Utilize its statements to display debug output in the terminal console.
 * For release versions, simply comment out DEBUG_ENABLED.
 * This ensures that string memory is excluded from compilation, optimizing memory for runtime tasks.
 *
 * @copyright (c) 2024 hansaplasst
 * @author hansaplasst
 *
 * @version 0.0.1
 * @license MIT
 */

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
