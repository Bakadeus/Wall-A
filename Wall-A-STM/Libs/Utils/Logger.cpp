/*
 * Logger.cpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#include "Logger.hpp"
#include "main.h"
#include <cstdio>
#include <cstdarg>

namespace Libs {
namespace Utils {

void Logger::init() {
    // TODO: initialize USB CDC or UART for printf if needed
}

static void vprint(const char* tag, const char* fmt, va_list args) {
    (void)tag; (void)fmt; (void)args;
    // If you have HAL printf/retarget implemented, you can:
    // vprintf(fmt, args);
}

void Logger::info(const char* fmt, ...) {
    va_list args; va_start(args, fmt);
    vprint("I", fmt, args);
    va_end(args);
}

void Logger::warn(const char* fmt, ...) {
    va_list args; va_start(args, fmt);
    vprint("W", fmt, args);
    va_end(args);
}

void Logger::error(const char* fmt, ...) {
    va_list args; va_start(args, fmt);
    vprint("E", fmt, args);
    va_end(args);
}

} // namespace Utils
} // namespace Libs
