/*
 * Logger.cpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#include "Logger.hpp"
#include "main.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

extern UART_HandleTypeDef huart1;

static constexpr size_t LOG_BUFFER_SIZE = 256;

namespace Libs {
namespace Utils {

void Logger::init() {
    // TODO: initialize USB CDC or UART for printf if needed
}

static void vprint(const char* tag, const char* fmt, va_list args)
{
    char buffer[LOG_BUFFER_SIZE];

    // Prefix
    int len = std::snprintf(buffer, LOG_BUFFER_SIZE, "[%s] ", tag);
    if (len < 0 || len >= (int)LOG_BUFFER_SIZE)
        return;

    // Message
    int msg_len = std::vsnprintf(buffer + len,
                                 LOG_BUFFER_SIZE - len - 2,
                                 fmt,
                                 args);

    if (msg_len < 0)
        return;

    size_t total = len + msg_len;

    // Append CRLF
    buffer[total++] = '\r';
    buffer[total++] = '\n';

    HAL_UART_Transmit(&huart1,
                      reinterpret_cast<uint8_t*>(buffer),
                      total,
                      HAL_MAX_DELAY);
}

void Logger::info(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprint("I", fmt, args);
    va_end(args);
}

void Logger::warn(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprint("W", fmt, args);
    va_end(args);
}

void Logger::error(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprint("E", fmt, args);
    va_end(args);
}

} // namespace Utils
} // namespace Libs
