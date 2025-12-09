/*
 * Logger.hpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#ifndef UTILS_LOGGER_HPP_
#define UTILS_LOGGER_HPP_

#pragma once
#include <cstdarg>

namespace Libs {
namespace Utils {

class Logger {
public:
    static void init();
    static void info(const char* fmt, ...);
    static void warn(const char* fmt, ...);
    static void error(const char* fmt, ...);
};

} // namespace Utils
} // namespace Libs

#endif /* UTILS_LOGGER_HPP_ */
