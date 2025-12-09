/*
 * Drv8262.cpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#include "Drv8262.hpp"
#include "main.h"
#include "Logger.hpp"

namespace DriversCustom {
namespace Motor {

void Drv8262::init() {
    // TODO: start PWM timers with HAL_TIM_PWM_Start(&htimX, TIM_CHANNEL_y);
    // Example: HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    Libs::Utils::Logger::info("Drv8262 init");
}

static inline uint32_t dutyToCCR(float duty) {
    // TODO: map -1..1 to timer CCR (0 .. TIM->ARR)
    (void)duty;
    return 0;
}

void Drv8262::setLeftDuty(float d) {
    // TODO: write CCR to left motor PWM timer channel
    (void)d;
}

void Drv8262::setRightDuty(float d) {
    // TODO: write CCR to right motor PWM timer channel
    (void)d;
}

void Drv8262::enable(bool en) {
    // TODO: control EN GPIO pin for driver
    (void)en;
}

} // namespace Motor
} // namespace DriversCustom

