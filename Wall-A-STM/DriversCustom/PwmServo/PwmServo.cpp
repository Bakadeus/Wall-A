/*
 * PwmServo.cpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#include "PwmServo.hpp"
#include "main.h"
#include "Logger.hpp"

namespace DriversCustom {
namespace PwmServo {

void PwmServo::init() {
    // TODO: start TIM PWM channels if not started by CubeMX
    Libs::Utils::Logger::info("PwmServo init");
}

void PwmServo::setPulseUs(uint8_t channel, uint16_t pulse_us) {
    (void)channel; (void)pulse_us;
    // TODO: convert pulse_us to CCR value and write to TIMx->CCRy
}

} // namespace PwmServo
} // namespace DriversCustom
