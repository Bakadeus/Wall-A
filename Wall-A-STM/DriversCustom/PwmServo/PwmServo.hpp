/*
 * PwmServo.hpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#ifndef PWMSERVO_PWMSERVO_HPP_
#define PWMSERVO_PWMSERVO_HPP_

#pragma once
#include <cstdint>

namespace DriversCustom {
namespace PwmServo {

class PwmServo {
public:
    PwmServo() = default;
    void init(); // configure PWM timers if needed
    void setPulseUs(uint8_t channel, uint16_t pulse_us); // 500..2500 typical
};

} // namespace PwmServo
} // namespace DriversCustom

#endif /* PWMSERVO_PWMSERVO_HPP_ */
