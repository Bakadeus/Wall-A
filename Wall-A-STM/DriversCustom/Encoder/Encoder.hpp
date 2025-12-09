/*
 * Encoder.hpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#ifndef ENCODER_ENCODER_HPP_
#define ENCODER_ENCODER_HPP_

#pragma once
#include <cstdint>

namespace DriversCustom {
namespace Encoder {

class Encoder {
public:
    Encoder() = default;
    void init();        // start TIM encoder and interrupts if needed
    int32_t read();     // extended 32-bit position
    void reset();
    void isrNotify();   // call from TIM ISR wrapper (if you wire it)
private:
    volatile int32_t extension_ = 0;
};

} // namespace Encoder
} // namespace DriversCustom

#endif /* ENCODER_ENCODER_HPP_ */
