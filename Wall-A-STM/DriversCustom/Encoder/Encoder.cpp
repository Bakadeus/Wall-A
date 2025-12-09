/*
 * Encoder.cpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#include "Encoder.hpp"
#include "main.h"
#include "Logger.hpp"

namespace DriversCustom {
namespace Encoder {

void Encoder::init() {
    // TODO: call HAL_TIM_Encoder_Start_IT(&htimX, TIM_CHANNEL_ALL);
    extension_ = 0;
    Libs::Utils::Logger::info("Encoder init");
}

int32_t Encoder::read() {
    // TODO: combine extension_ and TIM counter to 32-bit value
    // Example: return (extension_ << 16) | (int32_t)(TIMX->CNT & 0xFFFF);
    return extension_;
}

void Encoder::reset() {
    extension_ = 0;
    // TODO: reset TIM counter (htimX.Instance->CNT = 0)
}

void Encoder::isrNotify() {
    // TODO: detect overflow/underflow and update extension_
    // Keep this function minimal as it may be called from ISR
    extension_ += 0; // placeholder
}

} // namespace Encoder
} // namespace DriversCustom

