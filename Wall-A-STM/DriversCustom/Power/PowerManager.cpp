/*
 * PowerManager.cpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#include "PowerManager.hpp"
#include "main.h"
#include "Logger.hpp"

namespace DriversCustom {
namespace Power {

void PowerManager::init() {
    // TODO: init ADC channels to monitor voltages
    Libs::Utils::Logger::info("PowerManager init");
}

float PowerManager::read5V() {
    // TODO: read and convert ADC
    return 5.0f;
}

float PowerManager::read12V() {
    return 12.0f;
}

float PowerManager::read24V() {
    return 24.0f;
}

bool PowerManager::is5Vok(float threshold) {
    return read5V() >= threshold;
}

bool PowerManager::is12Vok(float threshold) {
    return read12V() >= threshold;
}

bool PowerManager::is24Vok(float threshold) {
    return read24V() >= threshold;
}

} // namespace Power
} // namespace DriversCustom

