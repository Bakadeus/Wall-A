/*
 * Ax12.cpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#include "Ax12.hpp"
#include "main.h"
#include "Logger.hpp"

namespace DriversCustom {
namespace Ax12 {

void Ax12::init() {
    // TODO: start UART (HAL_UART_Receive_IT or DMA) and setup DE/RE GPIO
    Libs::Utils::Logger::info("AX12 init");
}

bool Ax12::ping(uint8_t id) {
    (void)id;
    // TODO: implement protocol
    return false;
}

bool Ax12::write(uint8_t id, uint8_t reg, const std::vector<uint8_t>& data) {
    (void)id; (void)reg; (void)data;
    // TODO: implement
    return false;
}

bool Ax12::read(uint8_t id, uint8_t reg, uint8_t len, std::vector<uint8_t>& out) {
    (void)id; (void)reg; (void)len; (void)out;
    // TODO: implement
    return false;
}

} // namespace Ax12
} // namespace DriversCustom

