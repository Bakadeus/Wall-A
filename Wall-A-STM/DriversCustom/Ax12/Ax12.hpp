/*
 * Ax12.hpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#ifndef AX12_AX12_HPP_
#define AX12_AX12_HPP_

#pragma once
#include <cstdint>
#include <vector>

namespace DriversCustom {
namespace Ax12 {

class Ax12 {
public:
    Ax12() = default;
    void init(); // configure UART and DE/RE pin for half-duplex
    bool ping(uint8_t id);
    bool write(uint8_t id, uint8_t reg, const std::vector<uint8_t>& data);
    bool read(uint8_t id, uint8_t reg, uint8_t len, std::vector<uint8_t>& out);
};

} // namespace Ax12
} // namespace DriversCustom


#endif /* AX12_AX12_HPP_ */
