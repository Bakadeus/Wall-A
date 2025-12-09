/*
 * PololuIR.hpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#ifndef DISTANCE_POLOLUIR_HPP_
#define DISTANCE_POLOLUIR_HPP_

#pragma once

namespace DriversCustom {
namespace Distance {

class PololuIR {
public:
    PololuIR() = default;
    void init();
    float readMeters();
};

} // namespace Distance
} // namespace DriversCustom

#endif /* DISTANCE_POLOLUIR_HPP_ */
