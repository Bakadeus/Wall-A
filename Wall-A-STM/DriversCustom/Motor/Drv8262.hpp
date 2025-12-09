/*
 * Drv8262.hpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#ifndef MOTOR_DRV8262_HPP_
#define MOTOR_DRV8262_HPP_

#pragma once
#include <cstdint>

namespace DriversCustom {
namespace Motor {

class Drv8262 {
public:
    Drv8262() = default;
    void init();                 // call after CubeMX MX_*_Init
    void setLeftDuty(float d);   // -1..1
    void setRightDuty(float d);  // -1..1
    void enable(bool en);
};

} // namespace Motor
} // namespace DriversCustom

#endif /* MOTOR_DRV8262_HPP_ */
