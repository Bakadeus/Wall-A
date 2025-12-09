/*
 * PowerManager.hpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#ifndef POWER_POWERMANAGER_HPP_
#define POWER_POWERMANAGER_HPP_

#pragma once

namespace DriversCustom {
namespace Power {

class PowerManager {
public:
    PowerManager() = delete;
    static void init();
    static float read5V();
    static float read12V();
    static float read24V();
    static bool is5Vok(float threshold = 4.5f);
    static bool is12Vok(float threshold = 11.0f);
    static bool is24Vok(float threshold = 22.0f);
};

} // namespace Power
} // namespace DriversCustom


#endif /* POWER_POWERMANAGER_HPP_ */
