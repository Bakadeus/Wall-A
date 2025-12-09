/*
 * B5W_LB2101.hpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#ifndef DISTANCE_B5W_LB2101_HPP_
#define DISTANCE_B5W_LB2101_HPP_

#pragma once

namespace DriversCustom {
namespace Distance {

class B5W_LB2101 {
public:
    B5W_LB2101() = default;
    void init();
    float readMeters(); // return distance in meters
};

} // namespace Distance
} // namespace DriversCustom


#endif /* DISTANCE_B5W_LB2101_HPP_ */
