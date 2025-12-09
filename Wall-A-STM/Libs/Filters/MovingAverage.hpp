/*
 * MovingAverage.hpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#ifndef FILTERS_MOVINGAVERAGE_HPP_
#define FILTERS_MOVINGAVERAGE_HPP_

#pragma once
#include <vector>
#include <cstddef>

namespace Libs {
namespace Filter {

class MovingAverage {
public:
    explicit MovingAverage(std::size_t window = 8);
    void push(float v);
    float value() const;
    void reset();
private:
    std::vector<float> buf_;
    std::size_t idx_;
    float sum_;
    bool full_;
};

} // namespace Filter
} // namespace Libs


#endif /* FILTERS_MOVINGAVERAGE_HPP_ */
