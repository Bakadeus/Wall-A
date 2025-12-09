/*
 * MovingAverage.cpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#include "MovingAverage.hpp"

namespace Libs {
namespace Filter {

MovingAverage::MovingAverage(std::size_t window) : buf_(window, 0.0f), idx_(0), sum_(0.0f), full_(false) {}

void MovingAverage::push(float v) {
    sum_ -= buf_[idx_];
    buf_[idx_] = v;
    sum_ += v;
    idx_ = (idx_ + 1) % buf_.size();
    if (idx_ == 0) full_ = true;
}

float MovingAverage::value() const {
    std::size_t len = full_ ? buf_.size() : idx_;
    return (len == 0) ? 0.0f : (sum_ / static_cast<float>(len));
}

void MovingAverage::reset() {
    std::fill(buf_.begin(), buf_.end(), 0.0f);
    idx_ = 0; sum_ = 0.0f; full_ = false;
}

} // namespace Filter
} // namespace Libs

