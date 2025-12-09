/*
 * PID.cpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#include "PID.hpp"

namespace Libs {
namespace Controller {

PID::PID(float kp, float ki, float kd) : kp_(kp), ki_(ki), kd_(kd), integ_(0.0f), prevErr_(0.0f) {}

void PID::setGains(float kp, float ki, float kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
}

float PID::update(float setpoint, float measurement, float dt) {
    float err = setpoint - measurement;
    integ_ += err * dt;
    float deriv = (dt > 0.0f) ? (err - prevErr_) / dt : 0.0f;
    prevErr_ = err;
    return kp_ * err + ki_ * integ_ + kd_ * deriv;
}

void PID::reset() {
    integ_ = 0.0f;
    prevErr_ = 0.0f;
}

} // namespace Controller
} // namespace Libs
