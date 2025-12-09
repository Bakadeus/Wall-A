/*
 * PID.hpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#ifndef PID_PID_HPP_
#define PID_PID_HPP_

#pragma once

namespace Libs {
namespace Controller {

class PID {
public:
    PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f);
    void setGains(float kp, float ki, float kd);
    float update(float setpoint, float measurement, float dt);
    void reset();
private:
    float kp_, ki_, kd_;
    float integ_;
    float prevErr_;
};

} // namespace Controller
} // namespace Libs

#endif /* PID_PID_HPP_ */
