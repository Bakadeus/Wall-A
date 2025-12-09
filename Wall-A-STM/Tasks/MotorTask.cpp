/*
 * MotorTask.cpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#include "MotorTask.hpp"
#include "Drv8262.hpp"
#include "Encoder.hpp"
#include "PID.hpp"
#include "Logger.hpp"
#include "StateMachine.hpp"

using namespace DriversCustom::Motor;
using namespace DriversCustom::Encoder;
using namespace Libs::Controller;
using namespace Libs::Utils;

namespace Tasks {

osThreadId_t MotorTask::threadId_ = nullptr;

void MotorTask::create() {
    const osThreadAttr_t attr = {
        .name = "MotorTask",
        .attr_bits = 0,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 1024
    };
    threadId_ = osThreadNew(threadFunc, nullptr, &attr);
}

void MotorTask::start() {
    // not needed - create starts it
}

static Drv8262 drv;
static DriversCustom::Encoder::Encoder encLeft;
static DriversCustom::Encoder::Encoder encRight;
static PID leftPid(0.1f, 0.01f, 0.0f);
static PID rightPid(0.1f, 0.01f, 0.0f);

void MotorTask::threadFunc(void* ) {
    drv.init();
    encLeft.init();
    encRight.init();

    const uint32_t periodMs = 1000 / 200; // default 200Hz
    while (true) {
        if (App::StateMachine::getState() == App::RobotState::EMERGENCY_STOP) {
            drv.setLeftDuty(0.0f);
            drv.setRightDuty(0.0f);
            osDelay(10);
            continue;
        }

        // TODO: read encoders and compute speed, then PID -> set duty
        // float speedL = ...; float speedR = ...;
        // float outL = leftPid.update(targetL, speedL, 1.0f/200.0f);
        // drv.setLeftDuty(outL);

        osDelay(periodMs);
    }
}

} // namespace Tasks

