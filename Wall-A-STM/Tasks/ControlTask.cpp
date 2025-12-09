/*
 * ControlTask.cpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#include "ControlTask.hpp"
#include "Logger.hpp"
#include "StateMachine.hpp"

namespace Tasks {

osThreadId_t ControlTask::threadId_ = nullptr;

void ControlTask::create() {
    const osThreadAttr_t attr = {
        .name = "ControlTask",
        .stack_size = 1024
    };
    threadId_ = osThreadNew(threadFunc, nullptr, &attr);
}

void ControlTask::threadFunc(void*) {
    const uint32_t ctrlMs = 20;
    while (true) {
        // High-level behaviors / state machine updates
        if (App::StateMachine::getState() == App::RobotState::RUNNING) {
            // TODO: run navigation/behaviour code
        }
        osDelay(ctrlMs);
    }
}

} // namespace Tasks
