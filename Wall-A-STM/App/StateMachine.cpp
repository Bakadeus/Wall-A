/*
 * StateMachine.cpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#include "StateMachine.hpp"
#include "Logger.hpp"

namespace App {

static RobotState currentState = RobotState::INIT;

void StateMachine::init() {
    currentState = RobotState::INIT;
    Libs::Utils::Logger::info("StateMachine: INIT");
}

void StateMachine::setState(RobotState s) {
    currentState = s;
    // optional logging:
    Libs::Utils::Logger::info("StateMachine: state changed");
}

RobotState StateMachine::getState() {
    return currentState;
}

} // namespace App



