/*
 * Wall-A_App.cpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#include "Wall-A_App.hpp"
#include "StateMachine.hpp"
#include "MotorTask.hpp"
#include "SensorTask.hpp"
#include "ControlTask.hpp"
#include "CommunicationTask.hpp"
#include "Logger.hpp"

extern "C" void WallAApp_Init() {
    App::WallAApp::init();
}

namespace App {

void WallAApp::init() {
    Libs::Utils::Logger::init();
    StateMachine::init();

    // Create tasks (CMSIS-RTOS v2)
    Tasks::MotorTask::create();
    Tasks::SensorTask::create();
    Tasks::ControlTask::create();
    Tasks::CommunicationTask::create();

    Libs::Utils::Logger::info("RobotApp initialized");
}

void WallAApp::shutdown() {
    // TODO: implement safe shutdown if needed
    Libs::Utils::Logger::info("RobotApp shutdown");
}

} // namespace App


