/*
 * CommunicationTask.cpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#include "CommunicationTask.hpp"
#include "Logger.hpp"
#include "main.h" // for USB / Ethernet handles if needed

namespace Tasks {

osThreadId_t CommunicationTask::threadId_ = nullptr;

void CommunicationTask::create() {
    const osThreadAttr_t attr = {
        .name = "CommTask",
        .stack_size = 1024
    };
    threadId_ = osThreadNew(threadFunc, nullptr, &attr);
}

void CommunicationTask::threadFunc(void*) {
    // TODO: setup USB CDC / lwIP sockets if needed. Use non-blocking APIs.
    while (true) {
        // Example: poll USB CDC for commands and dispatch
        osDelay(50);
    }
}

} // namespace Tasks
