/*
 * CommunicationTask.hpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#ifndef COMMUNICATIONTASK_HPP_
#define COMMUNICATIONTASK_HPP_

#pragma once
#include "cmsis_os2.h"

namespace Tasks {
class CommunicationTask {
public:
    static void create();
private:
    static void threadFunc(void* arg);
    static osThreadId_t threadId_;
};
} // namespace Tasks

#endif /* COMMUNICATIONTASK_HPP_ */
