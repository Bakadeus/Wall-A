/*
 * SensorTask.hpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#ifndef SENSORTASK_HPP_
#define SENSORTASK_HPP_

#pragma once
#include "cmsis_os2.h"

namespace Tasks {
class SensorTask {
public:
    static void create();
private:
    static void threadFunc(void* arg);
    static osThreadId_t threadId_;
};
} // namespace Tasks

#endif /* SENSORTASK_HPP_ */
