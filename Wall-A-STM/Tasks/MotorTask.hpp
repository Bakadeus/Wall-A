/*
 * MotorTask.hpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#ifndef MOTORTASK_HPP_
#define MOTORTASK_HPP_

#pragma once
#include "cmsis_os2.h"

namespace Tasks {

class MotorTask {
public:
    static void create();
    static void start(); // used if you want direct start
private:
    static void threadFunc(void* arg);
    static osThreadId_t threadId_;
};

} // namespace Tasks

#endif /* MOTORTASK_HPP_ */
