/*
 * ControlTask.hpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#ifndef CONTROLTASK_HPP_
#define CONTROLTASK_HPP_

#pragma once
#include "cmsis_os2.h"

namespace Tasks {
class ControlTask {
public:
    static void create();
private:
    static void threadFunc(void* arg);
    static osThreadId_t threadId_;
};
} // namespace Tasks

#endif /* CONTROLTASK_HPP_ */
