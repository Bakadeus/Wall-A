/*
 * main_cpp.cpp
 *
 *  Created on: Oct 30, 2025
 *      Author: thhoguin
 */


extern "C" {
	#include "main.h"
	#include "cmsis_os.h"
}

#include <cstdio>
#include "test.hpp"

extern "C" void cppMain() {
    lol test;
    test.go();
    for (;;) {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

