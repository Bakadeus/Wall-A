/*
 * main_cpp.cpp
 *
 *  Created on: Dec 2, 2025
 *      Author: thhoguin
 */

extern "C" {
	#include "main.h"
	#include "cmsis_os.h"
}

extern UART_HandleTypeDef huart1;

#include <iostream>
#include <cstring>
#include "Wall-A_App.hpp"

extern "C" void cppMain() {
	WallAApp_Init();
}
