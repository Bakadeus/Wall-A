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

#include <iostream>
#include "Wall-A_App.hpp"

extern "C" void cppMain() {
	std::cout << "Hello from C++17 main!" << std::endl;
	WallAApp_Init();
}
