/*
 * Wall-A_App.hpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#ifndef WALL_A_APP_HPP_
#define WALL_A_APP_HPP_

#pragma once
#include <cstdint>

extern "C" {
void WallAApp_Init(); // call from your existing main_cpp.cpp
}

namespace App {
class WallAApp {
public:
    static void init();
    static void shutdown();
private:
    WallAApp() = delete;
};
} // namespace App



#endif /* WALL_A_APP_HPP_ */
