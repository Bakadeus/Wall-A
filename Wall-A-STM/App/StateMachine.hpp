/*
 * StateMachine.hpp
 *
 *  Created on: Dec 8, 2025
 *      Author: thhoguin
 */

#ifndef STATEMACHINE_HPP_
#define STATEMACHINE_HPP_


#include <cstdint>

namespace App {

enum class RobotState : uint8_t {
    INIT = 0,
    IDLE,
    RUNNING,
    ERROR,
    EMERGENCY_STOP
};

class StateMachine {
public:
    static void init();
    static void setState(RobotState s);
    static RobotState getState();
};

} // namespace App




#endif /* STATEMACHINE_HPP_ */
