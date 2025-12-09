# Wall-A
The beggining of a great adventure in robotics.


## Architecture

/App/                        # high-level architecture
    ├── RobotApp.cpp
    ├── RobotApp.hpp
    ├── StateMachine.cpp
    └── StateMachine.hpp

/DriversCustom/              # C++ drivers (NOT Cube drivers)
    ├── Motor/
    │    ├── Drv8262.cpp
    │    └── Drv8262.hpp
    ├── Encoder/
    │    ├── Encoder.cpp
    │    └── Encoder.hpp
    ├── Ax12/
    │    ├── Ax12.cpp
    │    └── Ax12.hpp
    ├── PwmServo/
    │    ├── PwmServo.cpp
    │    └── PwmServo.hpp
    ├── Distance/
    │    ├── B5W_LB2101.cpp
    │    ├── B5W_LB2101.hpp
    │    ├── PololuIR.cpp
    │    └── PololuIR.hpp
    └── Power/
         ├── PowerManager.cpp
         └── PowerManager.hpp

/Tasks/                      # FreeRTOS C++ task classes
    ├── MotorTask.cpp
    ├── MotorTask.hpp
    ├── SensorTask.cpp
    ├── SensorTask.hpp
    ├── ControlTask.cpp
    ├── ControlTask.hpp
    ├── CommunicationTask.cpp
    └── CommunicationTask.hpp

/Libs/                        # Extern libs
    ├── PID/
    ├── Filters/
    └── Utils/
