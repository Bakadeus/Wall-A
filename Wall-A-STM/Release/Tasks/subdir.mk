################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Tasks/CommunicationTask.cpp \
../Tasks/ControlTask.cpp \
../Tasks/MotorTask.cpp \
../Tasks/SensorTask.cpp 

OBJS += \
./Tasks/CommunicationTask.o \
./Tasks/ControlTask.o \
./Tasks/MotorTask.o \
./Tasks/SensorTask.o 

CPP_DEPS += \
./Tasks/CommunicationTask.d \
./Tasks/ControlTask.d \
./Tasks/MotorTask.d \
./Tasks/SensorTask.d 


# Each subdirectory must supply rules for building sources it contributes
Tasks/%.o Tasks/%.su Tasks/%.cyclo: ../Tasks/%.cpp Tasks/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Tasks

clean-Tasks:
	-$(RM) ./Tasks/CommunicationTask.cyclo ./Tasks/CommunicationTask.d ./Tasks/CommunicationTask.o ./Tasks/CommunicationTask.su ./Tasks/ControlTask.cyclo ./Tasks/ControlTask.d ./Tasks/ControlTask.o ./Tasks/ControlTask.su ./Tasks/MotorTask.cyclo ./Tasks/MotorTask.d ./Tasks/MotorTask.o ./Tasks/MotorTask.su ./Tasks/SensorTask.cyclo ./Tasks/SensorTask.d ./Tasks/SensorTask.o ./Tasks/SensorTask.su

.PHONY: clean-Tasks

