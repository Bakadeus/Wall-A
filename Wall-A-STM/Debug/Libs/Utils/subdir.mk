################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Libs/Utils/Logger.cpp 

OBJS += \
./Libs/Utils/Logger.o 

CPP_DEPS += \
./Libs/Utils/Logger.d 


# Each subdirectory must supply rules for building sources it contributes
Libs/Utils/%.o Libs/Utils/%.su Libs/Utils/%.cyclo: ../Libs/Utils/%.cpp Libs/Utils/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../App -I../Tasks -I../DriversCustom/Motor -I../DriversCustom/Encoder -I../DriversCustom/PwmServo -I../DriversCustom/Distance -I../DriversCustom/Power -I../DriversCustom/Ax12 -I../Libs/Utils -I../Libs/PID -I../Libs/Filters -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Libs-2f-Utils

clean-Libs-2f-Utils:
	-$(RM) ./Libs/Utils/Logger.cyclo ./Libs/Utils/Logger.d ./Libs/Utils/Logger.o ./Libs/Utils/Logger.su

.PHONY: clean-Libs-2f-Utils

