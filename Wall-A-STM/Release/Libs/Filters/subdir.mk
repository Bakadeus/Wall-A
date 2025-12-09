################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Libs/Filters/MovingAverage.cpp 

OBJS += \
./Libs/Filters/MovingAverage.o 

CPP_DEPS += \
./Libs/Filters/MovingAverage.d 


# Each subdirectory must supply rules for building sources it contributes
Libs/Filters/%.o Libs/Filters/%.su Libs/Filters/%.cyclo: ../Libs/Filters/%.cpp Libs/Filters/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Libs-2f-Filters

clean-Libs-2f-Filters:
	-$(RM) ./Libs/Filters/MovingAverage.cyclo ./Libs/Filters/MovingAverage.d ./Libs/Filters/MovingAverage.o ./Libs/Filters/MovingAverage.su

.PHONY: clean-Libs-2f-Filters

