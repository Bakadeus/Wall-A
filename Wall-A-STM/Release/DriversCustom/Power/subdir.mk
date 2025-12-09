################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../DriversCustom/Power/PowerManager.cpp 

OBJS += \
./DriversCustom/Power/PowerManager.o 

CPP_DEPS += \
./DriversCustom/Power/PowerManager.d 


# Each subdirectory must supply rules for building sources it contributes
DriversCustom/Power/%.o DriversCustom/Power/%.su DriversCustom/Power/%.cyclo: ../DriversCustom/Power/%.cpp DriversCustom/Power/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DriversCustom-2f-Power

clean-DriversCustom-2f-Power:
	-$(RM) ./DriversCustom/Power/PowerManager.cyclo ./DriversCustom/Power/PowerManager.d ./DriversCustom/Power/PowerManager.o ./DriversCustom/Power/PowerManager.su

.PHONY: clean-DriversCustom-2f-Power

