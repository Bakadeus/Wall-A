################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../DriversCustom/Ax12/Ax12.cpp 

OBJS += \
./DriversCustom/Ax12/Ax12.o 

CPP_DEPS += \
./DriversCustom/Ax12/Ax12.d 


# Each subdirectory must supply rules for building sources it contributes
DriversCustom/Ax12/%.o DriversCustom/Ax12/%.su DriversCustom/Ax12/%.cyclo: ../DriversCustom/Ax12/%.cpp DriversCustom/Ax12/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../App -I../Tasks -I../DriversCustom/Motor -I../DriversCustom/Encoder -I../DriversCustom/PwmServo -I../DriversCustom/Distance -I../DriversCustom/Power -I../DriversCustom/Ax12 -I../Libs/Utils -I../Libs/PID -I../Libs/Filters -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DriversCustom-2f-Ax12

clean-DriversCustom-2f-Ax12:
	-$(RM) ./DriversCustom/Ax12/Ax12.cyclo ./DriversCustom/Ax12/Ax12.d ./DriversCustom/Ax12/Ax12.o ./DriversCustom/Ax12/Ax12.su

.PHONY: clean-DriversCustom-2f-Ax12

