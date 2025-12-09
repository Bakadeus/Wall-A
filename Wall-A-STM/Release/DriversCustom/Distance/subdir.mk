################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../DriversCustom/Distance/B5W_LB2101.cpp \
../DriversCustom/Distance/PololuIR.cpp 

OBJS += \
./DriversCustom/Distance/B5W_LB2101.o \
./DriversCustom/Distance/PololuIR.o 

CPP_DEPS += \
./DriversCustom/Distance/B5W_LB2101.d \
./DriversCustom/Distance/PololuIR.d 


# Each subdirectory must supply rules for building sources it contributes
DriversCustom/Distance/%.o DriversCustom/Distance/%.su DriversCustom/Distance/%.cyclo: ../DriversCustom/Distance/%.cpp DriversCustom/Distance/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DriversCustom-2f-Distance

clean-DriversCustom-2f-Distance:
	-$(RM) ./DriversCustom/Distance/B5W_LB2101.cyclo ./DriversCustom/Distance/B5W_LB2101.d ./DriversCustom/Distance/B5W_LB2101.o ./DriversCustom/Distance/B5W_LB2101.su ./DriversCustom/Distance/PololuIR.cyclo ./DriversCustom/Distance/PololuIR.d ./DriversCustom/Distance/PololuIR.o ./DriversCustom/Distance/PololuIR.su

.PHONY: clean-DriversCustom-2f-Distance

