################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Inc/rtos/museq/museq.cpp 

OBJS += \
./Core/Inc/rtos/museq/museq.o 

CPP_DEPS += \
./Core/Inc/rtos/museq/museq.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/rtos/museq/%.o Core/Inc/rtos/museq/%.su Core/Inc/rtos/museq/%.cyclo: ../Core/Inc/rtos/museq/%.cpp Core/Inc/rtos/museq/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32L5xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-rtos-2f-museq

clean-Core-2f-Inc-2f-rtos-2f-museq:
	-$(RM) ./Core/Inc/rtos/museq/museq.cyclo ./Core/Inc/rtos/museq/museq.d ./Core/Inc/rtos/museq/museq.o ./Core/Inc/rtos/museq/museq.su

.PHONY: clean-Core-2f-Inc-2f-rtos-2f-museq

