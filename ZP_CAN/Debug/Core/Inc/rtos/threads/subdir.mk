################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Inc/rtos/threads/am_threads.cpp \
../Core/Inc/rtos/threads/sm_threads.cpp \
../Core/Inc/rtos/threads/unified_threads.cpp 

OBJS += \
./Core/Inc/rtos/threads/am_threads.o \
./Core/Inc/rtos/threads/sm_threads.o \
./Core/Inc/rtos/threads/unified_threads.o 

CPP_DEPS += \
./Core/Inc/rtos/threads/am_threads.d \
./Core/Inc/rtos/threads/sm_threads.d \
./Core/Inc/rtos/threads/unified_threads.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/rtos/threads/%.o Core/Inc/rtos/threads/%.su Core/Inc/rtos/threads/%.cyclo: ../Core/Inc/rtos/threads/%.cpp Core/Inc/rtos/threads/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32L5xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-rtos-2f-threads

clean-Core-2f-Inc-2f-rtos-2f-threads:
	-$(RM) ./Core/Inc/rtos/threads/am_threads.cyclo ./Core/Inc/rtos/threads/am_threads.d ./Core/Inc/rtos/threads/am_threads.o ./Core/Inc/rtos/threads/am_threads.su ./Core/Inc/rtos/threads/sm_threads.cyclo ./Core/Inc/rtos/threads/sm_threads.d ./Core/Inc/rtos/threads/sm_threads.o ./Core/Inc/rtos/threads/sm_threads.su ./Core/Inc/rtos/threads/unified_threads.cyclo ./Core/Inc/rtos/threads/unified_threads.d ./Core/Inc/rtos/threads/unified_threads.o ./Core/Inc/rtos/threads/unified_threads.su

.PHONY: clean-Core-2f-Inc-2f-rtos-2f-threads

