################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Lib/efs-canard/canard_files/Src/canard.c 

C_DEPS += \
./Core/Lib/efs-canard/canard_files/Src/canard.d 

OBJS += \
./Core/Lib/efs-canard/canard_files/Src/canard.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/efs-canard/canard_files/Src/%.o Core/Lib/efs-canard/canard_files/Src/%.su Core/Lib/efs-canard/canard_files/Src/%.cyclo: ../Core/Lib/efs-canard/canard_files/Src/%.c Core/Lib/efs-canard/canard_files/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L433xx -c -I../Core/Inc -I"/home/jakob/Documents/warg/efs-kitchen-sinks/drone-can-gen/Core/Lib/efs-canard" -I"/home/jakob/Documents/warg/efs-kitchen-sinks/drone-can-gen/Core/Lib/efs-canard/canard_files/Inc" -I"/home/jakob/Documents/warg/efs-kitchen-sinks/drone-can-gen/Core/Lib/efs-canard/canard_files/dsdlc_generated/inc" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib-2f-efs-2d-canard-2f-canard_files-2f-Src

clean-Core-2f-Lib-2f-efs-2d-canard-2f-canard_files-2f-Src:
	-$(RM) ./Core/Lib/efs-canard/canard_files/Src/canard.cyclo ./Core/Lib/efs-canard/canard_files/Src/canard.d ./Core/Lib/efs-canard/canard_files/Src/canard.o ./Core/Lib/efs-canard/canard_files/Src/canard.su

.PHONY: clean-Core-2f-Lib-2f-efs-2d-canard-2f-canard_files-2f-Src

