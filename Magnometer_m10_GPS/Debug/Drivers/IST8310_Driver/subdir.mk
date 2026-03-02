################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Drivers/IST8310_Driver/ist8310_i2c.cpp 

OBJS += \
./Drivers/IST8310_Driver/ist8310_i2c.o 

CPP_DEPS += \
./Drivers/IST8310_Driver/ist8310_i2c.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/IST8310_Driver/%.o Drivers/IST8310_Driver/%.su Drivers/IST8310_Driver/%.cyclo: ../Drivers/IST8310_Driver/%.cpp Drivers/IST8310_Driver/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../Drivers/IST8310_Driver -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-IST8310_Driver

clean-Drivers-2f-IST8310_Driver:
	-$(RM) ./Drivers/IST8310_Driver/ist8310_i2c.cyclo ./Drivers/IST8310_Driver/ist8310_i2c.d ./Drivers/IST8310_Driver/ist8310_i2c.o ./Drivers/IST8310_Driver/ist8310_i2c.su

.PHONY: clean-Drivers-2f-IST8310_Driver

