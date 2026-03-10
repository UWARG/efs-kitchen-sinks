################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.c \
../CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.c \
../CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.c \
../CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.c \
../CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.c \
../CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_f32.c \
../CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.c \
../CMSIS-DSP/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.c 

C_DEPS += \
./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.d \
./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.d \
./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.d \
./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.d \
./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.d \
./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_f32.d \
./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.d \
./CMSIS-DSP/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.d 

OBJS += \
./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.o \
./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.o \
./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.o \
./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.o \
./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.o \
./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_f32.o \
./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.o \
./CMSIS-DSP/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.o 


# Each subdirectory must supply rules for building sources it contributes
CMSIS-DSP/Source/QuaternionMathFunctions/%.o CMSIS-DSP/Source/QuaternionMathFunctions/%.su CMSIS-DSP/Source/QuaternionMathFunctions/%.cyclo: ../CMSIS-DSP/Source/QuaternionMathFunctions/%.c CMSIS-DSP/Source/QuaternionMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../CMSIS-DSP/Include -I../CMSIS-DSP/PrivateInclude -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2d-DSP-2f-Source-2f-QuaternionMathFunctions

clean-CMSIS-2d-DSP-2f-Source-2f-QuaternionMathFunctions:
	-$(RM) ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.cyclo ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.d ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.o ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion2rotation_f32.su ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.cyclo ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.d ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.o ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_conjugate_f32.su ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.cyclo ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.d ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.o ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_inverse_f32.su ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.cyclo ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.d ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.o ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_norm_f32.su ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.cyclo ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.d ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.o ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_normalize_f32.su ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_f32.cyclo ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_f32.d ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_f32.o ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_f32.su ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.cyclo ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.d ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.o ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_quaternion_product_single_f32.su ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.cyclo ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.d ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.o ./CMSIS-DSP/Source/QuaternionMathFunctions/arm_rotation2quaternion_f32.su

.PHONY: clean-CMSIS-2d-DSP-2f-Source-2f-QuaternionMathFunctions

