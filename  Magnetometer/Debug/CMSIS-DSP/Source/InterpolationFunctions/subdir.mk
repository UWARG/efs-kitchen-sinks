################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_f16.c \
../CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_f32.c \
../CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q15.c \
../CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q31.c \
../CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q7.c \
../CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_f16.c \
../CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_f32.c \
../CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q15.c \
../CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q31.c \
../CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q7.c \
../CMSIS-DSP/Source/InterpolationFunctions/arm_spline_interp_f32.c \
../CMSIS-DSP/Source/InterpolationFunctions/arm_spline_interp_init_f32.c 

C_DEPS += \
./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_f16.d \
./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_f32.d \
./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q15.d \
./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q31.d \
./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q7.d \
./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_f16.d \
./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_f32.d \
./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q15.d \
./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q31.d \
./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q7.d \
./CMSIS-DSP/Source/InterpolationFunctions/arm_spline_interp_f32.d \
./CMSIS-DSP/Source/InterpolationFunctions/arm_spline_interp_init_f32.d 

OBJS += \
./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_f16.o \
./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_f32.o \
./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q15.o \
./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q31.o \
./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q7.o \
./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_f16.o \
./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_f32.o \
./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q15.o \
./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q31.o \
./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q7.o \
./CMSIS-DSP/Source/InterpolationFunctions/arm_spline_interp_f32.o \
./CMSIS-DSP/Source/InterpolationFunctions/arm_spline_interp_init_f32.o 


# Each subdirectory must supply rules for building sources it contributes
CMSIS-DSP/Source/InterpolationFunctions/%.o CMSIS-DSP/Source/InterpolationFunctions/%.su CMSIS-DSP/Source/InterpolationFunctions/%.cyclo: ../CMSIS-DSP/Source/InterpolationFunctions/%.c CMSIS-DSP/Source/InterpolationFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../CMSIS-DSP/Include -I../CMSIS-DSP/PrivateInclude -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2d-DSP-2f-Source-2f-InterpolationFunctions

clean-CMSIS-2d-DSP-2f-Source-2f-InterpolationFunctions:
	-$(RM) ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_f16.cyclo ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_f16.d ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_f16.o ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_f16.su ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_f32.cyclo ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_f32.d ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_f32.o ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_f32.su ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q15.cyclo ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q15.d ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q15.o ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q15.su ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q31.cyclo ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q31.d ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q31.o ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q31.su ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q7.cyclo ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q7.d ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q7.o ./CMSIS-DSP/Source/InterpolationFunctions/arm_bilinear_interp_q7.su ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_f16.cyclo ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_f16.d ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_f16.o ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_f16.su ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_f32.cyclo ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_f32.d ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_f32.o ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_f32.su ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q15.cyclo ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q15.d ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q15.o ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q15.su ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q31.cyclo ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q31.d ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q31.o ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q31.su ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q7.cyclo ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q7.d ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q7.o ./CMSIS-DSP/Source/InterpolationFunctions/arm_linear_interp_q7.su ./CMSIS-DSP/Source/InterpolationFunctions/arm_spline_interp_f32.cyclo ./CMSIS-DSP/Source/InterpolationFunctions/arm_spline_interp_f32.d ./CMSIS-DSP/Source/InterpolationFunctions/arm_spline_interp_f32.o ./CMSIS-DSP/Source/InterpolationFunctions/arm_spline_interp_f32.su ./CMSIS-DSP/Source/InterpolationFunctions/arm_spline_interp_init_f32.cyclo ./CMSIS-DSP/Source/InterpolationFunctions/arm_spline_interp_init_f32.d ./CMSIS-DSP/Source/InterpolationFunctions/arm_spline_interp_init_f32.o ./CMSIS-DSP/Source/InterpolationFunctions/arm_spline_interp_init_f32.su

.PHONY: clean-CMSIS-2d-DSP-2f-Source-2f-InterpolationFunctions

