################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSIS-DSP/Source/TransformFunctions/arm_bitreversal.c \
../CMSIS-DSP/Source/TransformFunctions/arm_bitreversal2.c \
../CMSIS-DSP/Source/TransformFunctions/arm_bitreversal_f16.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_f16.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_f32.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_f64.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f16.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f32.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f64.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_q15.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_q31.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_q15.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_q31.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_f16.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_f32.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_f16.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_f32.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_q15.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_q31.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_q15.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_q31.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_f16.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_f32.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_f16.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_f32.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_q15.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_q31.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_q15.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_q31.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix8_f16.c \
../CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix8_f32.c \
../CMSIS-DSP/Source/TransformFunctions/arm_mfcc_f16.c \
../CMSIS-DSP/Source/TransformFunctions/arm_mfcc_f32.c \
../CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_f16.c \
../CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_f32.c \
../CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_q15.c \
../CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_q31.c \
../CMSIS-DSP/Source/TransformFunctions/arm_mfcc_q15.c \
../CMSIS-DSP/Source/TransformFunctions/arm_mfcc_q31.c \
../CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f16.c \
../CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f32.c \
../CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f64.c \
../CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f16.c \
../CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f32.c \
../CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f64.c \
../CMSIS-DSP/Source/TransformFunctions/arm_rfft_init_q15.c \
../CMSIS-DSP/Source/TransformFunctions/arm_rfft_init_q31.c \
../CMSIS-DSP/Source/TransformFunctions/arm_rfft_q15.c \
../CMSIS-DSP/Source/TransformFunctions/arm_rfft_q31.c \
../CMSIS-DSP/Source/TransformFunctions/arm_transform_buffer_sizes.c 

C_DEPS += \
./CMSIS-DSP/Source/TransformFunctions/arm_bitreversal.d \
./CMSIS-DSP/Source/TransformFunctions/arm_bitreversal2.d \
./CMSIS-DSP/Source/TransformFunctions/arm_bitreversal_f16.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_f16.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_f32.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_f64.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f16.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f32.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f64.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_q15.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_q31.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_q15.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_q31.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_f16.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_f32.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_f16.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_f32.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_q15.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_q31.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_q15.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_q31.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_f16.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_f32.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_f16.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_f32.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_q15.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_q31.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_q15.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_q31.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix8_f16.d \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix8_f32.d \
./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_f16.d \
./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_f32.d \
./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_f16.d \
./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_f32.d \
./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_q15.d \
./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_q31.d \
./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_q15.d \
./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_q31.d \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f16.d \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f32.d \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f64.d \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f16.d \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f32.d \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f64.d \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_init_q15.d \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_init_q31.d \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_q15.d \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_q31.d \
./CMSIS-DSP/Source/TransformFunctions/arm_transform_buffer_sizes.d 

OBJS += \
./CMSIS-DSP/Source/TransformFunctions/arm_bitreversal.o \
./CMSIS-DSP/Source/TransformFunctions/arm_bitreversal2.o \
./CMSIS-DSP/Source/TransformFunctions/arm_bitreversal_f16.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_f16.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_f32.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_f64.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f16.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f32.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f64.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_q15.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_q31.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_q15.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_q31.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_f16.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_f32.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_f16.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_f32.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_q15.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_q31.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_q15.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_q31.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_f16.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_f32.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_f16.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_f32.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_q15.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_q31.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_q15.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_q31.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix8_f16.o \
./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix8_f32.o \
./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_f16.o \
./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_f32.o \
./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_f16.o \
./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_f32.o \
./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_q15.o \
./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_q31.o \
./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_q15.o \
./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_q31.o \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f16.o \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f32.o \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f64.o \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f16.o \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f32.o \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f64.o \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_init_q15.o \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_init_q31.o \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_q15.o \
./CMSIS-DSP/Source/TransformFunctions/arm_rfft_q31.o \
./CMSIS-DSP/Source/TransformFunctions/arm_transform_buffer_sizes.o 


# Each subdirectory must supply rules for building sources it contributes
CMSIS-DSP/Source/TransformFunctions/%.o CMSIS-DSP/Source/TransformFunctions/%.su CMSIS-DSP/Source/TransformFunctions/%.cyclo: ../CMSIS-DSP/Source/TransformFunctions/%.c CMSIS-DSP/Source/TransformFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../CMSIS-DSP/Include -I../CMSIS-DSP/PrivateInclude -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2d-DSP-2f-Source-2f-TransformFunctions

clean-CMSIS-2d-DSP-2f-Source-2f-TransformFunctions:
	-$(RM) ./CMSIS-DSP/Source/TransformFunctions/arm_bitreversal.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_bitreversal.d ./CMSIS-DSP/Source/TransformFunctions/arm_bitreversal.o ./CMSIS-DSP/Source/TransformFunctions/arm_bitreversal.su ./CMSIS-DSP/Source/TransformFunctions/arm_bitreversal2.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_bitreversal2.d ./CMSIS-DSP/Source/TransformFunctions/arm_bitreversal2.o ./CMSIS-DSP/Source/TransformFunctions/arm_bitreversal2.su ./CMSIS-DSP/Source/TransformFunctions/arm_bitreversal_f16.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_bitreversal_f16.d ./CMSIS-DSP/Source/TransformFunctions/arm_bitreversal_f16.o ./CMSIS-DSP/Source/TransformFunctions/arm_bitreversal_f16.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_f16.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_f16.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_f16.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_f16.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_f32.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_f32.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_f32.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_f32.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_f64.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_f64.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_f64.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_f64.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f16.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f16.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f16.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f16.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f32.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f32.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f32.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f32.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f64.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f64.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f64.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f64.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_q15.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_q15.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_q15.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_q15.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_q31.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_q31.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_q31.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_q31.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_q15.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_q15.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_q15.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_q15.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_q31.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_q31.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_q31.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_q31.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_f16.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_f16.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_f16.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_f16.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_f32.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_f32.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_f32.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_f32.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_f16.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_f16.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_f16.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_f16.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_f32.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_f32.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_f32.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_f32.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_q15.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_q15.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_q15.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_q15.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_q31.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_q31.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_q31.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_init_q31.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_q15.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_q15.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_q15.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_q15.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_q31.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_q31.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_q31.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix2_q31.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_f16.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_f16.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_f16.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_f16.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_f32.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_f32.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_f32.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_f32.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_f16.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_f16.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_f16.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_f16.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_f32.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_f32.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_f32.o
	-$(RM) ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_f32.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_q15.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_q15.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_q15.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_q15.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_q31.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_q31.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_q31.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_init_q31.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_q15.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_q15.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_q15.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_q15.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_q31.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_q31.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_q31.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix4_q31.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix8_f16.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix8_f16.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix8_f16.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix8_f16.su ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix8_f32.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix8_f32.d ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix8_f32.o ./CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix8_f32.su ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_f16.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_f16.d ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_f16.o ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_f16.su ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_f32.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_f32.d ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_f32.o ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_f32.su ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_f16.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_f16.d ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_f16.o ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_f16.su ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_f32.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_f32.d ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_f32.o ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_f32.su ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_q15.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_q15.d ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_q15.o ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_q15.su ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_q31.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_q31.d ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_q31.o ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_init_q31.su ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_q15.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_q15.d ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_q15.o ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_q15.su ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_q31.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_q31.d ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_q31.o ./CMSIS-DSP/Source/TransformFunctions/arm_mfcc_q31.su ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f16.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f16.d ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f16.o ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f16.su ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f32.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f32.d ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f32.o ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f32.su ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f64.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f64.d ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f64.o ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f64.su ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f16.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f16.d ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f16.o ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f16.su ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f32.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f32.d ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f32.o ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f32.su ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f64.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f64.d ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f64.o ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f64.su ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_init_q15.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_init_q15.d ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_init_q15.o ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_init_q15.su ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_init_q31.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_init_q31.d ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_init_q31.o ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_init_q31.su ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_q15.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_q15.d ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_q15.o ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_q15.su ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_q31.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_q31.d ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_q31.o ./CMSIS-DSP/Source/TransformFunctions/arm_rfft_q31.su ./CMSIS-DSP/Source/TransformFunctions/arm_transform_buffer_sizes.cyclo ./CMSIS-DSP/Source/TransformFunctions/arm_transform_buffer_sizes.d ./CMSIS-DSP/Source/TransformFunctions/arm_transform_buffer_sizes.o
	-$(RM) ./CMSIS-DSP/Source/TransformFunctions/arm_transform_buffer_sizes.su

.PHONY: clean-CMSIS-2d-DSP-2f-Source-2f-TransformFunctions

