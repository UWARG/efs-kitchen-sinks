################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f16.c \
../CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_bitonic_sort_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_bubble_sort_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_copy_f16.c \
../CMSIS-DSP/Source/SupportFunctions/arm_copy_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_copy_f64.c \
../CMSIS-DSP/Source/SupportFunctions/arm_copy_q15.c \
../CMSIS-DSP/Source/SupportFunctions/arm_copy_q31.c \
../CMSIS-DSP/Source/SupportFunctions/arm_copy_q7.c \
../CMSIS-DSP/Source/SupportFunctions/arm_f16_to_f64.c \
../CMSIS-DSP/Source/SupportFunctions/arm_f16_to_float.c \
../CMSIS-DSP/Source/SupportFunctions/arm_f16_to_q15.c \
../CMSIS-DSP/Source/SupportFunctions/arm_f64_to_f16.c \
../CMSIS-DSP/Source/SupportFunctions/arm_f64_to_float.c \
../CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q15.c \
../CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q31.c \
../CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q7.c \
../CMSIS-DSP/Source/SupportFunctions/arm_fill_f16.c \
../CMSIS-DSP/Source/SupportFunctions/arm_fill_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_fill_f64.c \
../CMSIS-DSP/Source/SupportFunctions/arm_fill_q15.c \
../CMSIS-DSP/Source/SupportFunctions/arm_fill_q31.c \
../CMSIS-DSP/Source/SupportFunctions/arm_fill_q7.c \
../CMSIS-DSP/Source/SupportFunctions/arm_float_to_f16.c \
../CMSIS-DSP/Source/SupportFunctions/arm_float_to_f64.c \
../CMSIS-DSP/Source/SupportFunctions/arm_float_to_q15.c \
../CMSIS-DSP/Source/SupportFunctions/arm_float_to_q31.c \
../CMSIS-DSP/Source/SupportFunctions/arm_float_to_q7.c \
../CMSIS-DSP/Source/SupportFunctions/arm_heap_sort_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_insertion_sort_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_init_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f16.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f64.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q15_to_float.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q31.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q7.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q31_to_f64.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q31_to_float.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q15.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q7.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q7_to_f64.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q7_to_float.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q15.c \
../CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q31.c \
../CMSIS-DSP/Source/SupportFunctions/arm_quick_sort_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_selection_sort_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_sort_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_sort_init_f32.c \
../CMSIS-DSP/Source/SupportFunctions/arm_weighted_average_f16.c \
../CMSIS-DSP/Source/SupportFunctions/arm_weighted_average_f32.c 

C_DEPS += \
./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f16.d \
./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_bitonic_sort_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_bubble_sort_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_f16.d \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_f64.d \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_q15.d \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_q31.d \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_q7.d \
./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_f64.d \
./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_float.d \
./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_q15.d \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_f16.d \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_float.d \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q15.d \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q31.d \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q7.d \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_f16.d \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_f64.d \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_q15.d \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_q31.d \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_q7.d \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f16.d \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f64.d \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q15.d \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q31.d \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q7.d \
./CMSIS-DSP/Source/SupportFunctions/arm_heap_sort_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_insertion_sort_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_init_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f16.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f64.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_float.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q31.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q7.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_f64.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_float.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q15.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q7.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_f64.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_float.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q15.d \
./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q31.d \
./CMSIS-DSP/Source/SupportFunctions/arm_quick_sort_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_selection_sort_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_sort_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_sort_init_f32.d \
./CMSIS-DSP/Source/SupportFunctions/arm_weighted_average_f16.d \
./CMSIS-DSP/Source/SupportFunctions/arm_weighted_average_f32.d 

OBJS += \
./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f16.o \
./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_bitonic_sort_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_bubble_sort_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_f16.o \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_f64.o \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_q15.o \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_q31.o \
./CMSIS-DSP/Source/SupportFunctions/arm_copy_q7.o \
./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_f64.o \
./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_float.o \
./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_q15.o \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_f16.o \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_float.o \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q15.o \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q31.o \
./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q7.o \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_f16.o \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_f64.o \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_q15.o \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_q31.o \
./CMSIS-DSP/Source/SupportFunctions/arm_fill_q7.o \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f16.o \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f64.o \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q15.o \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q31.o \
./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q7.o \
./CMSIS-DSP/Source/SupportFunctions/arm_heap_sort_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_insertion_sort_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_init_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f16.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f64.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_float.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q31.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q7.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_f64.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_float.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q15.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q7.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_f64.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_float.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q15.o \
./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q31.o \
./CMSIS-DSP/Source/SupportFunctions/arm_quick_sort_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_selection_sort_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_sort_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_sort_init_f32.o \
./CMSIS-DSP/Source/SupportFunctions/arm_weighted_average_f16.o \
./CMSIS-DSP/Source/SupportFunctions/arm_weighted_average_f32.o 


# Each subdirectory must supply rules for building sources it contributes
CMSIS-DSP/Source/SupportFunctions/%.o CMSIS-DSP/Source/SupportFunctions/%.su CMSIS-DSP/Source/SupportFunctions/%.cyclo: ../CMSIS-DSP/Source/SupportFunctions/%.c CMSIS-DSP/Source/SupportFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../CMSIS-DSP/Include -I../CMSIS-DSP/PrivateInclude -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CMSIS-2d-DSP-2f-Source-2f-SupportFunctions

clean-CMSIS-2d-DSP-2f-Source-2f-SupportFunctions:
	-$(RM) ./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f16.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f16.d ./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f16.o ./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f16.su ./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_barycenter_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_bitonic_sort_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_bitonic_sort_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_bitonic_sort_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_bitonic_sort_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_bubble_sort_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_bubble_sort_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_bubble_sort_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_bubble_sort_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f16.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f16.d ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f16.o ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f16.su ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f64.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f64.d ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f64.o ./CMSIS-DSP/Source/SupportFunctions/arm_copy_f64.su ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q15.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q15.d ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q15.o ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q15.su ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q31.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q31.d ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q31.o ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q31.su ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q7.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q7.d ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q7.o ./CMSIS-DSP/Source/SupportFunctions/arm_copy_q7.su ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_f64.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_f64.d ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_f64.o ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_f64.su ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_float.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_float.d ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_float.o ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_float.su ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_q15.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_q15.d ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_q15.o ./CMSIS-DSP/Source/SupportFunctions/arm_f16_to_q15.su ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_f16.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_f16.d ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_f16.o ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_f16.su ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_float.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_float.d ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_float.o ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_float.su ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q15.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q15.d ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q15.o ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q15.su ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q31.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q31.d ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q31.o ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q31.su ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q7.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q7.d ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q7.o ./CMSIS-DSP/Source/SupportFunctions/arm_f64_to_q7.su ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f16.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f16.d ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f16.o ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f16.su ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f64.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f64.d ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f64.o ./CMSIS-DSP/Source/SupportFunctions/arm_fill_f64.su ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q15.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q15.d ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q15.o ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q15.su ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q31.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q31.d ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q31.o ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q31.su ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q7.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q7.d ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q7.o ./CMSIS-DSP/Source/SupportFunctions/arm_fill_q7.su ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f16.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f16.d ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f16.o ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f16.su ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f64.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f64.d ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f64.o ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_f64.su ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q15.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q15.d ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q15.o ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q15.su ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q31.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q31.d
	-$(RM) ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q31.o ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q31.su ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q7.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q7.d ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q7.o ./CMSIS-DSP/Source/SupportFunctions/arm_float_to_q7.su ./CMSIS-DSP/Source/SupportFunctions/arm_heap_sort_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_heap_sort_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_heap_sort_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_heap_sort_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_insertion_sort_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_insertion_sort_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_insertion_sort_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_insertion_sort_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_init_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_init_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_init_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_merge_sort_init_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f16.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f16.d ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f16.o ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f16.su ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f64.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f64.d ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f64.o ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_f64.su ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_float.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_float.d ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_float.o ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_float.su ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q31.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q31.d ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q31.o ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q31.su ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q7.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q7.d ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q7.o ./CMSIS-DSP/Source/SupportFunctions/arm_q15_to_q7.su ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_f64.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_f64.d ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_f64.o ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_f64.su ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_float.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_float.d ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_float.o ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_float.su ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q15.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q15.d ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q15.o ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q15.su ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q7.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q7.d ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q7.o ./CMSIS-DSP/Source/SupportFunctions/arm_q31_to_q7.su ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_f64.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_f64.d ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_f64.o ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_f64.su ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_float.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_float.d ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_float.o ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_float.su ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q15.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q15.d ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q15.o ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q15.su ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q31.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q31.d ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q31.o ./CMSIS-DSP/Source/SupportFunctions/arm_q7_to_q31.su ./CMSIS-DSP/Source/SupportFunctions/arm_quick_sort_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_quick_sort_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_quick_sort_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_quick_sort_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_selection_sort_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_selection_sort_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_selection_sort_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_selection_sort_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_sort_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_sort_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_sort_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_sort_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_sort_init_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_sort_init_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_sort_init_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_sort_init_f32.su ./CMSIS-DSP/Source/SupportFunctions/arm_weighted_average_f16.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_weighted_average_f16.d ./CMSIS-DSP/Source/SupportFunctions/arm_weighted_average_f16.o ./CMSIS-DSP/Source/SupportFunctions/arm_weighted_average_f16.su ./CMSIS-DSP/Source/SupportFunctions/arm_weighted_average_f32.cyclo ./CMSIS-DSP/Source/SupportFunctions/arm_weighted_average_f32.d ./CMSIS-DSP/Source/SupportFunctions/arm_weighted_average_f32.o ./CMSIS-DSP/Source/SupportFunctions/arm_weighted_average_f32.su

.PHONY: clean-CMSIS-2d-DSP-2f-Source-2f-SupportFunctions

