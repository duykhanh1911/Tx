################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/DSP/Source/CommonTables/arm_common_tables.c \
D:/DSP/Source/CommonTables/arm_common_tables_f16.c \
D:/DSP/Source/CommonTables/arm_const_structs.c \
D:/DSP/Source/CommonTables/arm_const_structs_f16.c \
D:/DSP/Source/CommonTables/arm_mve_tables.c \
D:/DSP/Source/CommonTables/arm_mve_tables_f16.c 

OBJS += \
./DSP/Source/CommonTables/arm_common_tables.o \
./DSP/Source/CommonTables/arm_common_tables_f16.o \
./DSP/Source/CommonTables/arm_const_structs.o \
./DSP/Source/CommonTables/arm_const_structs_f16.o \
./DSP/Source/CommonTables/arm_mve_tables.o \
./DSP/Source/CommonTables/arm_mve_tables_f16.o 

C_DEPS += \
./DSP/Source/CommonTables/arm_common_tables.d \
./DSP/Source/CommonTables/arm_common_tables_f16.d \
./DSP/Source/CommonTables/arm_const_structs.d \
./DSP/Source/CommonTables/arm_const_structs_f16.d \
./DSP/Source/CommonTables/arm_mve_tables.d \
./DSP/Source/CommonTables/arm_mve_tables_f16.d 


# Each subdirectory must supply rules for building sources it contributes
DSP/Source/CommonTables/arm_common_tables.o: D:/DSP/Source/CommonTables/arm_common_tables.c DSP/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/DSP/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
DSP/Source/CommonTables/arm_common_tables_f16.o: D:/DSP/Source/CommonTables/arm_common_tables_f16.c DSP/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/DSP/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
DSP/Source/CommonTables/arm_const_structs.o: D:/DSP/Source/CommonTables/arm_const_structs.c DSP/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/DSP/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
DSP/Source/CommonTables/arm_const_structs_f16.o: D:/DSP/Source/CommonTables/arm_const_structs_f16.c DSP/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/DSP/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
DSP/Source/CommonTables/arm_mve_tables.o: D:/DSP/Source/CommonTables/arm_mve_tables.c DSP/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/DSP/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
DSP/Source/CommonTables/arm_mve_tables_f16.o: D:/DSP/Source/CommonTables/arm_mve_tables_f16.c DSP/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/DSP/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DSP-2f-Source-2f-CommonTables

clean-DSP-2f-Source-2f-CommonTables:
	-$(RM) ./DSP/Source/CommonTables/arm_common_tables.cyclo ./DSP/Source/CommonTables/arm_common_tables.d ./DSP/Source/CommonTables/arm_common_tables.o ./DSP/Source/CommonTables/arm_common_tables.su ./DSP/Source/CommonTables/arm_common_tables_f16.cyclo ./DSP/Source/CommonTables/arm_common_tables_f16.d ./DSP/Source/CommonTables/arm_common_tables_f16.o ./DSP/Source/CommonTables/arm_common_tables_f16.su ./DSP/Source/CommonTables/arm_const_structs.cyclo ./DSP/Source/CommonTables/arm_const_structs.d ./DSP/Source/CommonTables/arm_const_structs.o ./DSP/Source/CommonTables/arm_const_structs.su ./DSP/Source/CommonTables/arm_const_structs_f16.cyclo ./DSP/Source/CommonTables/arm_const_structs_f16.d ./DSP/Source/CommonTables/arm_const_structs_f16.o ./DSP/Source/CommonTables/arm_const_structs_f16.su ./DSP/Source/CommonTables/arm_mve_tables.cyclo ./DSP/Source/CommonTables/arm_mve_tables.d ./DSP/Source/CommonTables/arm_mve_tables.o ./DSP/Source/CommonTables/arm_mve_tables.su ./DSP/Source/CommonTables/arm_mve_tables_f16.cyclo ./DSP/Source/CommonTables/arm_mve_tables_f16.d ./DSP/Source/CommonTables/arm_mve_tables_f16.o ./DSP/Source/CommonTables/arm_mve_tables_f16.su

.PHONY: clean-DSP-2f-Source-2f-CommonTables

