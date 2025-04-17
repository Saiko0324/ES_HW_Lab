################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/B-L475E-IOT01A1/b_l475e_iot01a1.c \
../Drivers/BSP/B-L475E-IOT01A1/b_l475e_iot01a1_bus.c \
../Drivers/BSP/B-L475E-IOT01A1/stm32l475e_iot01_accelero.c 

OBJS += \
./Drivers/BSP/B-L475E-IOT01A1/b_l475e_iot01a1.o \
./Drivers/BSP/B-L475E-IOT01A1/b_l475e_iot01a1_bus.o \
./Drivers/BSP/B-L475E-IOT01A1/stm32l475e_iot01_accelero.o 

C_DEPS += \
./Drivers/BSP/B-L475E-IOT01A1/b_l475e_iot01a1.d \
./Drivers/BSP/B-L475E-IOT01A1/b_l475e_iot01a1_bus.d \
./Drivers/BSP/B-L475E-IOT01A1/stm32l475e_iot01_accelero.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/B-L475E-IOT01A1/%.o Drivers/BSP/B-L475E-IOT01A1/%.su Drivers/BSP/B-L475E-IOT01A1/%.cyclo: ../Drivers/BSP/B-L475E-IOT01A1/%.c Drivers/BSP/B-L475E-IOT01A1/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../BlueNRG_MS/App -I../BlueNRG_MS/Target -I../Drivers/BSP/B-L475E-IOT01A1 -I../Middlewares/ST/BlueNRG-MS/utils -I../Middlewares/ST/BlueNRG-MS/includes -I../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../Drivers/BSP/Components/Common -I../Drivers/BSP/Components/lsm6dsl -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-B-2d-L475E-2d-IOT01A1

clean-Drivers-2f-BSP-2f-B-2d-L475E-2d-IOT01A1:
	-$(RM) ./Drivers/BSP/B-L475E-IOT01A1/b_l475e_iot01a1.cyclo ./Drivers/BSP/B-L475E-IOT01A1/b_l475e_iot01a1.d ./Drivers/BSP/B-L475E-IOT01A1/b_l475e_iot01a1.o ./Drivers/BSP/B-L475E-IOT01A1/b_l475e_iot01a1.su ./Drivers/BSP/B-L475E-IOT01A1/b_l475e_iot01a1_bus.cyclo ./Drivers/BSP/B-L475E-IOT01A1/b_l475e_iot01a1_bus.d ./Drivers/BSP/B-L475E-IOT01A1/b_l475e_iot01a1_bus.o ./Drivers/BSP/B-L475E-IOT01A1/b_l475e_iot01a1_bus.su ./Drivers/BSP/B-L475E-IOT01A1/stm32l475e_iot01_accelero.cyclo ./Drivers/BSP/B-L475E-IOT01A1/stm32l475e_iot01_accelero.d ./Drivers/BSP/B-L475E-IOT01A1/stm32l475e_iot01_accelero.o ./Drivers/BSP/B-L475E-IOT01A1/stm32l475e_iot01_accelero.su

.PHONY: clean-Drivers-2f-BSP-2f-B-2d-L475E-2d-IOT01A1

