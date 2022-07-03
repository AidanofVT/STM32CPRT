################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Model/Model.c 

OBJS += \
./Core/Src/Model/Model.o 

C_DEPS += \
./Core/Src/Model/Model.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Model/%.o Core/Src/Model/%.su: ../Core/Src/Model/%.c Core/Src/Model/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -I../Core/Inc/Utilities -I../Core/Inc/Led -I../Core/Inc/Switches -I../Core/Inc/Display -I../Core/Inc/Model -I../Core/Inc/Buzzer -I../Core/Inc/Lcd -I../Core/Inc/Eeprom -I../Core/Inc/Adc -I../Core/Inc/Hw -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Model

clean-Core-2f-Src-2f-Model:
	-$(RM) ./Core/Src/Model/Model.d ./Core/Src/Model/Model.o ./Core/Src/Model/Model.su

.PHONY: clean-Core-2f-Src-2f-Model

