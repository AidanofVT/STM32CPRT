################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Lcd/lcd.c 

OBJS += \
./Core/Src/Lcd/lcd.o 

C_DEPS += \
./Core/Src/Lcd/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Lcd/%.o Core/Src/Lcd/%.su: ../Core/Src/Lcd/%.c Core/Src/Lcd/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F722xx -c -I../Core/Inc -I../Core/Inc/Display -I../Core/Inc/Adc -I../Core/Inc/Switches -I../Core/Inc/Hw -I../Core/Inc/Utilities -I../Core/Inc/Model -I../Core/Inc/Buzzer -I../Core/Inc/Eeprom -I../Core/Inc/Led -I../FATFS/Target -I../Core/Inc/Lcd -I../FATFS/App -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/Third_Party/FatFs/src -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/STM32F7xx_HAL_Driver/Inc -I/home/aidan/Documents/DesignCatapult_CubeIDE_Project/Core/Inc/Utilities -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Lcd

clean-Core-2f-Src-2f-Lcd:
	-$(RM) ./Core/Src/Lcd/lcd.d ./Core/Src/Lcd/lcd.o ./Core/Src/Lcd/lcd.su

.PHONY: clean-Core-2f-Src-2f-Lcd

