################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/DEV_Config.c \
../Core/Src/EPD_2in13_V4.c \
../Core/Src/EPD_2in13_V4_test.c \
../Core/Src/GUI_Paint.c \
../Core/Src/ImageData.c \
../Core/Src/ImageData2.c \
../Core/Src/bme280.c \
../Core/Src/device.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/pwrmode.c \
../Core/Src/rtc.c \
../Core/Src/spi.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/DEV_Config.o \
./Core/Src/EPD_2in13_V4.o \
./Core/Src/EPD_2in13_V4_test.o \
./Core/Src/GUI_Paint.o \
./Core/Src/ImageData.o \
./Core/Src/ImageData2.o \
./Core/Src/bme280.o \
./Core/Src/device.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/pwrmode.o \
./Core/Src/rtc.o \
./Core/Src/spi.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/DEV_Config.d \
./Core/Src/EPD_2in13_V4.d \
./Core/Src/EPD_2in13_V4_test.d \
./Core/Src/GUI_Paint.d \
./Core/Src/ImageData.d \
./Core/Src/ImageData2.d \
./Core/Src/bme280.d \
./Core/Src/device.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/pwrmode.d \
./Core/Src/rtc.d \
./Core/Src/spi.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/DEV_Config.cyclo ./Core/Src/DEV_Config.d ./Core/Src/DEV_Config.o ./Core/Src/DEV_Config.su ./Core/Src/EPD_2in13_V4.cyclo ./Core/Src/EPD_2in13_V4.d ./Core/Src/EPD_2in13_V4.o ./Core/Src/EPD_2in13_V4.su ./Core/Src/EPD_2in13_V4_test.cyclo ./Core/Src/EPD_2in13_V4_test.d ./Core/Src/EPD_2in13_V4_test.o ./Core/Src/EPD_2in13_V4_test.su ./Core/Src/GUI_Paint.cyclo ./Core/Src/GUI_Paint.d ./Core/Src/GUI_Paint.o ./Core/Src/GUI_Paint.su ./Core/Src/ImageData.cyclo ./Core/Src/ImageData.d ./Core/Src/ImageData.o ./Core/Src/ImageData.su ./Core/Src/ImageData2.cyclo ./Core/Src/ImageData2.d ./Core/Src/ImageData2.o ./Core/Src/ImageData2.su ./Core/Src/bme280.cyclo ./Core/Src/bme280.d ./Core/Src/bme280.o ./Core/Src/bme280.su ./Core/Src/device.cyclo ./Core/Src/device.d ./Core/Src/device.o ./Core/Src/device.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pwrmode.cyclo ./Core/Src/pwrmode.d ./Core/Src/pwrmode.o ./Core/Src/pwrmode.su ./Core/Src/rtc.cyclo ./Core/Src/rtc.d ./Core/Src/rtc.o ./Core/Src/rtc.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32l4xx_hal_msp.cyclo ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_it.cyclo ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.cyclo ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

