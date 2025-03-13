################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../stm_drivers/RCC.c \
../stm_drivers/stm32_F103C6_EXTI_driver.c \
../stm_drivers/stm32_F103C6_gpio_driver.c \
../stm_drivers/stm32f103x6_spi_driver.c \
../stm_drivers/stm32f103x8_i2c_driver.c \
../stm_drivers/uart.c 

OBJS += \
./stm_drivers/RCC.o \
./stm_drivers/stm32_F103C6_EXTI_driver.o \
./stm_drivers/stm32_F103C6_gpio_driver.o \
./stm_drivers/stm32f103x6_spi_driver.o \
./stm_drivers/stm32f103x8_i2c_driver.o \
./stm_drivers/uart.o 

C_DEPS += \
./stm_drivers/RCC.d \
./stm_drivers/stm32_F103C6_EXTI_driver.d \
./stm_drivers/stm32_F103C6_gpio_driver.d \
./stm_drivers/stm32f103x6_spi_driver.d \
./stm_drivers/stm32f103x8_i2c_driver.d \
./stm_drivers/uart.d 


# Each subdirectory must supply rules for building sources it contributes
stm_drivers/%.o stm_drivers/%.su stm_drivers/%.cyclo: ../stm_drivers/%.c stm_drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -g -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/target/AdaptiveCruiseControl/Core/Inc" -I"D:/DOCUMENTARY/EDU TUTORIALS/FastBit/Embedded-C/My-workspace/target/AdaptiveCruiseControl/stm_drivers/inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-stm_drivers

clean-stm_drivers:
	-$(RM) ./stm_drivers/RCC.cyclo ./stm_drivers/RCC.d ./stm_drivers/RCC.o ./stm_drivers/RCC.su ./stm_drivers/stm32_F103C6_EXTI_driver.cyclo ./stm_drivers/stm32_F103C6_EXTI_driver.d ./stm_drivers/stm32_F103C6_EXTI_driver.o ./stm_drivers/stm32_F103C6_EXTI_driver.su ./stm_drivers/stm32_F103C6_gpio_driver.cyclo ./stm_drivers/stm32_F103C6_gpio_driver.d ./stm_drivers/stm32_F103C6_gpio_driver.o ./stm_drivers/stm32_F103C6_gpio_driver.su ./stm_drivers/stm32f103x6_spi_driver.cyclo ./stm_drivers/stm32f103x6_spi_driver.d ./stm_drivers/stm32f103x6_spi_driver.o ./stm_drivers/stm32f103x6_spi_driver.su ./stm_drivers/stm32f103x8_i2c_driver.cyclo ./stm_drivers/stm32f103x8_i2c_driver.d ./stm_drivers/stm32f103x8_i2c_driver.o ./stm_drivers/stm32f103x8_i2c_driver.su ./stm_drivers/uart.cyclo ./stm_drivers/uart.d ./stm_drivers/uart.o ./stm_drivers/uart.su

.PHONY: clean-stm_drivers

