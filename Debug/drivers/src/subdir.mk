################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stmf446xx_gpio_driver.c 

OBJS += \
./drivers/src/stmf446xx_gpio_driver.o 

C_DEPS += \
./drivers/src/stmf446xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o drivers/src/%.su drivers/src/%.cyclo: ../drivers/src/%.c drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -I"C:/Users/Igor Wolfs/Projects/stm32/github/drivers_nucleo/drivers/src" -I"C:/Users/Igor Wolfs/Projects/stm32/github/drivers_nucleo/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-src

clean-drivers-2f-src:
	-$(RM) ./drivers/src/stmf446xx_gpio_driver.cyclo ./drivers/src/stmf446xx_gpio_driver.d ./drivers/src/stmf446xx_gpio_driver.o ./drivers/src/stmf446xx_gpio_driver.su

.PHONY: clean-drivers-2f-src

