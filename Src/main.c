
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"

void delay(void) {
	for (int i=0; i< 5000000; i++) {
		;
	}
}


// Initialize SPI to the pins that you can find

int main() {
	// NOTE: PC13 is the button
	// PB13


	GPIO_Handle_t GPIOled;
	GPIOled.GPIO_PinConfig.GPIO_PinNumber = 13;
	GPIOled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	GPIOled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GPIOled);

	while (1) {
		GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_13);
		delay();
	}

    return 0;
}
