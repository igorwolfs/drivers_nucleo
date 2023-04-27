
#include "stm32f446xx_gpio_driver.h"


/*
 * Peripheral Clock Setup
*/

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pGPIOx==GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx==GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx==GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx==GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx==GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx==GPIOF){
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx==GPIOG){
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx==GPIOH){
			GPIOH_PCLK_EN();
		}
	}
	else if (EnorDi == DISABLE) {
		if (pGPIOx==GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx==GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx==GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx==GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx==GPIOE){
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx==GPIOF){
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx==GPIOG){
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx==GPIOH){
			GPIOH_PCLK_DI();
		}
	}
}



/* Initialization / deinitialization */

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initialises the GPIO port and pin
 *
 * @param[in]         - Handle containing the base address and configuration
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t temp;
	// 1. Configure mode of pin (interrupt vs no-interrupt)
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// Get value of pin in mode register
		// set it to right value
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		// Clear 2 bits (11 = 0x3) shifted to the right position
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		// BITWISE OR: Only change the fields that matter
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else {
		// Configure the type of interrupt use EXTI register
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			// First 41 lines are on the FTSR
			EXTI->FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			EXTI->FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// Configure GPIO port selection in SYSCFG EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		// pinA, pinB, pinC, pinD, ..
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] =	(portcode << (temp2 * 4));

		// Enable EXTI interrupt delivery using IMR
		EXTI->IMR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// 2. Configure speed
	// Left shift by 2 per pin number since the register has 2 values.
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	// 3. Configure pupd
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	// 4. Configure OPtype
	// Leftshift by 1 per pin Number
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	// 5. Configure alt functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		 // configure alternate functionality
		 // Check the functionalities available with respect to the AF and the pin number


		// 0/1 (which special register)
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		// 0-7 which subset of the register
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
	}
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function de-initialises the GPIO port and pin
 *
 * @param[in]         - Struct initialised at the base address
 *
 * @return            - none
 *
 * @Note              - simply sets the RCC reset register to reset the peripheral

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx==GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx==GPIOB){
		GPIOB_REG_RESET();
	}
	else if(pGPIOx==GPIOC){
		GPIOC_REG_RESET();
	}
	else if(pGPIOx==GPIOD){
		GPIOD_REG_RESET();
	}
	else if(pGPIOx==GPIOE){
		GPIOE_REG_RESET();
	}
	else if(pGPIOx==GPIOF){
		GPIOF_REG_RESET();
	}
	else if(pGPIOx==GPIOG){
		GPIOG_REG_RESET();
	}
	else if(pGPIOx==GPIOH){
		GPIOH_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function reads from the input pin
 *
 * @param[in]         - Struct initialised at the base address
 * @param[in]         - Pin number to read from
 *
 * @return            - 1 or 0
 *
 * @Note              - Reads the value from the input pin
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	uint8_t value;
	// Right shift with pin number, mask and cast
	value = (uint8_t) (pGPIOx->IDR >> PinNumber) & (0x00000001);
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function reads from the input pin
 *
 * @param[in]         - Struct initialised at the base address
 *
 * @return            - current IDR register (16 bit number)
 *
 * @Note              - Reads entire IDR register for that port. So uint16 is used.
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value;
	// Right shift with pin number, mask and cast
	value = (uint16_t) (pGPIOx->IDR);
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function reads from the output pin
 *
 * @param[in]         - Struct initialised at the base address
 * @param[in]         - Pin number to write to
 * @param[in]		  - Value to write to pin
 *
 * @return            - none
 *
 * @Note              - Writes to 1 bit in ODR register
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {
	if (Value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	}
	if (Value == GPIO_PIN_RESET) {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function reads from the output port
 *
 * @param[in]         - Struct initialised at the base address
 * @param[in]		  - Value to write to pin
 *
 * @return            - current ODR register (16 bit number)
 *
 * @Note              - Writes entire ODR register for that port. So uint16 is used.
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = (Value);
}

/*Data reading / Writing*/
/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPort
 *
 * @brief             - This function writes to the output pin
 *
 * @param[in]         - Struct initialised at the base address
 * @param[in]         - Pin number to write to
 *
 * @return            - none
 *
 * @Note              - Changes output state of the GPIO pin
 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	// Change output to what it previously was not.
	// Use xor
	pGPIOx->ODR = ((pGPIOx->ODR) ^= (1<<PinNumber));
}

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	// NOTE: max 90 interrupts
	if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if ((IRQNumber>31) && (IRQNumber<64)) {
			*NVIC_ISER1 |= (1 << IRQNumber);
		}
		else if ((IRQNumber>=64 && (IRQNumber<96))) {
			*NVIC_ISER2 |= (1 << IRQNumber);
		}
	}
	else {
		if (IRQNumber <= 31) {
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if ((IRQNumber>31) && (IRQNumber<64)) {
			*NVIC_ICER1 |= (1 << IRQNumber);
		}
		else if ((IRQNumber>=64 && (IRQNumber<96))) {
			*NVIC_ICER2 |= (1 << IRQNumber);
		}
	}
}


void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	// Interrupt priority register:
	// Only 4 bits are implemented for priority
	// So make sure you have to skip the initial bits that aren^t implemented to modify the implemented ones.
	uint8_t shift_amount =(8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx*4) |= (IRQPriority << shift_amount);
}

// 2 pending register: EXTI and NVIC
// Interrupt service routine saved at address shown in vector table.
// Called by the IRQ handler which is application specific!
void GPIO_IRQHandler(uint8_t PinNumber) {
	// Clear the exti pr register corresponding to pin register
	if ((EXTI->PR1) & (1<<PinNumber)) {
		EXTI->PR1 |= (1<<PinNumber); // Selected trigger request occurred
	}
}
