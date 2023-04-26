#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DIRVER_H_
#include "stm32f446xx.h"

typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;			//!< possible values from @GPIO_PIN_MODES >
	uint8_t GPIO_PinSpeed;			//!< possible values from @GPIO_PIN_SPEED >
	uint8_t GPIO_PinPuPdControl;	//!< possible values from @GPIO_PIN_PU/PD control >
	uint8_t GPIO_PinOPType;			//!< possible values from @GPIO_PINOPType >
	uint8_t GPIO_PinAltFunMode;		//!< possible values from @GPIO_PINAltFunMode >
} GPIO_PinConfig_t;


typedef struct {
	// NOTE: convention p (ptr)
	GPIO_RegDef_t *pGPIOx; // holds base address of GPIO port (A-H)
	GPIO_PinConfig_t GPIO_PinConfig; // Holds base address of GPIO port (A-H)
} GPIO_Handle_t;


// GPIO_BASEADDR_TO_CODE

#define GPIO_BASEADDR_TO_CODE(x)	(x == GPIOA) ? 0:  \
									(x == GPIOB) ? 1:  \
									(x == GPIOC) ? 2:  \
									(x == GPIOD) ? 3:  \
									(x == GPIOE) ? 4:  \
									(x == GPIOF) ? 5:  \
									(x == GPIOG) ? 6:  \
									(x == GPIOH) ? 7:0 \

// @GPIO_PIN_NUMBERS
// GPIO pin numbers

#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15

// GPIO possible modes
#define GPIO_MODE_IN 		0 	// Input
#define GPIO_MODE_OUT 		1	// Output
#define GPIO_MODE_ALTFN 	2	// Alternate function
#define GPIO_MODE_ANALOG	3	// Analog
#define GPIO_MODE_IT_FT		4	// Falling edge interrupt
#define GPIO_MODE_IT_RT		5	// Rising edge interrupt
#define GPIO_MODE_IT_RFT	6	// Both falling and rising edge interrupt


// GPIO possible output types PP/OD
#define GPIO_OP_TYPE_PP		0 // Push pull no floating
#define GPIO_OP_TYPE_OD		1 // Open drain: floating when not pulled to ground

// GPIO possible speeds
#define GPIO_SPEED_LOW 		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIHG		3

// GPIO pull up down
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2


/******** Driver API's **********/

// Peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGpiox, uint8_t EnOrDi);

// Initialization / deinitialization
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGpiox); //Use the RCC reset register

// Data reading / Writing
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value); // 16 pins in one port

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandler(uint8_t PinNumber);


#endif
