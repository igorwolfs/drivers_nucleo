#ifndef INC_STM32F446_H_
#define INC_STM32F446_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))

/* Processor STM32L47x */

/*
 * ARM Cortex Mx Processor NVIC ISERx register address
 */
#define NVIC_ISER0			((__vo uint32_t*) 0xE000E100)
#define NVIC_ISER1			((__vo uint32_t*) 0xE000E104)
#define NVIC_ISER2			((__vo uint32_t*) 0xE000E108)
#define NVIC_ISER3			((__vo uint32_t*) 0xE000E10C)

#define NVIC_ICER0			((__vo uint32_t*) 0xE000E180)
#define NVIC_ICER1			((__vo uint32_t*) 0xE000E184)
#define NVIC_ICER2			((__vo uint32_t*) 0xE000E188)
#define NVIC_ICER3			((__vo uint32_t*) 0xE000E18C)

#define NVIC_PR_BASE_ADDR	((__vo uint32_t*) 0xE000E400)

#define NO_PR_BITS_IMPLEMENTED	4

/* Memory Macro's*/
// Addresses by default signed so type cast them to unsigned
#define FLASH_BASEADDR		0x00000000U
#define SRAM1_BASEADDR		0x20000000U
#define SRAM2_BASEADDR		0x10000000U
#define ROM_BASEADDR 		0x1FFF0000U // System Memory

/* Bus addresses */
#define PERIPH_BASEADDR				0x40000000U
#define AHB2PERIPH_BASEADDR 		0x48000000U
#define AHB1PERIPH_BASEADDR 		0x40020000U
#define APB2PERIPH_BASEADDR 		0x40010000U
#define APB1PERIPH_BASEADDR 		PERIPH_BASEADDR

/* AHB1 bus peripherals*/
#define RCC_BASEADDR		(AHB1PERIPH_BASEADDR+0x1000)


/* AHB2 bus peripherals */
#define GPIOA_BASEADDR		(AHB2PERIPH_BASEADDR+0x0000)
#define GPIOB_BASEADDR		(AHB2PERIPH_BASEADDR+0x0400)
#define GPIOC_BASEADDR		(AHB2PERIPH_BASEADDR+0x0800)
#define GPIOD_BASEADDR		(AHB2PERIPH_BASEADDR+0x0C00)
#define GPIOE_BASEADDR		(AHB2PERIPH_BASEADDR+0x1000)
#define GPIOF_BASEADDR		(AHB2PERIPH_BASEADDR+0x1400)
#define GPIOG_BASEADDR		(AHB2PERIPH_BASEADDR+0x1800)
#define GPIOH_BASEADDR		(AHB2PERIPH_BASEADDR+0x1C00)


/* APB1 bus peripherals */
#define TIM2_BASEADDR		(APB1PERIPH_BASEADDR+0x0000)
#define TIM3_BASEADDR 		(APB1PERIPH_BASEADDR+0x0400)
#define TIM4_BASEADDR 		(APB1PERIPH_BASEADDR+0x0800)
#define TIM5_BASEADDR		(APB1PERIPH_BASEADDR+0x0C00)
#define TIM6_BASEADDR		(APB1PERIPH_BASEADDR+0x1000)
#define TIM7_BASEADDR		(APB1PERIPH_BASEADDR+0x1400)
#define RTC_BASEADDR		(APB1PERIPH_BASEADDR+0x2800)
#define SPI2_BASEADDR		(APB1PERIPH_BASEADDR+0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASEADDR+0x3C00)
#define USART2_BASEADDR		(APB1PERIPH_BASEADDR+0x4400) // Synchronous
#define USART3_BASEADDR		(APB1PERIPH_BASEADDR+0x4800) // Synchronous
#define UART4_BASEADDR		(APB1PERIPH_BASEADDR+0x4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASEADDR+0x5000)
#define I2C1_BASEADDR		(APB1PERIPH_BASEADDR+0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASEADDR+0x5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASEADDR+0x5C00)


/* APB2 bus peripherals*/
#define TIM1_BASEADDR		(APB2PERIPH_BASEADDR+0x2C00)
#define SPI1_BASEADDR		(APB2PERIPH_BASEADDR+0x3000)
#define	TIM8_BASEADDR		(APB2PERIPH_BASEADDR+0x3400)
#define USART1_BASEADDR		(APB2PERIPH_BASEADDR+0x3800)
#define TIM15_BASEADDR		(APB2PERIPH_BASEADDR+0x4000)
#define TIM16_BASEADDR		(APB2PERIPH_BASEADDR+0x4400)
#define TIM17_BASEADDR		(APB2PERIPH_BASEADDR+0x4800)
#define EXTI_BASEADDR		(APB2PERIPH_BASEADDR+0x0400)
#define SYSCFG_BASEADDR		(APB2PERIPH_BASEADDR+0x0000)

/* ------------------	Peripheral register definition structures -------------- */
/*	NOTE:
 * 	Every register is 32 bits (4 bytes)
 * 	Initialise the register by initialising pointer at Base address region of respective GPIO
 * */

// GPIO
typedef struct {
	__vo uint32_t MODER; 	/*GPIO port mode register*/
	__vo uint32_t OTYPER;	/*GPIO port output type register*/
	__vo uint32_t OSPEEDR; 	/*GPIO port output speed register */
	__vo uint32_t PUPDR;	/*GPIO port pull-up/pull-down register*/
	__vo uint32_t IDR;		/*GPIO port input data register*/
	__vo uint32_t ODR;		/*GPIO port output data register*/
	__vo uint32_t BSRR;		/*GPIO port bit set/reset register*/
	__vo uint32_t LCKR;		/*GPIO port configuration lock register*/
	__vo uint32_t AFR[2]; 	/*GPIO port low (1) and high (2) alternate function register*/
	__vo uint32_t BRR;		/*GPIO port bit reset register*/
	__vo uint32_t ASCR;		/*GPIO port analog switch control register*/
} GPIO_RegDef_t;


// RCC
typedef struct {
	__vo uint32_t CR; 				/*Clock control register*/
	__vo uint32_t ICSR;				/*Internal clock sources calibration register*/
	__vo uint32_t CFGR; 			/*Clock configuration register */
	__vo uint32_t PLLCFGR;			/*PLL configuration register */
	__vo uint32_t PLLSAICFGR[2];	/*PLLSAI1/PLLsA2 ocnfiguration registers*/
	__vo uint32_t CIER;				/*Clock interrupt enable register*/
	__vo uint32_t CIFR;				/*Clock interrupt flag register*/
	__vo uint32_t CICR;				/*Clock interrupt clear register*/
	__vo uint32_t AHB1RSTR; 		/*AHB1 peripheral reset register*/
	__vo uint32_t AHB2RSTR; 		/*AHB2 peripheral reset register*/
	__vo uint32_t AHB3RSTR; 		/*AHB3 peripheral reset register */
	__vo uint32_t APB1RSTR[2];		/*APB1 peripheral reset register 1 and 2*/
	__vo uint32_t APB2RSTR;			/*APB2 peripheral reset register */
	__vo uint32_t AHB1ENR;			/*AHB1 peripheral clock enable register */
	__vo uint32_t AHB2ENR;			/*AHB2 peripheral clock enable register */
	__vo uint32_t AHB3ENR;			/*AHB3 peripheral clock enable register */
	__vo uint32_t APB1ENR[2];		/*APB1 peripheral clock enable register 1 and 2*/
	__vo uint32_t APB2ENR;			/*APB2 peripheral clock enable register*/
	__vo uint32_t AHB1SMENR;		/*AHB1 peripheral clocks enable in Sleep and Stop modes register*/
	__vo uint32_t AHB2SMENR;		/*AHB2 peripheral clocks enable in Sleep and Stop modes register*/
	__vo uint32_t AHB3SMENR;		/*AHB3 peripheral clocks enable in Sleep and Stop modes register*/
	__vo uint32_t APB1SMENR1[2];	/*APB1 peripheral clocks enable in Sleep and Stop modes registers 1, 2*/
	__vo uint32_t APB2SMENR;		/*APB2 peripheral clocks enable in Sleep and Stop modes registers 1*/
	__vo uint32_t CCIPR;			/*Peripherals independent clock configuration register*/
	__vo uint32_t BDCR;				/*Backup domain control register*/
	__vo uint32_t CSR;				/*Control/status register*/
	__vo uint32_t CRRCR;			/*Clock recovery RC register*/
	__vo uint32_t CCIPR2;			/*Peripherals independent clock configuration register */
} RCC_RegDef_t;


// EXTI
typedef struct {
	__vo uint32_t IMR1; 	/*GPIO port mode register*/
	__vo uint32_t EMR1;	/*GPIO port output type register*/
	__vo uint32_t RTSR1; 	/*GPIO port output speed register */
	__vo uint32_t FTSR1;	/*GPIO port pull-up/pull-down register*/
	__vo uint32_t SWIER1;		/*GPIO port input data register*/
	__vo uint32_t PR1;		/*GPIO port output data register*/
	__vo uint32_t IMR2;		/*GPIO port bit set/reset register*/
	__vo uint32_t EMR2;		/*GPIO port configuration lock register*/
	__vo uint32_t RTSR2; 	/*GPIO port low (1) and high (2) alternate function register*/
	__vo uint32_t FTSR2;		/*GPIO port bit reset register*/
	__vo uint32_t SWIER2;		/*GPIO port analog switch control register*/
	__vo uint32_t PR2;		/*GPIO port analog switch control register*/
} EXTI_RegDef_t;


/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;       /*	*/
	__vo uint32_t CFGR1;        /*	*/
	__vo uint32_t EXTICR[4];    /* Decides on the port that takes over the interrupt line. (15 lines)	*/
	__vo uint32_t SCSR;  		/*	*/
	__vo uint32_t CFGR2;        /*  */
	__vo uint32_t SWPR;  		/*  */
	__vo uint32_t SKR;         	/*	*/
	__vo uint32_t SWPR2;        /*	*/
} SYSCFG_RegDef_t;


/*
 * peripheral register definition structure for SPI
 */
typedef struct {
	__vo uint32_t CR1;       /*	*/
	__vo uint32_t CR2;       /*	*/
	__vo uint32_t SR;        /*	*/
	__vo uint32_t DR;    /* Decides on the port that takes over the interrupt line. (15 lines)	*/
	__vo uint32_t CRCPR;  		/*	*/
	__vo uint32_t RXCRCR;        /*  */
	__vo uint32_t TXCRCR;  		/*  */
} SPI_RegDef_t;


// Peripheral register definition structure for I2C
typedef struct {
  __vo uint32_t CR1;
  __vo uint32_t CR2;
  __vo uint32_t OAR1;
  __vo uint32_t OAR2;
  __vo uint32_t TIMINGR;
  __vo uint32_t TIMEOUTR;
  __vo uint32_t ISR;
  __vo uint32_t ICR;
  __vo uint32_t PECR;
  __vo uint32_t RXDR;
}I2C_RegDef_t;

/* ---------------- peripheral definitions ---------------- */
#define GPIOA 		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 		((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC			((RCC_RegDef_t*) 	RCC_BASEADDR)

#define EXTI 		((EXTI_RegDef_t*)	EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1		((SPI_RegDef_t*) 	SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*) 	SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*) 	SPI3_BASEADDR)

#define I2C1		((I2C_RegDef_t*) 	I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*) 	I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t*) 	I2C3_BASEADDR)

/* ---------- RCC MACRO's ---------- */
/****** Clock enable MACRO's *******/
/* Perform enable on bit by | on that bit*/
// GPIOx
#define GPIOA_PCLK_EN()		(RCC->AHB2ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB2ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB2ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB2ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB2ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB2ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB2ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB2ENR |= (1 << 7))


// I2C1 enable
#define I2C1_PCLK_EN()	(RCC->APB1ENR[0] |= (1 << 21))
#define I2C2_PCLK_EN()	(RCC->APB1ENR[0] |= (1 << 22))
#define I2C3_PCLK_EN()	(RCC->APB1ENR[0] |= (1 << 23))
#define I2C4_PCLK_EN()	(RCC->APB1ENR[1] |= (1 << 1))


// SPI enable
#define SPI1_PCLK_EN()	(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()	(RCC->APB1ENR[0] |= (1 << 14))
#define SPI3_PCLK_EN()	(RCC->APB1ENR[0] |= (1 << 15))


// UART/USART enable
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()	(RCC->APB1ENR[0] |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR[0] |= (1 << 18))
#define UART4_PCLK_EN()		(RCC->APB1ENR[0] |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR[0] |= (1 << 20))


// SysConfig enable
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 0))


/****** Clock Disable MACRO's ******/
/* Perform clear on bit by & on all except for that bit*/
// GPIOx
#define GPIOA_PCLK_DI()	(RCC->AHB2ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()	(RCC->AHB2ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()	(RCC->AHB2ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()	(RCC->AHB2ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()	(RCC->AHB2ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()	(RCC->AHB2ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()	(RCC->AHB2ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()	(RCC->AHB2ENR &= ~(1 << 7))

// I2C1 Disable
#define I2C1_PCLK_DI()	(RCC->APB1ENR[0] &= ~(1 << 21))
#define I2C2_PCLK_DI()	(RCC->APB1ENR[0] &= ~(1 << 22))
#define I2C3_PCLK_DI()	(RCC->APB1ENR[0] &= ~(1 << 23))
#define I2C4_PCLK_DI()	(RCC->APB1ENR[1] &= ~(1 << 1))


// SPI Disable
#define SPI1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()	(RCC->APB1ENR[0] &= ~(1 << 14))
#define SPI3_PCLK_DI()	(RCC->APB1ENR[0] &= ~(1 << 15))

// UART/USART Disable
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()	(RCC->APB1ENR[0] &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR[0] &= ~(1 << 18))
#define UART4_PCLK_DI()		(RCC->APB1ENR[0] &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1ENR[0] &= ~(1 << 20))

// SysConfig Disable
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 0))

/******** RESET MACRO's ********/
// GPIO Set and reset: so first set the bit to 1, then set to 0

#define GPIOA_REG_RESET()		do {(RCC->AHB2RSTR|=(1 << 0));	((RCC->AHB2RSTR) &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()		do {(RCC->AHB2RSTR|=(1 << 1));	((RCC->AHB2RSTR) &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()		do {(RCC->AHB2RSTR|=(1 << 2));	((RCC->AHB2RSTR) &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()		do {(RCC->AHB2RSTR|=(1 << 3));	((RCC->AHB2RSTR) &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()		do {(RCC->AHB2RSTR|=(1 << 4));	((RCC->AHB2RSTR) &= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET()		do {(RCC->AHB2RSTR|=(1 << 5));	((RCC->AHB2RSTR) &= ~(1 << 5));} while(0)
#define GPIOG_REG_RESET()		do {(RCC->AHB2RSTR|=(1 << 6));	((RCC->AHB2RSTR) &= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET()		do {(RCC->AHB2RSTR|=(1 << 7));	((RCC->AHB2RSTR) &= ~(1 << 7));} while(0)

// I2C Reset
#define I2C1_REG_RESET()		do {(RCC->APB1STR[0]|= (1 << 21));	((RCC->APB1STR[0]) &= ~(1 << 21));} while(0)
#define I2C2_REG_RESET()		do {(RCC->APB1STR[0]|= (1 << 22));	((RCC->APB1STR[0]) &= ~(1 << 21));} while(0)
#define I2C3_REG_RESET()		do {(RCC->APB1STR[0]|= (1 << 23));	((RCC->APB1STR[0]) &= ~(1 << 21));} while(0)
#define I2C4_REG_RESET()		do {(RCC->APB1STR[1]|= (1 << 1));	((RCC->APB1STR[1]) &= ~(1 << 21));} while(0)

// SPI Set and reset: so first set the bit to 1, then set to 0
#define SPI1_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 12)); ((RCC->APB2RSTR) &= (~( 1 << 12 )));} while(0)
#define SPI2_REG_RESET()	do {(RCC->APB1RSTR[0] |= (1 << 14)); (RCC->APB1RSTR[0] &= ~(1 << 14));} while(0)
#define SPI3_REG_RESET()	do {(RCC->APB1RSTR[0] |= (1 << 15)); (RCC->APB1RSTR[0] &= ~(1 << 15));} while(0)

/*GENERIC MACRO's*/
#define ENABLE 			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_SET		SET
#define FLAG_RESET		RESET

/* IRQ Numbers */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/******* Bit position definitions of SPI peripheral ********/
/******* BitFields configuration ********/
/*SPI	CR1*/
#define SPI_CR1_BIDIMODE 		15
#define SPI_CR1_BIDIOE 			14
#define SPI_CR1_CRCEN 			13
#define SPI_CR1_CRCNEXT 		12
#define SPI_CR1_CRCL 			11	// DFF
#define SPI_CR1_RXONLY 			10
#define SPI_CR1_SSM	 			9
#define SPI_CR1_SSI	 			8
#define SPI_CR1_LSBFIRST	 	7
#define SPI_CR1_SPE			 	6
#define SPI_CR1_BR			 	3
#define SPI_CR1_MSTR			2
#define SPI_CR1_CPOL			1
#define SPI_CR1_CPHA			0

/*SPI	CR2*/
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7

/*SPI	SR*/
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/******* Bit position definitions of I2C peripheral ********/
/******* BitFields configuration ********/

// Bit position definitions I2C_SR
#define I2C_ISR_TXE 					0 // Transmit data register empty
#define I2C_ISR_TXIS 					1 // Transmit interrupt status
#define I2C_ISR_RXNE 					2 // Receive data register not empty
#define I2C_ISR_ADDR 				 	3 // Address matched
#define I2C_ISR_NACKF 					4 // Not Acknowledge received flag
#define I2C_ISR_STOPF 					5 // Stop detection flag
#define I2C_ISR_TC	 					6 // Transfer Complete
#define I2C_ISR_TCR	 					7 // Transfer Complete Reload
#define I2C_ISR_BERR 					8 // Bus error
#define I2C_ISR_ARLO 					9 // Arbitration lost
#define I2C_ISR_OVR 					10 // Overrun / Underrun
#define I2C_ISR_PECERR 					11 // PEC Error in exception
#define I2C_ISR_TIMEOUT 				12 // timeout detection tlow flag
#define I2C_ISR_ALERT	 				13 // SMBus alert
#define I2C_ISR_BUSY	 				15 // SMBus alert

// Bit position definitions I2C_CR1
#define I2C_CR1_PE						0
#define I2C_CR1_TXIE					1
#define I2C_CR1_RXIE					2
#define I2C_CR1_ADDRIE					3
#define I2C_CR1_ERROE					7

// Bit position definitions I2C_CR2
#define I2C_CR2_START					13
#define I2C_CR2_STOP					14
#define I2C_CR2_NACK					15
#define I2C_CR2_RELOAD					24


// Bit position definitions I2C_TIMINGR
#define I2C_TIMINGR_SCLL				0 	// Low period (in master mode)
#define I2C_TIMINGR_SCLH				8	// High period (in master mode)
#define I2C_TIMINGR_SDADEL				16	// Data hold time (delay between SCL falling and SDA edge)
#define I2C_TIMINGR_SCLDEL				20	// Data setup time (Data setup time)
#define I2C_TIMINGR_PRESC				28	// Used to pre-scale the I2CCLK

// Bit position definitions in own address 1 OAR1
#define I2C_OAR1_OA1EN					15
#define I2C_OAR1_OA17bit				1
#define I2C_OAR1_OA1MODE				10

// Bit position definitions in own address 1 OAR2
#define I2C_OAR2_OA2EN					15
#define I2C_OAR2_OA2					1
#define I2C_OAR2_OA2MODE				10

/******* Bit position definitions of RCC peripheral ********/
/******* BitFields configuration ********/

// CFG Register
#define RCC_CFGR_SWS					2
#define RCC_CFGR_HPRE					4 // AHB Prescaler
#define RCC_CFGR_PPRE1					8 // APB1 Prescaler
#define RCC_CFGR_PPRE2					11 // APB2 Prescaler


#endif	/* INC_STM32F446_H_ */
