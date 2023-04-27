#include "stm32f446xx_rcc_driver.h"
uint32_t AHB_PreScaler[] = {2,4,8,16,64,128,256,512};
uint32_t APB1_PreScaler[] = {2,4,8,16};

uint32_t RCC_GetPCLK1Value(void) {
	uint32_t pclk1,SystemClk;
	uint8_t clksrc,temp,ahbp,apb1p;

	// Mask bit 0 and 1 to get clocksource
	clksrc = ((RCC->CFGR >> RCC_CFGR_SWS) & 0x3);

	// MSI
	if(clksrc == 0) {
		SystemClk = 48000000; // Reference manual t 60
	}
	// HSI16
	else if (clksrc == 1) {
		SystemClk = 16000000; // Reference manual t 59
	}
	//HSE
	else if (clksrc == 2) {
		SystemClk = 48000000; // Reference manual t 55
	}
	// PLL
	else if (clksrc == 3) {
		// define function to find PLL output clock speed
		SystemClk = RCC_GetPLLOutputClock();
	}

	// Find the value of the prescaler
	// Clock diagram fig 15: AHBPRESC and APB1PRESC
	// Get AHBPrescaler & mask first 4 bits (1111 = 0xF)
	// Register counts from 1000 - 1111
	temp = ((RCC->CFGR >> RCC_CFGR_HPRE) & 0xF);

	// Not divided
	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	//apb1
	// 0x7: 111, Register counts from 100 - 111
	temp = ((RCC->CFGR >> RCC_CFGR_PPRE1 ) & 0x7);

	if(temp < 4) {
		apb1p = 1;
	} else {
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 =  (SystemClk / ahbp) /apb1p;

	return pclk1;
}
