#include "stm32f446xx_i2c_driver.h"

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}
}


void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0 ;

	//enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//ack control bit
	// ADAPT
	//tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
	//pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	//tempreg = 0;
	//tempreg |= RCC_GetPCLK1Value() /1000000U ;
	//pI2CHandle->pI2Cx->CR2 =  (tempreg & 0x3F);

    // Program the device own address
	// Mode is by default set to 7 bit address.
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_OA1;
	tempreg |= ( 1 << I2C_OAR1_OA1EN);
	pI2CHandle->pI2Cx->OAR1 = tempreg;


	//CCR calculations
	// REDO THEM FOR NUCLEO while debugging, different registers apply

}
