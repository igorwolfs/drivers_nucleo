#include "stm32f446xx_i2c_driver.h"


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR2 |= ( 1 << I2C_CR2_START);
}


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {
	if (pI2Cx->ISR & FlagName) {
		return FLAG_SET;
	}
	else {
		return FLAG_RESET;
	}
}

// Set slave address in data register
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->CR2 = SlaveAddr;
}


static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; //SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->CR2 = SlaveAddr;
}


void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
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
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_OA17bit;
	tempreg |= ( 1 << I2C_OAR1_OA1EN);
	pI2CHandle->pI2Cx->OAR1 = tempreg;


	//CCR calculations
	// REDO THEM FOR NUCLEO while debugging, different registers apply
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr) {

	// Program n of bytes to be sent
	pI2CHandle->pI2Cx->CR2 = (Len << I2C_CR2_NBYTES);

	// Slave address to send to
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);

	// Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

}
