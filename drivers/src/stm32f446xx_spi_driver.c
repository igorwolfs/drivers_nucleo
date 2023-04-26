#include "stm32f446xx_spi_driver.h"


static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_ovr_err_handle(SPI_Handle_t *pHandle);

/******** Driver API's **********/
//--- Peripheral clock setup ---//

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {

	if(EnorDi == ENABLE) {
		if(pSPIx == SPI1) {
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
	}
	else {
		if(pSPIx == SPI1) {
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}
	}
}

//--- Initialization / deinitialization ---//
void SPI_Init(SPI_Handle_t *pSPIHandle) {

	uint32_t temp_cr1=0x0U;
	// Configure device mode
	temp_cr1 |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	// configure bus communication
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		// 2 line unidirectional
		temp_cr1 &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		// 1 line bidirectional
		temp_cr1 |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_DUPLEX_RXONLY) {
		temp_cr1 &= ~(1 << SPI_CR1_BIDIMODE);
		temp_cr1 |= (1 << SPI_CR1_RXONLY);
	}

	// Configure ClockSpeed
	temp_cr1 |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	// Configure DFF
	temp_cr1 |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_CRCL);

	// Configure CPOL
	temp_cr1 |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// Configure CPHA
	temp_cr1 |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	// Configure SSM
	temp_cr1 |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if(pSPIx == SPI1) {
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

// Data reading / Writing

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {
	while (len != 0) {
		while (SPI_GetFlagStatus(pSPIx, SPI_FLAG_TX));
		// IF 16 BIT DFF
		if ((pSPIx->CR1) & (1<<SPI_CR1_CRCL)) {
			// len: number of bytes
			// pSPI->DR register where to put bits to transmit
			// pTxBuffer: cast buffer to pass 16 bytes at a time
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			len--;
			len--;
			(uint16_t*)pTxBuffer ++;
		}
		// IF 8 BIT DFF
		else {
			pSPIx->DR = *((uint8_t*) pTxBuffer);
			len--;
			(uint8_t*)pTxBuffer ++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len) {
	while (len != 0) {
		while (SPI_GetFlagStatus(pSPIx, SPI_FLAG_RX));
		// IF 16 BIT DFF
		if ((pSPIx->CR1) & (1<<SPI_CR1_CRCL)) {
			// len: number of bytes
			// pSPI->DR register where to put bits to read
			// pTxBuffer: cast buffer to pass 16 bytes at a time
			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			len--;
			len--;
			(uint16_t*)pRxBuffer ++;
		}
		// IF 8 BIT DFF
		else {
			*((uint8_t*) pRxBuffer) = pSPIx->DR;
			len--;
			(uint8_t*)pRxBuffer ++;
		}
	}
}


/////// INTERRUPT ///////
void SPI_SendDataWithIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint8_t len) {
	// set length
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX) {
		pSPIHandle->TxLen = len;

		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		pSPIHandle->pTxBuffer = pTxBuffer;

		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
}

void SPI_ReceiveDataWithIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len) {
	// set length
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_RX) {
		pSPIHandle->RxLen = len;

		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		pSPIHandle->pRxBuffer = pRxBuffer;

		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

// IRQ Configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
// IRQPriority is uint32_t because the might be larger than 255
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);



static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle) {
	if ((pHandle->pSPIx->CR1) & (1<<SPI_CR1_CRCL)) {
		// len: number of bytes
		// pSPI->DR register where to put bits to read
		// pTxBuffer: cast buffer to pass 16 bytes at a time
		*((uint16_t*) pHandle->pRxBuffer) = pHandle->pSPIx->DR;
		pHandle->TxLen--;
		pHandle->TxLen--;
		(uint16_t*)pHandle->pRxBuffer ++;
	}
	// IF 8 BIT DFF
	else {
		*((uint8_t*) pHandle->pRxBuffer) = pHandle->pSPIx->DR;
		pHandle->TxLen--;
		(uint8_t*)pHandle->pRxBuffer ++;
	}
	if (pHandle->TxLen == 0) {
		// Stop the SPI communication
		SPI_CloseTransmission(pHandle);
		SPI_ApplicationEventCallback(pHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle) {
	if ((pHandle->pSPIx->CR1) & (1<<SPI_CR1_CRCL)) {
		// len: number of bytes
		// pSPI->DR register where to put bits to read
		// pTxBuffer: cast buffer to pass 16 bytes at a time
		*((uint16_t*)pHandle->pRxBuffer) = pHandle->pSPIx->DR;
		pHandle->RxLen--;
		pHandle->RxLen--;
		(uint16_t*)pHandle->pRxBuffer ++;
	}
	// IF 8 BIT DFF
	else {
		*((uint8_t*) pHandle->pRxBuffer) = pHandle->pSPIx->DR;
		pHandle->RxLen--;
		(uint8_t*)pHandle->pRxBuffer ++;
	}
	if (pHandle->RxState == 0) {
		SPI_CloseReception(pHandle);
		SPI_ApplicationEventCallback(pHandle, SPI_EVENT_RX_CMPLT);
	}
}

 // Overrun flag:
 // - Clearing OVR bit cleared through
 // 1. access of data register
 // 2. Read of SPI_SR register

static void spi_ovr_err_handle(SPI_Handle_t *pHandle) {
	// DON'T CLEAR THE OVER FLAG!
	// it might be that the spi is transmitting while you're reading
	if (pHandle->TxState != SPI_BUSY_IN_TX) {
		SPI_ClearOVRFlag(pHandle->pSPIx);
	}

	SPI_ApplicationEventCallback(pHandle, SPI_EVENT_OVR_ERR);

}


void SPI_IRQHandling(SPI_Handle_t *pHandle) {
	uint8_t temp1=0, temp2=0;
	temp1 = ((pHandle->pSPIx->SR) & (1 << SPI_SR_TXE));
	temp2 = ((pHandle->pSPIx->CR2) & (1 << SPI_CR2_TXEIE));

	if (temp1 & temp2) {
		spi_txe_interrupt_handle(pHandle);
	}

	temp1 = ((pHandle->pSPIx->SR) & (1 << SPI_SR_RXNE));
	temp2 = ((pHandle->pSPIx->CR2) & (1 << SPI_CR2_RXNEIE));

	if (temp1 & temp2) {
		spi_rxne_interrupt_handle(pHandle);
	}

	// Overflow error, data that is received is discarded
	temp1 = ((pHandle->pSPIx->SR) & (1 << SPI_SR_OVR));
	temp2 = ((pHandle->pSPIx->CR2) & (1 << SPI_CR2_ERRIE));
	if (temp1 & temp2) {
		spi_ovr_err_handle(pHandle);
	}
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){
	// Weak implementation
}
