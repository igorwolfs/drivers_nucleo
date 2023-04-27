#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DIRVER_H_

#include "stm32f446xx.h"


////// SPI configuration structure //////
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;


/////  Handle structure for SPIx peripheral /////
typedef struct {
	SPI_RegDef_t 	*pSPIx;
	SPI_Config_t 	SPIConfig;
	uint8_t 		*pTxBuffer; // To store for interrupts
	uint8_t 		*pRxBuffer;
	uint32_t 		TxLen;
	uint32_t 		RxLen;
	uint8_t 		TxState;
	uint8_t 		RxState;
} SPI_Handle_t;


/******* SPI Configure MACRO's ********/
 /// SPI device modes
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0


/// POSSIBLE SPI APPLICATION EVENTS
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4

/// Busconfig
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_DUPLEX_RXONLY		3


/// Clock speed (check baud rate control fields)
#define SPI_SCLK_SPEED_DIV2             	0
#define SPI_SCLK_SPEED_DIV4             	1
#define SPI_SCLK_SPEED_DIV8             	2
#define SPI_SCLK_SPEED_DIV16             	3
#define SPI_SCLK_SPEED_DIV32             	4
#define SPI_SCLK_SPEED_DIV64             	5
#define SPI_SCLK_SPEED_DIV128             	6
#define SPI_SCLK_SPEED_DIV256             	7


/// SPI Application states
#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2


/// SPI_DFF (8 vs 16 bits data frame format)
#define SPI_DFF_8BITS 		0
#define SPI_DFF_16BITS  	1


/// CPOL
#define SPI_CPOL_HIGH 		1
#define SPI_CPOL_LOW 		0


/// CPHA
#define SPI_CPHA_HIGH 		1
#define SPI_CPHA_LOW 		0


/// CPSSM
#define SPI_SSM_EN     		1
#define SPI_SSM_DI     		0


////// FLAG MACROS ///////
#define SPI_FLAG_TX		 	(1<<1)
#define SPI_FLAG_RX		 	(1<<0)

////// INTERUPT MACROS ///////
#define SPI_READY 			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2




/******** Driver API's **********/
//-- Peripheral clock setup --//
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//-- Initialization / deinitialization --//
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx); /*Use the RCC reset register*/

//--Data reading / Writing--//
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

//-- Send / receive data--//
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
// IRQPriority is uint32_t because the might be larger than 255
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);
#endif
