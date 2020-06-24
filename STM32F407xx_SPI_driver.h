/*
 * STM32F407xx_SPI_driver.h
 *
 *  Created on: May 17, 2020
 *      Author: hp
 */

//The work implemented is specific to the SPI1 only. IT IS NOT GENERIC

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include <stdint.h>
#include "STM32F407xx.h"

// Structure configuration for the SPI peripheral
typedef struct
{
	uint8_t SPI_mode;                 //Master or slave mode
	uint8_t DataFormat;				  //8-bits or 16-bits
	uint8_t SlaveManagement;          //Software or hardware slave select management
	uint8_t CPHA;					  //Clock phase
	uint8_t CPOL;					  //Clock polarity
	uint8_t SPI_SCLKspeed;			  //Source clock frequency. This value controls the prescaler dividing the APB2 clock
	uint8_t BusConfig;				  //Full-duplex, half-duplex or simplex (RX only or TX only)
	//These members are needed when the SPI send/receive are based on interrupts
	uint8_t *TXbuffer;				  //Address of TXbuffer
	uint8_t *RXbuffer;				  //Address of RXbuffer
	uint8_t TXLength;				  //TX length
	uint8_t RXLength;				  //RX length
	uint8_t TXState;				  //TX state
	uint8_t RXState;				  //RX state
}SPI_config_t;

//SPI peripheral possible modes
#define SPI_DEVICE_SLAVE_MODE        0
#define SPI_DEVICE_MASTER_MODE  	 1

//SPI based interrupt possible states
#define SPI_READY					 0
#define SPI_BUSY_IN_RX				 1
#define SPI_BUSY_IN_TX				 2

//SPI peripheral possible data formats
#define SPI_DFF_8        0		//8 bits
#define SPI_DFF_16		 1  	//16 bits

//SPI peripheral possible slave select management modes
#define SPI_DEVICE_HARDWARE_SM       0			//The NSS pin must be pulled externally to ground to select the slave (the software slave management is disabled)
#define SPI_DEVICE_SOFTWARE_SM		 1  	 	//The NSS pin receives value internally from the SSI bit in SPI_CR1 register (the software slave management is enabled)

//SPI peripheral possible clock phase
#define SPI_CPHA_LOW     0 		//Data capture during first edge
#define SPI_CPHA_HIGH    1 		//Data capture during second edge

//SPI peripheral possible clock polarity
#define SPI_CPOL_LOW     0 		//The clock is low during IDLE state
#define SPI_CPOL_HIGH    1 		//The clock is high during IDLE state

//SPI peripheral possible SCLK
#define SPI_SCLK_SPEED_DIV_2          0     	//Prescaler=2; SCLK=PCLK2/2
#define SPI_SCLK_SPEED_DIV_4          1     	//Prescaler=4; SCLK=PCLK2/4
#define SPI_SCLK_SPEED_DIV_8          2     	//Prescaler=8; SCLK=PCLK2/8
#define SPI_SCLK_SPEED_DIV_16         3     	//Prescaler=16; SCLK=PCLK2/16
#define SPI_SCLK_SPEED_DIV_32         4     	//Prescaler=32; SCLK=PCLK2/32
#define SPI_SCLK_SPEED_DIV_64         5     	//Prescaler=64; SCLK=PCLK2/64
#define SPI_SCLK_SPEED_DIV_128        6     	//Prescaler=128; SCLK=PCLK2/128
#define SPI_SCLK_SPEED_DIV_256        7     	//Prescaler=256; SCLK=PCLK2/256

//SPI peripheral possible bus configuration
#define SPI_BUS_CONFIG_FD  	 		 0  		//Full duplex
#define SPI_BUS_CONFIG_HD  	 		 1			//Half duplex
#define SPI_BUS_CONFIG_S_RXONLY   	 2			//Simplex RX only

//Possible SPI Application events
#define EVT_SPI_TX_CMPLT   1
#define EVT_SPI_RX_CMPLT   2
#define EVT_SPI_OVR_ERR    3

//SPI2 init and reset
void SPI_init(SPI_config_t);
void SPI_reset(void);

//SPI2 peripheral clock setup
void SPI_clockControl(uint8_t);

//SPI2 peripheral: data send and receive: Blocking methods
void SPI_dataSend(uint8_t*, uint32_t);
void SPI_dataReceive(uint8_t*, uint32_t);

//SPI2 peripheral: data send and receive: Interrupt based methods
uint8_t SPI_dataSendIT(SPI_config_t,uint8_t*, uint32_t);
uint8_t SPI_dataReceiveIT(SPI_config_t,uint8_t*, uint32_t);

//IRQ configuration and ISR handling
void SPI_IRQcnfg(uint8_t, uint8_t);
void SPI_IRQpriorityCnfg(uint8_t, uint8_t);
void SPI_IRQhandler();

//SPI1 peripheral control
void SPI2_control(uint8_t);
//Application callback
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_config_t,uint8_t);
#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
