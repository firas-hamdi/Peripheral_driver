/*
 * STM32F407xx_SPI_driver.c
 *
 *  Created on: May 17, 2020
 *      Author: hp
 */

#include "STM32F407xx_SPI_driver.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>


static void SPI_OVR_IT_handler();
static void SPI_TX_IT_handler();
static void SPI_RX_IT_handler();

//SPI2 peripheral clock setup
/*Enable or disable the SPI2 peripheral clock
 *@param1: The base address of the SPI2 peripheral
 *@param2: ENABLE macro to enable or DISABLE macro to disable*/
void SPI_clockControl(uint8_t ONorOFF)
{
	if(ONorOFF==ENABLE)
	{
		SPI2_CLOCK_ENABLE();
	}
	else
	{
		SPI2_CLOCK_DISABLE();
	}
}

//SPI peripheral initialization
/*Initialize the SPI2 peripheral according to the user's input
 *@param1: The handle structure containing the SPI peripheral address and the configuration parameters
 *Note: Must be called after enabling the peripheral clock @SPI_clockControl*/
void SPI_init(SPI_config_t SPIconfig)
{
	uint32_t temp=0;
	temp |= SPIconfig.SPI_mode<<2;
	temp |= SPIconfig.DataFormat<<11;
	temp |= SPIconfig.SlaveManagement<<9;
	temp |= SPIconfig.CPHA;
	temp |= SPIconfig.CPOL<<1;
	switch(SPIconfig.SPI_SCLKspeed)
	{
	case SPI_SCLK_SPEED_DIV_2:
		temp &=~(0x7<<3);
		break;
	case SPI_SCLK_SPEED_DIV_4 :
		temp |=(SPI_SCLK_SPEED_DIV_4<<3);
		break;
	case SPI_SCLK_SPEED_DIV_8:
		temp |=(SPI_SCLK_SPEED_DIV_8<<3);
		break;
	case SPI_SCLK_SPEED_DIV_16:
		temp |=(SPI_SCLK_SPEED_DIV_16<<3);
		break;
	case SPI_SCLK_SPEED_DIV_32:
		temp |=(SPI_SCLK_SPEED_DIV_32<<3);
		break;
	case SPI_SCLK_SPEED_DIV_64:
		temp |=(SPI_SCLK_SPEED_DIV_64<<3);
		break;
	case SPI_SCLK_SPEED_DIV_128:
		temp |=(SPI_SCLK_SPEED_DIV_128<<3);
		break;
	case SPI_SCLK_SPEED_DIV_256:
		temp |=(SPI_SCLK_SPEED_DIV_256<<3);
		break;
	default:
		break;
	}

	switch(SPIconfig.BusConfig)
	{
	case SPI_BUS_CONFIG_FD:
		temp &=~(0x1<<15);
		break;
	case SPI_BUS_CONFIG_HD:
		temp |=(0x1<<15);
		break;
	case SPI_BUS_CONFIG_S_RXONLY:
		temp &=~(0x1<<15);
		temp |= (0x1<<10);
		break;
	default:
		break;
	}
	*SPI2_CR1 |=temp;
}

//SPI1 peripheral reset
/*De-init the SPI1 peripheral registers*/
void SPI_reset(void)
{
	RCC->APB1RSTR |= (0x1<<12);      //Reset state
	RCC->APB1RSTR &=~ (0x1<<12);	 //Clear the reset bit of SPI2
}

//SPI send data process
/*Send the data with input length
 *@param1: a pointer to the TXbuffer
 *@param2: the length of data to be sent
 *@NOTE: This is a blocking call of the send data function */
void SPI_dataSend(uint8_t* TXbuffer, uint32_t length)
{
	while(length!=0)
	{
		while (!(*SPI2_SR &(0x1<<SPI2_SR_TXE))) ; //BLOCK: Wait until the TX buffer is empty
		if(*SPI2_CR1 &(0x1<<SPI2_CR1_DFF))
		{
			*SPI2_DR =*((uint16_t*) TXbuffer);
			(uint16_t*)TXbuffer++;
			length--;
			length--;
		}
		else
		{
			*SPI2_DR =*TXbuffer;
			TXbuffer++;
			length--;
		}
	}
}
/*Send the data with input length when a TXE interrupt is generated
 *@param1: a pointer to the TXbuffer
 *@param2: the length of data to be sent*/

uint8_t SPI_dataSendIT(SPI_config_t SPI_configuration, uint8_t* TXbuffer, uint32_t length)
{
	uint8_t SPIstate=SPI_configuration.TXState;
	//Every SPI TX communication should be established only when the same SPI peripheral is not busy with other TX communication
	if(SPIstate != SPI_BUSY_IN_TX)
	{
		//Store the address of Tx buffer and the length
		SPI_configuration.TXbuffer=TXbuffer;
		SPI_configuration.TXLength=length;
		//Mark the SPI state as busy so that no other code can use same SPI peripheral until transmission is over
		SPI_configuration.TXState=SPI_BUSY_IN_TX;
		//Enable the interrupt generated when the TX buffer is empty
		*SPI2_CR2 |= (0x1<<SPI2_CR2_TXEIE);
	}
	return SPIstate;
}
//SPI read data process
/*read the data with input length
 *@param1: a pointer to the data to be received (RX buffer)
 *@param2: the length of data to be received
 *@NOTE: This is a blocking call of the send data function */
void SPI_dataReceive( uint8_t *RXbuffer,uint32_t length)
{
	while(length)
	{
		while (!(*SPI2_SR &(0x1<<SPI2_SR_RXNE))) ; //BLOCK: Wait until the RX buffer is not empty
		if(*SPI2_CR1 &(0x1<<SPI2_CR1_DFF))
		{
			*((uint16_t*) RXbuffer)= *SPI2_DR;
			length--;
			length--;
			(uint16_t*)RXbuffer++;
		}
		else
		{
			*RXbuffer=*SPI2_DR ;
			length--;
			RXbuffer++;
		}
	}
}

/*Receive the data when a RXNE interrupt is generated
 *@param1: a pointer to the RXbuffer
 *@param2: the length of data to be received*/

uint8_t SPI_dataReceiveIT(SPI_config_t SPI_configuration, uint8_t* RXbuffer, uint32_t length)
{
	uint8_t SPIstate=SPI_configuration.RXState;
	//Every SPI RX communication should be established only when the same SPI peripheral is not busy with other RX communication
	if(SPIstate != SPI_BUSY_IN_RX)
	{
		//Store the address of RX buffer and the length
		SPI_configuration.RXbuffer=RXbuffer;
		SPI_configuration.RXLength=length;
		//Mark the SPI state as busy so that no other code can use same SPI peripheral until reception is over
		SPI_configuration.RXState=SPI_BUSY_IN_RX;
		//Enable the interrupt generated when the RX buffer is empty
		*SPI2_CR2 |= (0x1<<SPI2_CR2_RXNEIE);
	}
	return SPIstate;
}
//SPI1 peripheral control
/*Enable or disable the SPI1 peripheral
 *@param1: Enable or disable
 */
void SPI2_control(uint8_t ONorOFF)
{
	if(ONorOFF==ENABLE)
	{
		*SPI2_CR1 |= (0x1<<SPI2_CR1_SPE);
	}
	else
	{
		*SPI2_CR1 &=~ (0x1<<SPI2_CR1_SPE);
	}

}

//IRQ configuration
/*@INparam1: the IRQ number
 *@INparam2: enable or disable the interruption
 */
void SPI_IRQcnfg(uint8_t IRQ_number, uint8_t ONorOFF)
{
	if(ONorOFF==ENABLE)
	{
		if(IRQ_number<32)
		{
			*NVIC_ISER0_BASE_ADDRESS |= (0x1<<IRQ_number);
		}
		if((IRQ_number<64)&&(IRQ_number>=32))
		{
			*NVIC_ISER1_BASE_ADDRESS |= (0x1<<(IRQ_number%32));
		}
		if((IRQ_number<96)&&(IRQ_number>=64))
		{
			*NVIC_ISER2_BASE_ADDRESS |= (0x1<<(IRQ_number%64));
		}
	}
	else
	{
		if(IRQ_number<32)
		{
			*NVIC_ICER0_BASE_ADDRESS |= (0x1<<IRQ_number);
		}
		if((IRQ_number<64)&&(IRQ_number>=32))
		{
			*NVIC_ICER1_BASE_ADDRESS |= (0x1<<(IRQ_number%32));
		}
		if((IRQ_number<96)&&(IRQ_number>=64))
		{
			*NVIC_ICER2_BASE_ADDRESS |= (0x1<<(IRQ_number%64));
		}
	}
}

//IRQ priority configuration
/*@INparam1: the IRQ number
 *@INparam2: the IRQ priority
 */
void SPI_IRQpriorityCnfg(uint8_t IRQ_number, uint8_t IRQ_priority)
{
	uint8_t temp1=IRQ_number/4;
	uint8_t temp2=IRQ_number%4;
	*(NVIC_IPR0_BASE_ADDRESS + (4*temp1)) |= ((0x1<<(8*temp2))<<4);
}

void SPI_IRQhandler(SPI_config_t SPI_cnfg)
{
	uint8_t temp1,temp2;
	temp1= *SPI2_SR & (0x1<< SPI2_SR_TXE);
	temp2= *SPI2_CR2 & (0x1<<SPI2_CR2_TXEIE);
	if(temp1 && temp2)
	{
		SPI_TX_IT_handler(SPI_cnfg);
	}

	temp1= *SPI2_SR & (0x1<< SPI2_SR_RXNE);
	temp2= *SPI2_CR2 & (0x1<<SPI2_CR2_RXNEIE);
	if(temp1 && temp2)
	{
		SPI_RX_IT_handler(SPI_cnfg);
	}

	temp1= *SPI2_SR & (0x1<< SPI2_SR_OVR);
	temp2= *SPI2_CR2 & (0x1<<SPI2_CR2_ERRIE);
	if(temp1 && temp2)
	{
		SPI_OVR_IT_handler(SPI_cnfg);
	}
}

static void SPI_TX_IT_handler(SPI_config_t SPI_cnfg)
{
	if(*SPI2_CR1 &(0x1<<SPI2_CR1_DFF))
	{
		*SPI2_DR =*((uint16_t*) SPI_cnfg.TXbuffer);
		(uint16_t*)SPI_cnfg.TXbuffer++;
		SPI_cnfg.TXLength--;
		SPI_cnfg.TXLength--;
	}
	else
	{
		*SPI2_DR =*(SPI_cnfg.TXbuffer);
		SPI_cnfg.TXbuffer++;
		SPI_cnfg.TXLength--;
	}
	if(!SPI_cnfg.TXLength)
	{
		/*TX length is zero
		 * Close SPI communication
		 * Inform the application that TX is over
		 */
		*SPI2_CR2 &=~ (0x1<<SPI2_CR2_TXEIE);
		SPI_cnfg.TXbuffer=NULL;
		SPI_cnfg.TXLength=0;
		SPI_cnfg.TXState=SPI_READY;
		SPI_ApplicationEventCallback(SPI_cnfg,EVT_SPI_TX_CMPLT);
	}
}

static void SPI_RX_IT_handler(SPI_config_t SPI_cnfg)
{
	if(*SPI2_CR1 &(0x1<<SPI2_CR1_DFF))
	{
		*((uint16_t*) SPI_cnfg.RXbuffer)= *SPI2_DR;
		SPI_cnfg.RXLength--;
		SPI_cnfg.RXLength--;
		(uint16_t*)SPI_cnfg.RXbuffer++;
	}
	else
	{
		*SPI_cnfg.RXbuffer=*SPI2_DR ;
		SPI_cnfg.RXLength--;
		SPI_cnfg.RXbuffer++;
	}
	if (!SPI_cnfg.RXLength)
	{
		/*TX length is zero
		* Close SPI communication
		* Inform the application that TX is over
	    */
		*SPI2_CR2 &=~ (0x1<<SPI2_CR2_RXNEIE);
		SPI_cnfg.RXbuffer=NULL;
		SPI_cnfg.RXLength=0;
		SPI_cnfg.RXState=SPI_READY;
		SPI_ApplicationEventCallback(SPI_cnfg,EVT_SPI_RX_CMPLT);
	}
}

static void SPI_OVR_IT_handler(SPI_config_t SPI_cnfg)
{
	uint8_t temp;
	if(SPI_cnfg.TXState!=SPI_BUSY_IN_TX)
	{
		//clear the overrun flag
		temp=*SPI2_DR;
		temp=*SPI2_SR;
	}
	(void)temp;
	//Inform the application that an overrun error occured
	SPI_ApplicationEventCallback(SPI_cnfg,EVT_SPI_OVR_ERR);
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_config_t SPI_cnfg,uint8_t EVT)
{

}
