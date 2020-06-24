/*
 * STM32F407xx_USART_driver.c
 *
 *  Created on: 7 juin 2020
 *      Author: hp
 */

#include "STM32F407xx_USART_driver.h"
#include <math.h>
#include <stdio.h>

//USARTx clock frequency (helper function)
uint32_t USART_get_clockFreq(USART_Registers_t*);

//USARTx baud rate setting (helper function)
void USART_setBaudRate(USART_handle_t*);

/*
 * Calculate the bus frequency connected to the USARTx peripheral
 * @paramIN1: USARTx peripheral base address
 */
uint32_t USART_get_clockFreq(USART_Registers_t* USARTx)
{
	static uint32_t ahbPres[]={2,4,8,16,64,128,256,512};
	static uint32_t apbPres[]={2,4,8,16};
	uint32_t clockFreq=0;
	uint32_t ahbPre,apb1Pre,apb2Pre;
	if(!(RCC->CFGR & (0x03)))
	{
		//HSI oscillator selected as system clock
		clockFreq=16000000;
	}
	else if(RCC->CFGR & (0x1))
	{
		//HSE oscillator selected as system clock
		clockFreq=8000000;
	}
	uint8_t value=0;
	value= (RCC->CFGR>>4 & (0x0F));
	if(value<8)
	{
		ahbPre=1;
	}
	else
	{
		ahbPre=ahbPres[value-8];
	}
	//USART1 and USART6 are connected to APB2 bus
	if(USARTx==USART1 || USARTx==USART6)
	{
		value= (RCC->CFGR>>13 & (0x07));
		if(value<4)
		{
			apb2Pre=1;
		}
		else
		{
			apb2Pre=apbPres[value-4];
		}
		clockFreq=clockFreq/ahbPre/apb2Pre;
	}
	//USART2 and USART3 are connected to APB1 bus
	else if(USARTx==USART2 || USARTx==USART3)
	{
		value= (RCC->CFGR>>10 & (0x07));
		if(value<4)
		{
			apb1Pre=1;
		}
		else
		{
			apb1Pre=apbPres[value-4];
		}
		clockFreq=clockFreq/ahbPre/apb1Pre;
	}
	return clockFreq/1000000;
}

/*
 * Calculate the USARTDIV to get the baudrate desired
 * @paramIN1: USARTx handle structure
 */
void USART_setBaudRate(USART_handle_t* USART_handle)
{
	uint8_t over8=0;
	float USART_div=0;
	uint16_t DIV_mantissa=0;
	uint8_t DIV_fraction=0;
	over8= (USART_handle->USARTx->CR1>>USART_CR1_OVER8) & (0x1);
	USART_div= (float)(USART_get_clockFreq(USART_handle->USARTx)*1000000)/(8*USART_handle->USART_cfg.USART_baud*(2-over8));
	DIV_mantissa=(uint16_t)USART_div;
	USART_handle->USARTx->BRR |= (DIV_mantissa<<USART_BRR_MANTISSA);
	if(over8)
	{
		DIV_fraction=round((USART_div-DIV_mantissa)*8);
	}
	else
	{
		DIV_fraction=round((USART_div-DIV_mantissa)*16);
	}
	USART_handle->USARTx->BRR |= (DIV_fraction<<USART_BRR_FRACTION);
}

//USARTx peripheral clock setup
/*Enable or disable the USARTx peripheral clock
 *@param1: The base address of the USARTx peripheral
 *@param2: ENABLE macro to enable or DISABLE macro to disable
 */
void USART_clockControl(USART_Registers_t* USARTx, uint8_t EnorDI)
{
	if(EnorDI==ENABLE)
	{
		if(USARTx==USART1)
		{
			USART1_CLOCK_ENABLE();
		}
		else if(USARTx==USART2)
		{
			USART2_CLOCK_ENABLE();
		}
		else if(USARTx==USART3)
		{
			USART3_CLOCK_ENABLE();
		}
		else if(USARTx==USART6)
		{
			USART6_CLOCK_ENABLE();
		}
	}
	else if(EnorDI==DISABLE)
	{
		if(USARTx==USART1)
		{
			USART1_CLOCK_DISABLE();
		}
		else if(USARTx==USART2)
		{
			USART2_CLOCK_DISABLE();
		}
		else if(USARTx==USART3)
		{
			USART3_CLOCK_DISABLE();
		}
		else if(USARTx==USART6)
		{
			USART6_CLOCK_DISABLE();
		}
	}
}

//USARTx peripheral control
/*Enable or disable the USARTx peripheral
 *@param1: USARTx peripheral's base address
 *@param1: Enable or disable
 */
void USART_PeriControl(USART_Registers_t* USARTx, uint8_t ENorDI)
{
	if(ENorDI==ENABLE)
	{
		USARTx->CR1 |= (0x1<<USART_CR1_UE);
	}
	else
	{
		USARTx->CR1 &=~(0x1<<USART_CR1_UE);
	}
}

void USART_init(USART_handle_t USART_handle)
{
	if(USART_handle.USART_cfg.USART_mode==USART_MODE_ONLY_RX)
	{
		USART_handle.USARTx->CR1 |= (0x1<<USART_CR1_RE);
	}
	else if (USART_handle.USART_cfg.USART_mode==USART_MODE_ONLY_TX)
	{
		USART_handle.USARTx->CR1 |= (0x1<<USART_CR1_TE);
	}
	else if (USART_handle.USART_cfg.USART_mode==USART_MODE_TXRX)
	{
		USART_handle.USARTx->CR1 |= (0x1<<USART_CR1_RE);
		USART_handle.USARTx->CR1 |= (0x1<<USART_CR1_TE);
	}
	USART_handle.USARTx->CR1 |= (USART_handle.USART_cfg.USART_wordLen<<USART_CR1_M);
	USART_handle.USARTx->CR1 |= (USART_handle.USART_cfg.USART_parity<<USART_CR1_PS);
	USART_handle.USARTx->CR2 |= (USART_handle.USART_cfg.USART_stopBits<<USART_CR2_STOP);
	USART_handle.USARTx->CR3 |= (USART_handle.USART_cfg.USART_HWflowControl<<USART_CR3_RTSE);
	USART_handle.USARTx->CR1 |= (USART_handle.USART_cfg.USART_OVER<<USART_CR1_OVER8);
	USART_setBaudRate(&USART_handle);
}

//USARTx send data process
/*Send the data with input length
 *@param1: USART peripheral base address
 *@param2: a pointer to the TXbuffer
 *@param3: the length of data to be sent
 *@NOTE: This is a blocking call of the send data function */
void USART_dataSend(USART_handle_t* USART_handle, uint8_t* TXbuffer, uint32_t length)
{
	uint16_t* data=0;
	for(uint32_t i=0;i<length;i++)
	{
		//Wait until the TXE flag is set
		while(!(USART_handle->USARTx->SR & (0x1<<USART_SR_TXE)));
		//Check for the word length
		if(USART_handle->USART_cfg.USART_wordLen==USART_WORDLEN_8BITS)
		{
			//8 bit data transfer
			USART_handle->USARTx->DR=*TXbuffer & 0xFF;
			//Increment the TX buffer address
			TXbuffer++;
		}
		else
		{
			//9 bit data transfer
			*data=*((uint16_t*) TXbuffer) & (0x01FF);
			USART_handle->USARTx->DR= *data;
			if(USART_handle->USART_cfg.USART_parity==USART_PARITY_DISABLE)
			{
				//Parity disabled. So the 9th bit sent is the data bit
				//Increment the TX buffer twice
				TXbuffer++;
				TXbuffer++;
			}
			else
			{
				//Parity enabled. So the the 9th is the parity bit and not the data bit
				//Increment the TX buffer once
				TXbuffer++;
			}
		}
	}
	//Wait until the communication is complete (TC flag is set)
	while (!(USART_handle->USARTx->SR & (0x1<<USART_SR_TC)));
}

//USARTx send data process using interrupts
/*Send the data with input length
 *@param1: USART peripheral base address
 *@param2: a pointer to the TXbuffer
 *@param3: the length of data to be sent
 *@NOTE: This is a non-blocking call of the send data function */
uint8_t USART_dataSendIT(USART_handle_t* USART_handle, uint8_t* TXbuffer, uint32_t length)
{
	uint8_t TXstate=USART_handle->TXstate;
	if(USART_handle->TXstate != USART_TX_BUSY)
	{
		USART_handle->TXstate=USART_TX_BUSY;
		USART_handle->TXbuffer=TXbuffer;
		USART_handle->TXlength=length;

		//Enable the TXE interrupt
		USART_handle->USARTx->CR1 |= (0x1<<USART_CR1_TXEIE);
		//Enable the TC interrupt
		USART_handle->USARTx->CR1 |= (0x1<<USART_CR1_TCIE);
	}
	return TXstate;
}

//USARTx receive data process
/*Receive the data with input length
 *@param1: USART peripheral base address
 *@param2: a pointer to the RXbuffer
 *@param3: the length of data to be received
 *@NOTE: This is a blocking call of the receive data function */
void USART_dataReceive(USART_handle_t* USART_handle, uint8_t* RXbuffer, uint32_t length)
{
	for(uint32_t i=0;i<length;i++)
	{
		//Wait until the RXNE flag is set
		while(!(USART_handle->USARTx->SR & (0x1<<USART_SR_RXNE)));
		//Check for the word length
		if(USART_handle->USART_cfg.USART_wordLen==USART_WORDLEN_8BITS)
		{
			//8 bit data reception
			if(USART_handle->USART_cfg.USART_parity==USART_PARITY_DISABLE)
			{
				//Parity control disabled, so the 8 bits received are data bits
				*RXbuffer=USART_handle->USARTx->DR & (0xFF);
				//Increment the RX buffer address
				RXbuffer++;
			}
			else
			{
				//Parity control enabled, so the 8th bit received will be the parity bit
				//We read only the 7 received data bits
				*RXbuffer=USART_handle->USARTx->DR & (0x7F);
				//Increment the RX buffer address
				RXbuffer++;
			}
		}
		else
		{
			//9 bit data transfer
			//Check whether the parity control is enabled
			if(USART_handle->USART_cfg.USART_parity==USART_PARITY_DISABLE)
			{
				//Parity disabled. So the 9th bit sent is the data bit
				//Increment the TX buffer twice
				*((uint16_t*) RXbuffer)= ((uint16_t)USART_handle->USARTx->DR) & (0x01FF);
				//Increment the RX buffer address twice
				RXbuffer++;
				RXbuffer++;
			}
			else
			{
				//Parity enabled. So the the 9th is the parity bit and not the data bit
				*RXbuffer= USART_handle->USARTx->DR;
				//Increment the RX buffer once
				RXbuffer++;
			}
		}
	}
}

//USARTx receive data process using interrupts
/*Send the data with input length
 *@param1: USART peripheral base address
 *@param2: a pointer to the TXbuffer
 *@param3: the length of data to be sent
 *@NOTE: This is a non-blocking call of the send data function */
uint8_t USART_dataReceiveIT(USART_handle_t* USART_handle, uint8_t* RXbuffer, uint32_t length)
{
	uint8_t RXstate=USART_handle->RXstate;
	if(USART_handle->RXstate != USART_RX_BUSY)
	{
		USART_handle->RXstate=USART_RX_BUSY;
		USART_handle->RXbuffer=RXbuffer;
		USART_handle->RXlength=length;
		(void)USART_handle->USARTx->DR;
		//Enable the RXNE interrupt
		USART_handle->USARTx->CR1 |= (0x1<<USART_CR1_RXNEIE);
	}
	return RXstate;
}

//IRQ configuration
/*@INparam1: the IRQ number
 *@INparam2: enable or disable the interruption
 */
void USART_IRQcnfg(uint8_t IRQ_number, uint8_t ONorOFF)
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
void USART_IRQpriorityCnfg(uint8_t IRQ_number, uint8_t IRQ_priority)
{
	uint8_t temp1=IRQ_number/4;
	uint8_t temp2=IRQ_number%4;
	*(NVIC_IPR0_BASE_ADDRESS + (4*temp1)) |= ((0x1<<(8*temp2))<<4);
}

void USART_IRQhandle(USART_handle_t* USART_handle)
{
	uint8_t temp1= USART_handle->USARTx->CR1>>USART_CR1_TXEIE & 0x1;
	uint8_t temp2= USART_handle->USARTx->SR>>USART_SR_TXE & 0x1;
	//Check if the interrupt is generated by TXE event
	if(temp1 && temp2)
	{
		//Check if the device is busy in USART TX
		if(USART_handle->TXstate==USART_TX_BUSY)
		{
			//Check if there is more data to transmit
			if(USART_handle->TXlength>0)
			{
				//Check the word length (8 or 9 bits)
				if (USART_handle->USARTx->CR1 & (0x1<<USART_CR1_M))
				{
					//9 bits
					USART_handle->USARTx->DR=*((uint16_t*)USART_handle->TXbuffer) & (0x1FF);
					//Check if the parity control is enabled
					if(USART_handle->USARTx->CR1 & (0x1<<USART_CR1_PCE))
					{
						//Parity enabled, so the 9th bit sent is the parity bit
						USART_handle->TXbuffer++;
						USART_handle->TXlength--;
					}
					else
					{
						//Parity disabled, so the 9th bit sent is a data bit
						USART_handle->TXbuffer++;
						USART_handle->TXbuffer++;
						USART_handle->TXlength--;
						USART_handle->TXlength--;
					}
				}
				else
				{
					//8 bits
					USART_handle->USARTx->DR = *USART_handle->TXbuffer & (0xFF);
					USART_handle->TXbuffer++;
					USART_handle->TXlength--;
				}
			}
		}
	}
	temp1=USART_handle->USARTx->CR1>>USART_CR1_RXNEIE & 0x1;
	temp2=USART_handle->USARTx->SR>>USART_SR_RXNE & 0x1;
	//Check if the interrupt is generated by RXNE event
	if(temp1 && temp2)
	{
		//Check if the device is busy in USART RX
		if(USART_handle->RXstate ==USART_RX_BUSY)
		{
			//Check if there is more data to receive
			if(USART_handle->RXlength>0)
			{
				//Check the word length (8 or 9 bits)
				if (USART_handle->USARTx->CR1 & (0x1<<USART_CR1_M))
				{
					//9 bits
					//Check if the parity control is enabled
					if(USART_handle->USARTx->CR1 & (0x1<<USART_CR1_PCE))
					{
						//Parity enabled, so the 9th received sent is the parity bit
						*USART_handle->RXbuffer=USART_handle->USARTx->DR & 0xFF;
						USART_handle->RXbuffer++;
						USART_handle->RXlength--;
					}
					else
					{
						//Parity disabled, so the 9th bit received is a data bit
						*USART_handle->RXbuffer=USART_handle->USARTx->DR & 0x1FF;
						USART_handle->RXbuffer++;
						USART_handle->RXbuffer++;
						USART_handle->RXlength--;
						USART_handle->RXlength--;
					}
				}
				else
				{
					//8 bits
					//Check if the parity control is enabled
					if(USART_handle->USARTx->CR1 & (0x1<<USART_CR1_PCE))
					{
						//Parity enabled, so the 8th bit received is a parity bit
						*USART_handle->RXbuffer=USART_handle->USARTx->DR & 0x7F;
					}
					else
					{
						//Parity disabled, so the 8 th bit received is a data bit
						*USART_handle->RXbuffer=USART_handle->USARTx->DR & 0xFF;
					}
					USART_handle->RXbuffer++;
					USART_handle->RXlength--;
				}
			}
			if(USART_handle->RXlength==0)
			{
				//No more data to be received
				//Disable the RXNE flag interrupt
				USART_handle->USARTx->CR1 &=~ (0x1<<USART_CR1_RXNEIE);
				USART_handle->RXstate=USART_READY;
				USART_ApplicationEventCallback(USART_handle, USART_EVT_RX_COMPLETE);
			}
		}
	}
	temp1= USART_handle->USARTx->SR>>USART_SR_TC & (0x1);
	temp2= USART_handle->USARTx->CR1>>USART_CR1_TCIE & (0x1);
	//Check if the interrupt is generated by TC event
	if(temp1 && temp2)
	{
		//Check if the device is busy in USART TX
		if(USART_handle->TXstate==USART_TX_BUSY)
		{
			//Check if the length is null
			if(USART_handle->TXlength==0)
			{
				USART_handle->USARTx->CR1 &=~ (0x1<<USART_CR1_TXEIE);
				USART_handle->USARTx->CR1 &=~ (0x1<<USART_CR1_TCIE);
				uint32_t dummyRead=0;
				uint32_t dummyWrite=0;
				//Close the TX communication
				dummyRead=USART_handle->USARTx->SR;
				(void)dummyRead;
				USART_handle->USARTx->DR |= dummyWrite;

				USART_handle->TXstate=USART_READY;
				USART_handle->TXlength=0;
				USART_handle->TXbuffer=NULL;
				USART_ApplicationEventCallback(USART_handle, USART_EVT_TX_COMPLETE);
			}
		}
	}
}

__attribute__((weak)) void USART_ApplicationEventCallback(USART_handle_t *USART_handle,uint8_t event)
{

}
