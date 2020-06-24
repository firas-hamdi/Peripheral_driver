/*
 * STM32F407xx_I2C_driver.c
 *
 *  Created on: 23 mai 2020
 *      Author: hp
 */
#include "STM32F407xx_I2C_driver.h"
#include <stdio.h>
#include <string.h>

uint16_t static ahbPreArr[]={2,4,8,16,32,64,128,256,512};
uint8_t static apbPreArr[]={2,4,8,16};

uint32_t I2C_GetClockFrequency()
{
	uint32_t I2C_CLKfreq=0;
	uint32_t SYSCLK=0;
	if(!(RCC->CFGR & 0x3))
	{
		SYSCLK=16000000;			//HSI oscillator selected as system slock
	}
	else if (RCC->CFGR & 0x1)
	{
		SYSCLK=8000000;				//HSE oscillator selected as system slock
	}
	uint8_t value=0;
	uint16_t ahbPre=0;
	uint8_t apbPre=0;
	value= (RCC->CFGR>>4) & (0xF);
	if(value<8)
	{
		ahbPre=1;
	}
	else
	{
		ahbPre=ahbPreArr[value-8];
	}
	value= (RCC->CFGR>>10) & (0x7);
	if(value<4)
	{
		apbPre=1;
	}
	else
	{
		apbPre=apbPreArr[value-4];
	}
	I2C_CLKfreq=SYSCLK/ahbPre/apbPre;
	return (I2C_CLKfreq/1000000);
}

void I2C_clockControl(I2C_Registers_t* I2C, uint8_t EnorDI)
{
	if(EnorDI==ENABLE)
	{
		if(I2C==I2C1)
		{
			I2C1_CLOCK_ENABLE();
		}
		else if(I2C==I2C2)
		{
			I2C2_CLOCK_ENABLE();
		}
		else if(I2C==I2C3)
		{
			I2C3_CLOCK_ENABLE();
		}
	}
	else if(EnorDI==DISABLE)
	{
		if(I2C==I2C1)
		{
			I2C1_CLOCK_DISABLE();
		}
		else if(I2C==I2C2)
		{
			I2C2_CLOCK_DISABLE();
		}
		else if(I2C==I2C3)
		{
			I2C3_CLOCK_DISABLE();
		}
	}
}

void I2C_PeriControl(I2C_Registers_t* I2Cx, uint8_t ENorDI)
{
	if(ENorDI==ENABLE)
	{
		I2Cx->CR1 |= (0x1);
	}
	else
	{
		I2Cx->CR1 &=~(0x1);
	}
}

void I2C_init(I2C_handle_t* I2C_handle)
{
	uint16_t CCR_value=0;
	I2C_handle->I2Cx->CR2|=(I2C_GetClockFrequency() & 0x3F);
	I2C_handle->I2Cx->OAR1|=(I2C_handle->I2C_config.I2C_deviceAddress<<I2C_OAR1_ADDR);
	//This is done according to the reference manual: "BIT14 of OAR1 must be set by software"
	I2C_handle->I2Cx->OAR1|=(1<<14);
	if(I2C_handle->I2C_config.I2C_SCL<=I2C_SCL_SM_100Khz)
	{
		//Standard mode
		//CCR calculation
		CCR_value=(I2C_GetClockFrequency()*1000000) / ((2*I2C_handle->I2C_config.I2C_SCL));
		I2C_handle->I2Cx->CCR |= CCR_value & 0xFFF;
	}
	else
	{
		//Fast mode
		I2C_handle->I2Cx->CCR |= (I2C_FM_DUTY_2 << I2C_CCR_FS);
		switch (I2C_handle->I2C_config.i2C_FMDutyCycle)
		{
		case I2C_FM_DUTY_2:
			//Tlow=2*Thigh
			//CCR calculation
			CCR_value=(I2C_GetClockFrequency()) / ((3*I2C_handle->I2C_config.I2C_SCL)/1000);
			I2C_handle->I2Cx->CCR |= CCR_value & 0xFFF;
			break;
		case I2C_FM_DUTY_16_9:
			//Tlow=(16*Thigh)/9
			I2C_handle->I2Cx->CCR |= (I2C_FM_DUTY_16_9<<I2C_CCR_FM_DUTY);
			//CCR calculation
			CCR_value=(I2C_GetClockFrequency()) / ((25*I2C_handle->I2C_config.I2C_SCL)/1000);
			I2C_handle->I2Cx->CCR |= CCR_value & 0xFFF;
		}
	}
	uint8_t riseTime=0;
	//Trise calculation
	if (!(I2C_handle->I2Cx->CCR & (0x1<<I2C_CCR_FS)))
	{
		//Standard mode
		riseTime=(I2C_GetClockFrequency())+1;
	}
	else
	{
		riseTime=((I2C_GetClockFrequency()*300)/1000U) +1;
	}
	I2C_handle->I2Cx->TRISE &=~(0x3F);
	I2C_handle->I2Cx->TRISE |= (riseTime & 0x3F);
}

void I2C_MasterSendData(I2C_handle_t *I2C_handle, uint8_t *TXbuffer, uint32_t length, uint8_t slaveAddress, uint8_t rs)
{
	//Generate the Start condition
	I2C_handle->I2Cx->CR1 |= (0x1<<I2C_CR1_START);
	/*Wait until the start condition is generated
	 * When the start condition is generated, the SB bit of I2C_SR1 is set
	 */
	while(!(I2C_handle->I2Cx->SR1 & (0x1)));
	//Send the address of the slave (7 bits) with r/#w bit cleared(0) (0: Write operation;1: Read operation)
	I2C_handle->I2Cx->DR = slaveAddress<<1;
	I2C_handle->I2Cx->DR &=~(0x1);
	/*Check whether the address phase is completed
	 * Read the ADDR bit of I2C_SR1
	 */
	while(!(I2C_handle->I2Cx->SR1 & (0x1<<I2C_SR1_ADDR)));
	//Clear the ADDR bit in SR1 by reading the SR2
	uint32_t dummyRead=0;
	dummyRead= I2C_handle->I2Cx->SR2;
	(void)dummyRead;
	//Send the data until length becomes zero
	while (length)
	{
		//Check whether the TXE is set: Data can be loaded into the Data register
		while (!(I2C_handle->I2Cx->SR1 &(0x1<<I2C_SR1_TXE)));
		I2C_handle->I2Cx->DR=*TXbuffer;
		TXbuffer++;
		length--;
	}
	/*After length reached zero, wait until TXE is set and BTF is set
	 * After they are set, a STOP condition can be generated to close the I2C communication
	 */
	while(!(I2C_handle->I2Cx->SR1 & (0x1<<I2C_SR1_TXE)));
	while(!(I2C_handle->I2Cx->SR1 & (0x1<<I2C_SR1_BTF)));
	if(!rs)
	{
		I2C_handle->I2Cx->CR1 |= (0x1<<I2C_CR1_STOP);
	}
}

void I2C_SlaveSendData(I2C_Registers_t* I2Cx,uint8_t data)
{
	I2Cx->DR=data;
}

uint8_t I2C_SlaveReceiveData(I2C_Registers_t* I2Cx)
{
	return (uint8_t) I2Cx->DR;
}


uint8_t I2C_MasterSendDataIT(I2C_handle_t* I2C_handle, uint8_t *TxBuffer, uint32_t length, uint8_t slaveAddress, uint8_t rs)
{
	uint8_t I2Cstate= I2C_handle->RxTxState;
	//Every I2C communication should be established only when the device is not busy
	if((I2Cstate != I2C_RX_BUSY) && (I2Cstate != I2C_TX_BUSY))
	{
		//The bus is busy
		I2C_handle->RxTxState=I2C_TX_BUSY;
		I2C_handle->TxBuffer=TxBuffer;
		I2C_handle->TxLength=length;
		I2C_handle->rs=rs;
		I2C_handle->SlaveAddress=slaveAddress;

		//Generate the start condition
		I2C_handle->I2Cx->CR1 |= (0x1<<I2C_CR1_START);

		//Enable the transmission event and error related interrupts
		I2C_handle->I2Cx->CR2 |= (0x1<<I2C_CR2_ITBUFEN);
		I2C_handle->I2Cx->CR2 |= (0x1<<I2C_CR2_ITEVTEN);
		I2C_handle->I2Cx->CR2 |= (0x1<<I2C_CR2_ITERREN);
	}
	return I2Cstate;
}

void I2C_MasterReceiveData(I2C_handle_t *I2C_handle, uint8_t *RXbuffer, uint32_t length, uint8_t slaveAddress, uint8_t rs)
{
	//Generate the Start condition
	I2C_handle->I2Cx->CR1 |= (0x1<<I2C_CR1_START);
	/*Wait until the start condition is generated
	 * When the start condition is generated, the SB bit of I2C_SR1 is set
	 */
	while(!(I2C_handle->I2Cx->SR1 & (0x1)));
	//Send the address of the slave (7 bits) with r/#w bit set(1) (0: Write operation;1: Read operation)
	I2C_handle->I2Cx->DR = (slaveAddress<<1)|(0x1);
	/*Check whether the address phase is completed
	 * Read the ADDR bit of I2C_SR1
	 */
	while(!(I2C_handle->I2Cx->SR1 & (0x1<<I2C_SR1_ADDR)));
	//To read only one byte from slave
	if(length==1)
	{
		/*Disable the ACK
		 * By doing so, the master orders the slave to stop sending data
		 */
		I2C_handle->I2Cx->CR1 &=~ (0x1<<I2C_CR1_ACK);
		//Clear the ADDR bit in SR1 by reading the SR2
		uint32_t dummyRead=0;
		dummyRead= I2C_handle->I2Cx->SR1;
		dummyRead= I2C_handle->I2Cx->SR2;
		(void)dummyRead;
		//Wait until RXNE is set
		while(!(I2C_handle->I2Cx->SR1 & (0x1<<I2C_SR1_RXNE)));
		//Generate the stop condition
		if(!rs)
		{
			I2C_handle->I2Cx->CR1 |= (0x1<<I2C_CR1_STOP);
		}
		*RXbuffer= I2C_handle->I2Cx->DR;
	}
	//To read more than one byte from slave
	if(length>1)
	{
		//Clear the ADDR bit in SR1 by reading the SR2
		uint32_t dummyRead=0;
		dummyRead= I2C_handle->I2Cx->SR2;
		(void)dummyRead;
		for(uint32_t i=length;i>0;i--)
		{
			//Wait until RxNE is set
			while(!(I2C_handle->I2Cx->SR1 & (0x1<<I2C_SR1_RXNE)));
			if(i==2)
			{
				//Clear the ACK bit
				I2C_handle->I2Cx->CR1 &=~ (0x1<<I2C_CR1_ACK);
				//Generate the STOP condition
				if(!rs)
				{
					I2C_handle->I2Cx->CR1 |= (0x1<<I2C_CR1_STOP);
				}
			}
			*RXbuffer= I2C_handle->I2Cx->DR;
			RXbuffer++;
		}
	}
	//Re-enable the ACK bit
	if(I2C_handle->I2C_config.I2C_Ack==ENABLE)
	{
		I2C_handle->I2Cx->CR1 |= (0x1<<I2C_CR1_ACK);
	}
}

uint8_t I2C_MasterReceiveDataIT(I2C_handle_t* I2C_handle, uint8_t *RxBuffer, uint32_t length, uint8_t slaveAddress, uint8_t rs)
{
	uint8_t I2Cstate= I2C_handle->RxTxState;
	//Every I2C communication should be established only when the device is not busy
	if((I2Cstate != I2C_RX_BUSY) && (I2Cstate != I2C_TX_BUSY))
	{
		//The bus is busy
		I2C_handle->RxTxState=I2C_RX_BUSY;
		I2C_handle->RxBuffer=RxBuffer;
		I2C_handle->RxLength=length;
		I2C_handle->rs=rs;
		I2C_handle->SlaveAddress=slaveAddress;
		I2C_handle->RxSize=length;

		//Generate the start condition
		I2C_handle->I2Cx->CR1 |= (0x1<<I2C_CR1_START);

		//Enable the transmission event and error related interrupts
		I2C_handle->I2Cx->CR2 |= (0x1<<I2C_CR2_ITBUFEN);
		I2C_handle->I2Cx->CR2 |= (0x1<<I2C_CR2_ITEVTEN);
		I2C_handle->I2Cx->CR2 |= (0x1<<I2C_CR2_ITERREN);
	}
	return I2Cstate;
}

void I2C_IRQcnfg(uint8_t IRQ_number, uint8_t ONorOFF)
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
void I2C_IRQpriorityCnfg(uint8_t IRQ_number, uint8_t IRQ_priority)
{
	uint8_t temp1=IRQ_number/4;
	uint8_t temp2=IRQ_number%4;
	*(NVIC_IPR0_BASE_ADDRESS + (4*temp1)) |= ((0x1<<(8*temp2))<<4);
}

void I2C_SlaveApplicationCallback(I2C_Registers_t* I2Cx,uint8_t ENorDI)
{
	if(ENorDI==ENABLE)
	{
		//Enable the transmission event and error related interrupts
		I2Cx->CR2 |= (0x1<<I2C_CR2_ITBUFEN);
		I2Cx->CR2 |= (0x1<<I2C_CR2_ITEVTEN);
		I2Cx->CR2 |= (0x1<<I2C_CR2_ITERREN);
	}
	else
	{
		//Disable the transmission event and error related interrupts
		I2Cx->CR2 &=~ (0x1<<I2C_CR2_ITBUFEN);
		I2Cx->CR2 &=~ (0x1<<I2C_CR2_ITEVTEN);
		I2Cx->CR2 &=~ (0x1<<I2C_CR2_ITERREN);
	}
}


/*
 *
 *NOTE: This API is available for master and slave devices
 */
void I2C_EV_IRQhandle(I2C_handle_t* I2C_handle)
{
	uint8_t dummyRead=0;
	//Interrupt generated by start event (Only applicable in Master mode)
	uint32_t temp1=I2C_handle->I2Cx->CR2 & (0x1<<I2C_CR2_ITEVTEN);
	uint32_t temp2=I2C_handle->I2Cx->CR2 & (0x1<<I2C_CR2_ITBUFEN);
	uint32_t temp3=I2C_handle->I2Cx->SR1 & (0x1<<I2C_SR1_SB);
	if(temp1 && temp3)
	{
		//SB flag is set: Start condition generated successfully
		//Start address phase
		if(I2C_handle->RxTxState==I2C_TX_BUSY)
		{
			I2C_handle->I2Cx->DR= (I2C_handle->SlaveAddress<<1) & (~(0x1));
		}
		else if(I2C_handle->RxTxState==I2C_RX_BUSY)
		{
			I2C_handle->I2Cx->DR= (I2C_handle->SlaveAddress<<1) | (0x1);
		}
	}
	//Interrupt generated by address event (sent for Master or matched with own address for Slave)
	temp3= (I2C_handle->I2Cx->SR1 & (0x1<<I2C_SR1_ADDR))>>1;
	if(temp1 && temp3)
	{
		//ADDR flag is set
		//Check for device mode
		if(I2C_handle->I2Cx->SR2 & 0x1)
		{
			//Master mode
			if (I2C_handle->RxTxState==I2C_RX_BUSY)
			{
				if(I2C_handle->RxSize==1)
				{
					//Disable ACKing
					I2C_handle->I2Cx->CR1 &= ~(0x1<<I2C_CR1_ACK);
					//Clear ADDR flag
					dummyRead= I2C_handle->I2Cx->SR1;
					dummyRead= I2C_handle->I2Cx->SR2;
					(void) dummyRead;
				}
			}
			else
			{
				dummyRead= I2C_handle->I2Cx->SR1;
				dummyRead= I2C_handle->I2Cx->SR2;
				(void) dummyRead;
			}
		}
		else
		{
			//Slave mode
			dummyRead= I2C_handle->I2Cx->SR1;
			dummyRead= I2C_handle->I2Cx->SR2;
			(void) dummyRead;
		}
	}
	//Interrupt generated by BTF event
	temp3= (I2C_handle->I2Cx->SR1 & (0x1<<I2C_SR1_BTF))>>2;
	if(temp1 && temp3)
	{
		if(I2C_handle->RxTxState==I2C_TX_BUSY)
		{
			//BTF flag is set
			if(I2C_handle->I2Cx->SR1 & (0x1<<I2C_SR1_TXE))
			{
				if(I2C_handle->TxLength==0)
				{
					if(I2C_handle->rs==DISABLE)
					{
						//Generate STOP condition
						I2C_handle->I2Cx->CR1 |= (0x1<<I2C_CR1_STOP);
					}
					//Reset the handle member elements
					I2C_handle->I2Cx->CR2 &=~ (0x1<<I2C_CR2_ITEVTEN);
					I2C_handle->I2Cx->CR2 &=~ (0x1<<I2C_CR2_ITBUFEN);

					I2C_handle->RxTxState=I2C_READY;
					I2C_handle->TxBuffer=NULL;
					I2C_handle->TxLength=0;

					//Notify the application about transmission complete
					I2C_ApplicationEventCallback(I2C_handle,I2C_EVT_TX_COMPLETE);
				}
			}
		}
		else if(I2C_handle->RxTxState==I2C_RX_BUSY)
		{
			;
		}
	}
	//Interrupt generated by STOPF event (Only applicable in Slave mode)
	temp3= I2C_handle->I2Cx->SR1 & (0x1<<I2C_SR1_STOPF);
	if(temp1 && temp3)
	{
		//STOPF flag is set
		//To clear the flag, we need to write in the CR1 register
		I2C_handle->I2Cx->CR1 |= 0x0;		//Dummy write to CR1
		//Notify the application about stop event
		I2C_ApplicationEventCallback(I2C_handle,I2C_EVT_STOP);
	}
	//Interrupt generated by TXE event (TX empty)
	temp3= (I2C_handle->I2Cx->SR1 & (0x1<<I2C_SR1_TXE))>>7;
	if(temp1 && temp2 && temp3)
	{
		//Check the device mode
		if(I2C_handle->I2Cx->SR2 & (0x1))
		{
			//Master mode
			//TXE flag is set
			if(I2C_handle->RxTxState==I2C_TX_BUSY)
			{
				if(I2C_handle->TxLength>0)
				{
					I2C_handle->I2Cx->DR = *(I2C_handle->TxBuffer);
					I2C_handle->TxLength--;
					I2C_handle->TxBuffer++;
				}
			}
		}
		else
		{
			//Slave mode
			if(I2C_handle->I2Cx->SR2 & (0x1<<I2C_SR2_TRA))
			{
				//Slave transmitting
				I2C_ApplicationEventCallback(I2C_handle, I2C_EVT_SLAVE_SEND);
			}
		}
	}
	//Interrupt generated by RXNE event (RX not empty)
	temp3= (I2C_handle->I2Cx->SR1 & (0x1<<I2C_SR1_RXNE))>>6;
	if(temp1 && temp2 && temp3)
	{
		//Check the device mode
		if(I2C_handle->I2Cx->SR2 & (0x1))
		{
			//Master mode
			//RXNE flag is set
			if(I2C_handle->RxTxState==I2C_RX_BUSY)
			{
				if(I2C_handle->RxSize==1)
				{
					*I2C_handle->RxBuffer=I2C_handle->I2Cx->DR;
					I2C_handle->RxLength--;
				}
				if(I2C_handle->RxSize > 1)
				{
					if(I2C_handle->RxLength==2)
					{
						//Clear the ACK bit
						I2C_handle->I2Cx->CR1 &=~ (0x1<<I2C_CR1_ACK);
					}
					*I2C_handle->RxBuffer= I2C_handle->I2Cx->DR;
					I2C_handle->RxBuffer++;
					I2C_handle->RxLength--;
				}
				if(I2C_handle->RxLength==0)
				{
					//Close I2C data reception and notify the application
					if(I2C_handle->rs==DISABLE)
					{
						//Generate STOP condition
						I2C_handle->I2Cx->CR1 |= (0x1<<I2C_CR1_STOP);
					}
					//Reset the handle member elements
					I2C_handle->I2Cx->CR2 &=~ (0x1<<I2C_CR2_ITEVTEN);
					I2C_handle->I2Cx->CR2 &=~ (0x1<<I2C_CR2_ITBUFEN);

					I2C_handle->RxBuffer=NULL;
					I2C_handle->RxLength=0;
					I2C_handle->RxSize=0;
					I2C_handle->RxTxState=I2C_READY;

					//Re-enable the ACK bit
					if(I2C_handle->I2C_config.I2C_Ack==ENABLE)
					{
						I2C_handle->I2Cx->CR1 |= (0x1<<I2C_CR1_ACK);
					}

					//Notify the application about transmission complete
					I2C_ApplicationEventCallback(I2C_handle,I2C_EVT_RX_COMPLETE);
				}
			}
		}
		else
		{
			//Slave mode
			if(!(I2C_handle->I2Cx->SR2 & (0x1<<I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(I2C_handle, I2C_EVT_SLAVE_RECEIVE);
			}
		}
	}
}

void I2C_ERR_IRQhandle(I2C_handle_t* I2C_handle)
{
	uint32_t temp1=0;
	uint32_t temp2=0;
	temp1= (I2C_handle->I2Cx->CR2 & (0x1<<I2C_CR2_ITERREN))>>8;

	//Interrupt generated by bus error
	temp2= I2C_handle->I2Cx->SR1 & (0x1<<I2C_SR1_BERR);
	if(temp1 && temp2)
	{
		//Bus error flag is set
		//Clear the bus error flag
		I2C_handle->I2Cx->SR1 &=~ (0x1<<I2C_SR1_BERR);
		//Notify the application about the error
		I2C_ApplicationEventCallback(I2C_handle,I2C_ERR_BUS);
	}

	//Interrupt generated by arbitration loss error
	temp2= I2C_handle->I2Cx->SR1 & (0x1<<I2C_SR1_ARLO);
	if(temp1 && temp2)
	{
		//ARLO error flag is set
		//Clear the arbitration loss flag
		I2C_handle->I2Cx->SR1 &=~ (0x1<<I2C_SR1_ARLO);
		//Notify the application about the error
		I2C_ApplicationEventCallback(I2C_handle,I2C_ERR_ARLO);
	}

	//Interrupt generated by acknowledge failure
	temp2= (I2C_handle->I2Cx->SR1 & (0x1<<I2C_SR1_AF))>>10;
	if(temp1 && temp2)
	{
		//AF error flag is set
		//Clear the acknowledge failure flag
		I2C_handle->I2Cx->SR1 &=~ (0x1<<I2C_SR1_AF);
		//Notify the application about the error
		I2C_ApplicationEventCallback(I2C_handle,I2C_ERR_AF);
	}

	//Interrupt generated by overrun/underrun error
	temp2= I2C_handle->I2Cx->SR1 & (0x1<<I2C_SR1_OVR);
	if(temp1 && temp2)
	{
		//OVR error flag is set
		//Clear the overrun/underrun flag
		I2C_handle->I2Cx->SR1 &=~ (0x1<<I2C_SR1_OVR);
		//Notify the application about the error
		I2C_ApplicationEventCallback(I2C_handle,I2C_ERR_OVR);
	}

	//Interrupt generated by timeout error
	temp2= I2C_handle->I2Cx->SR1 & (0x1<<I2C_SR1_TIMEOUT);
	if(temp1 && temp2)
	{
		//timeout error flag is set
		//Clear the timeout flag
		I2C_handle->I2Cx->SR1 &=~ (0x1<<I2C_SR1_TIMEOUT);
		//Notify the application about the error
		I2C_ApplicationEventCallback(I2C_handle,I2C_ERR_TIMEOUT);
	}
}


