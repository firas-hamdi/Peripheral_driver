/*
 * STM32F407xx_GPIO.c
 *
 *  Created on: May 13, 2020
 *      Author: hp
 */

#include "STM32F407xx_GPIO_driver.h"


//GPIO peripheral clock setup
/*Enable or disable the GPIO peripheral clock
 *@param1: The base address of the GPIO peripheral
 *@param2: ENABLE macro to enable or DISABLE macro to disable*/
void GPIO_clockControl(GPIO_Registers_t* GPIOx, uint8_t ONorOFF)
{
	if (ONorOFF==ENABLE)
	{
		if(GPIOx==GPIOA)
		{
			GPIOA_CLOCK_ENABLE();
		}
		else if(GPIOx==GPIOB)
		{
			GPIOB_CLOCK_ENABLE();
		}
		else if(GPIOx==GPIOC)
		{
			GPIOC_CLOCK_ENABLE();
		}
		else if(GPIOx==GPIOD)
		{
			GPIOD_CLOCK_ENABLE();
		}
		else if(GPIOx==GPIOE)
		{
			GPIOE_CLOCK_ENABLE();
		}
		else if(GPIOx==GPIOF)
		{
			GPIOF_CLOCK_ENABLE();
		}
		else if(GPIOx==GPIOG)
		{
			GPIOG_CLOCK_ENABLE();
		}
		else if(GPIOx==GPIOH)
		{
			GPIOH_CLOCK_ENABLE();
		}
		else if(GPIOx==GPIOI)
		{
			GPIOI_CLOCK_ENABLE();
		}
		else if(GPIOx==GPIOJ)
		{
			GPIOJ_CLOCK_ENABLE();
		}
		else if(GPIOx==GPIOK)
		{
			GPIOK_CLOCK_ENABLE();
		}
	}
	else
	{
		if(GPIOx==GPIOA)
		{
			GPIOA_CLOCK_DISABLE();
		}
		else if(GPIOx==GPIOB)
		{
			GPIOB_CLOCK_DISABLE();
		}
		else if(GPIOx==GPIOC)
		{
			GPIOC_CLOCK_DISABLE();
		}
		else if(GPIOx==GPIOD)
		{
			GPIOD_CLOCK_DISABLE();
		}
		else if(GPIOx==GPIOE)
		{
			GPIOE_CLOCK_DISABLE();
		}
		else if(GPIOx==GPIOF)
		{
			GPIOF_CLOCK_DISABLE();
		}
		else if(GPIOx==GPIOG)
		{
			GPIOG_CLOCK_DISABLE();
		}
		else if(GPIOx==GPIOH)
		{
			GPIOH_CLOCK_DISABLE();
		}
		else if(GPIOx==GPIOI)
		{
			GPIOI_CLOCK_DISABLE();
		}
		else if(GPIOx==GPIOJ)
		{
			GPIOJ_CLOCK_DISABLE();
		}
		else if(GPIOx==GPIOK)
		{
			GPIOK_CLOCK_DISABLE();
		}
	}
}

//GPIO peripheral initialization
/*Initialize the GPIO peripheral according to the user's input
 *@param1: The handle structure containing the GPIO peripheral address and the configuration parameters
 *Note: Must be called after enabling the peripheral clock @GPIO_clockControl*/
void GPIO_init(GPIO_handle_t* GPIO_handlex)
{
	uint8_t temp=0;
	if(GPIO_handlex->GPIO_pin_cfg.pinMode<=3)
	{
		//Configure the GPIOx PINy mode
		temp= GPIO_handlex->GPIO_pin_cfg.pinMode;
		GPIO_handlex->GPIOx->MODER &=~ (0x3<< 2*(GPIO_handlex->GPIO_pin_cfg.pinNumber)); //clearing bits
		GPIO_handlex->GPIOx->MODER |= (temp << 2*(GPIO_handlex->GPIO_pin_cfg.pinNumber));//setting bits
	}
	else
	{
		GPIO_handlex->GPIOx->MODER &=~(0x3<<2*(GPIO_handlex->GPIO_pin_cfg.pinNumber));
		if(GPIO_handlex->GPIO_pin_cfg.pinMode==INPUT_INTERRUPT_RET)
		{
			EXTI->RTSR |= (0x1<<GPIO_handlex->GPIO_pin_cfg.pinNumber);
			EXTI->FTSR &=~ (0x1<<GPIO_handlex->GPIO_pin_cfg.pinNumber);
		}
		else if(GPIO_handlex->GPIO_pin_cfg.pinMode==INPUT_INTERRUPT_FET)
		{
			EXTI->FTSR |= (0x1<<GPIO_handlex->GPIO_pin_cfg.pinNumber);
			EXTI->RTSR &=~ (0x1<<GPIO_handlex->GPIO_pin_cfg.pinNumber);
		}
		else if(GPIO_handlex->GPIO_pin_cfg.pinMode==INPUT_INTERRUPT_FRET)
		{
			EXTI->FTSR |= (0x1<<GPIO_handlex->GPIO_pin_cfg.pinNumber);
			EXTI->RTSR |= (0x1<<GPIO_handlex->GPIO_pin_cfg.pinNumber);
		}
		SYSCFG_CLOCK_ENABLE();
		uint8_t temp1= GPIO_handlex->GPIO_pin_cfg.pinNumber/4;
		uint8_t temp2= GPIO_handlex->GPIO_pin_cfg.pinNumber%4;
		if(GPIO_handlex->GPIOx==GPIOA)
		{
			SYSCFG->EXTICR[temp1] &=~ (0xF<<(temp2*4));
		}
		else if(GPIO_handlex->GPIOx==GPIOB)
		{
			SYSCFG->EXTICR[temp1] |= (0x1<<(temp2*4));
		}
		else if(GPIO_handlex->GPIOx==GPIOC)
		{
			SYSCFG->EXTICR[temp1] |= (0x2<<(temp2*4));
		}
		else if(GPIO_handlex->GPIOx==GPIOD)
		{
			SYSCFG->EXTICR[temp1] |= (0x3<<(temp2*4));
		}
		else if(GPIO_handlex->GPIOx==GPIOE)
		{
			SYSCFG->EXTICR[temp1] |= (0x4<<(temp2*4));
		}
		else if(GPIO_handlex->GPIOx==GPIOF)
		{
			SYSCFG->EXTICR[temp1] |= (0x5<<(temp2*4));
		}
		else if(GPIO_handlex->GPIOx==GPIOG)
		{
			SYSCFG->EXTICR[temp1] |= (0x6<<(temp2*4));
		}
		else if(GPIO_handlex->GPIOx==GPIOH)
		{
			SYSCFG->EXTICR[temp1] |= (0x7<<(temp2*4));
		}
		else if(GPIO_handlex->GPIOx==GPIOI)
		{
			SYSCFG->EXTICR[temp1] |= (0x8<<(temp2*4));
		}
		EXTI->IMR |= (1<<GPIO_handlex->GPIO_pin_cfg.pinNumber);
	}
	temp=0;
	//Configure the GPIOx PINy output speed
	temp= GPIO_handlex->GPIO_pin_cfg.pinOutputSpeed;
	GPIO_handlex->GPIOx->OSPEEDR &=~ (0x3<< 2*(GPIO_handlex->GPIO_pin_cfg.pinNumber));//clearing bits
	GPIO_handlex->GPIOx->OSPEEDR |= (temp << 2*(GPIO_handlex->GPIO_pin_cfg.pinNumber));//setting bits
	temp=0;
	//Configure the GPIOx PINy output type
	temp= GPIO_handlex->GPIO_pin_cfg.pinOutputType;
	GPIO_handlex->GPIOx->OTYPER &=~ (0x1<< (GPIO_handlex->GPIO_pin_cfg.pinNumber));   //clearing bits
	GPIO_handlex->GPIOx->OTYPER |= (temp << (GPIO_handlex->GPIO_pin_cfg.pinNumber));  //setting bits
	temp=0;
	//Configure the GPIOx PINy pull-up or pull-down or none
	temp= GPIO_handlex->GPIO_pin_cfg.pinPUPD;
	GPIO_handlex->GPIOx->PUPDR &=~ (0x3<< 2*(GPIO_handlex->GPIO_pin_cfg.pinNumber)); //clearing bits
	GPIO_handlex->GPIOx->PUPDR |= (temp << 2*(GPIO_handlex->GPIO_pin_cfg.pinNumber));//settings bits
	temp=0;
	//Configure the GPIOx PINy alternate functionality
	if (GPIO_handlex->GPIO_pin_cfg.pinMode==AF_MODE)
	{
		if(GPIO_handlex->GPIO_pin_cfg.pinNumber<8)
		{
			temp= GPIO_handlex->GPIO_pin_cfg.pinAltFunMode;
			GPIO_handlex->GPIOx->AFLR &=~ (0xF<< 4*(GPIO_handlex->GPIO_pin_cfg.pinNumber)); //clearing bits
			GPIO_handlex->GPIOx->AFLR |= (temp<<4*(GPIO_handlex->GPIO_pin_cfg.pinNumber));  //setting bits
			temp=0;
		}
		else
		{
			temp= GPIO_handlex->GPIO_pin_cfg.pinAltFunMode;
			GPIO_handlex->GPIOx->AFHR &=~ (0xF<<4*(GPIO_handlex->GPIO_pin_cfg.pinNumber%8));  //clearing bits
			GPIO_handlex->GPIOx->AFHR |= (temp<<4*(GPIO_handlex->GPIO_pin_cfg.pinNumber%8));  //setting bits
			temp=0;
		}
	}
}


//GPIO peripheral reset
/*De-init the GPIO peripheral registers
 *@param1: The base address of the GPIO peripheral*/
void GPIO_reset(GPIO_Registers_t* GPIOx)
{
	//The macro @GPIO_REGISTER_RESET could be called instead of the two instructions
	if(GPIOx==GPIOA)
	{
		RCC->AHB1RSTR |= (0x1); //Set the reset bit of GPIOA (reset state)
		RCC->AHB1RSTR &=~(0x1); //Clear the reset bit of GPIOA
	}
	else if(GPIOx==GPIOB)
	{
		RCC->AHB1RSTR |= (0x1<<1); //Set the reset bit of GPIOB (reset state)
		RCC->AHB1RSTR &=~(0x1<<1); //Clear the reset bit of GPIOB
	}
	else if(GPIOx==GPIOC)
	{
		RCC->AHB1RSTR |= (0x1<<2); //Set the reset bit of GPIOC (reset state)
		RCC->AHB1RSTR &=~(0x1<<2); //Clear the reset bit of GPIOC
	}
	else if(GPIOx==GPIOD)
	{
		RCC->AHB1RSTR |= (0x1<<3); //Set the reset bit of GPIOD (reset state)
		RCC->AHB1RSTR &=~(0x1<<3); //Clear the reset bit of GPIOD
	}
	else if(GPIOx==GPIOE)
	{
		RCC->AHB1RSTR |= (0x1<<4); //Set the reset bit of GPIOE (reset state)
		RCC->AHB1RSTR &=~(0x1<<4); //Clear the reset bit of GPIOE
	}
	else if(GPIOx==GPIOF)
	{
		RCC->AHB1RSTR |= (0x1<<5); //Set the reset bit of GPIOF (reset state)
		RCC->AHB1RSTR &=~(0x1<<5); //Clear the reset bit of GPIOF
	}
	else if(GPIOx==GPIOG)
	{
		RCC->AHB1RSTR |= (0x1<<6); //Set the reset bit of GPIOG (reset state)
		RCC->AHB1RSTR &=~(0x1<<6); //Clear the reset bit of GPIOG
	}
	else if(GPIOx==GPIOH)
	{
		RCC->AHB1RSTR |= (0x1<<7); //Set the reset bit of GPIOH (reset state)
		RCC->AHB1RSTR &=~(0x1<<7); //Clear the reset bit of GPIOH
	}
	else if(GPIOx==GPIOI)
	{
		GPIOI_REGISTER_RESET();
	}
	else if(GPIOx==GPIOJ)
	{
		GPIOJ_REGISTER_RESET();
	}
	else if(GPIOx==GPIOK)
	{
		GPIOK_REGISTER_RESET();
	}
}

// Read data from input pin
/*@INparam1: The GPIO peripheral base address
 *@INparam2: The pin number to read data from
 *@Outparam: The pin value
 */
uint8_t GPIO_readInputPin(GPIO_Registers_t* GPIOx,uint8_t pinNumberToRead)
{
	uint8_t data=0;
	data= (GPIOx->IDR &(0x1<<pinNumberToRead))>>pinNumberToRead;
	return data;
}

// Read data from input port
/*@INparam: the base address of the GPIO peripheral
 *@OUTparam: The port value
 */
uint16_t GPIO_readInputPort(GPIO_Registers_t* GPIOx)
{
	uint16_t data=0;
	data=GPIOx->IDR & (0xFFFF);
	return data;
}

// Write data at the GPIO output data register bit field corresponding to the pin number
/*@INparam1: The GPIO peripheral base address
 *@INparam2: The pin number where the data will be written
 *@INparam3: The value to write
 */
void GPIO_writeOutputPin(GPIO_Registers_t* GPIOx,uint8_t pinNumberToWrite, uint8_t HIorLO)
{
	if(HIorLO==HIGH)
	{
		GPIOx->ODR |=(0x1<<pinNumberToWrite) ;
	}
	else if(HIorLO==LOW)
	{
		GPIOx->ODR &=~(0x1<<pinNumberToWrite);
	}
}

// Write data at the GPIO output data register
/*@INparam1: The GPIO peripheral base address
 *@INparam2: The value to write
 */
void GPIO_writeOutputPort(GPIO_Registers_t* GPIOx, uint16_t value)
{
		GPIOx->ODR |=(value) ;
}

// Toggle a GPIO port pin
/*@INparam1: the base address of the GPIO peripheral
 *@INparam2: the pin number to toggle
 */
void GPIO_toggleOutputPin(GPIO_Registers_t* GPIOx, uint8_t pinNumberToToggle)
{
	GPIOx->ODR^=(0x1<<pinNumberToToggle);
}

//IRQ configuration
/*@INparam1: the IRQ number
 *@INparam2: enable or disable the interruption
 */
void GPIO_IRQcnfg(uint8_t IRQ_number, uint8_t ONorOFF)
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
void GPIO_IRQpriorityCnfg(uint8_t IRQ_number, uint8_t IRQ_priority)
{
	uint8_t temp1=IRQ_number/4;
	uint8_t temp2=IRQ_number%4;
	*(NVIC_IPR0_BASE_ADDRESS + (4*temp1)) |= ((0x1<<(8*temp2))<<4);
}

//IRQ handler
/*@INparam: the pin number from which the IRQ is triggered
 * Called in the application layer by the the ISR implementation
 */
void GPIO_IRQhandler(uint8_t pinNumber)
{
	if(EXTI->PR &= (0x1<<pinNumber))
	{
		EXTI->PR |= (0x1<<pinNumber);
	}
}
