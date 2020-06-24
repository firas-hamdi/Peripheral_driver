/*
 * STM32F407xx_GPIO.h
 *
 *  Created on: May 13, 2020
 *      Author: hp
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "STM32F407xx.h"

// Structure configuration for a GPIO pin
typedef struct
{
	uint8_t pinNumber;
	uint8_t pinMode;					//Possible values @GPIO pins modes definition
	uint8_t pinOutputSpeed;				//Possible speed values @GPIO pins output speed definition
	uint8_t pinPUPD;					//Possible pull_up/Pull_down values @GPIO pins pull-up/pull-down configuration definition
	uint8_t pinOutputType;				//Possible output types values @GPIO pins output types definition
	uint8_t pinAltFunMode;				//Possible alternate function values @GPIO pins alternate function selection
}GPIO_pin_cfg_t;


// Handle structure for a GPIO pin
typedef struct
{
	GPIO_Registers_t* GPIOx;                 //Base address of the GPIO port
	GPIO_pin_cfg_t    GPIO_pin_cfg;          // GPIO pin configuration settings

}GPIO_handle_t;

// GPIO pins modes definition
#define INPUT_MODE    		 0
#define OUTPUT_MODE    		 1
#define AF_MODE        		 2                     //Alternate Function mode
#define ANALOG_MODE    		 3
#define INPUT_INTERRUPT_FET  4                     //Interrupt falling edge trigger
#define INPUT_INTERRUPT_RET  5                     //Interrupt raising edge trigger
#define INPUT_INTERRUPT_FRET 6                     //Interrupt falling and raising edge trigger

// GPIO pins output types definition
#define PUSH_PULL_OUTPUT    0
#define OPEN_DRAIN_OUTPUT   1

//GPIO pins output speed definition
#define LOW_SPEED           0
#define MEDIUM_SPEED		1
#define HIGH_SPEED			2
#define VERY_HIGH_SPEED		3

//GPIO pins pull-up/pull-down configuration definition
#define NO_PULL_UP_NO_PULL_DOWN			0
#define PULL_UP							1
#define PULL_DOWN						2

//GPIO pins alternate function selection
#define ALTERNATE_FUNCTION_0            0
#define ALTERNATE_FUNCTION_1            1
#define ALTERNATE_FUNCTION_2            2
#define ALTERNATE_FUNCTION_3            3
#define ALTERNATE_FUNCTION_4            4
#define ALTERNATE_FUNCTION_5            5
#define ALTERNATE_FUNCTION_6            6
#define ALTERNATE_FUNCTION_7            7
#define ALTERNATE_FUNCTION_8            8
#define ALTERNATE_FUNCTION_9            9
#define ALTERNATE_FUNCTION_10           10
#define ALTERNATE_FUNCTION_11           11
#define ALTERNATE_FUNCTION_12           12
#define ALTERNATE_FUNCTION_13           13
#define ALTERNATE_FUNCTION_14           14
#define ALTERNATE_FUNCTION_15           15


//*********************Supported APIs by the GPIO*************************//
//GPIO init and reset
void GPIO_init(GPIO_handle_t*);
void GPIO_reset(GPIO_Registers_t*);

//GPIO peripheral clock setup
void GPIO_clockControl(GPIO_Registers_t*, uint8_t);

//GPIO read and write data
uint8_t GPIO_readInputPin(GPIO_Registers_t*,uint8_t);
uint16_t GPIO_readInputPort(GPIO_Registers_t*);
void GPIO_writeOutputPin(GPIO_Registers_t*, uint8_t, uint8_t);
void GPIO_writeOutputPort(GPIO_Registers_t*,uint16_t);
void GPIO_toggleOutputPin(GPIO_Registers_t*,uint8_t);

//IRQ configuration and ISR handling
void GPIO_IRQcnfg(uint8_t, uint8_t);
void GPIO_IRQpriorityCnfg(uint8_t, uint8_t);
void GPIO_IRQhandler(uint8_t);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
