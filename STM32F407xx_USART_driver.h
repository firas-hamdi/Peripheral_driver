/*
 * STM32F407xx_USART_driver.c
 *
 *  Created on: 7 juin 2020
 *      Author: hp
 */

#ifndef INC_STM32F407XX_USART_DRIVER_C_
#define INC_STM32F407XX_USART_DRIVER_C_

#include <stdint.h>
#include "STM32F407xx.h"

/*
 * Configuration structure for USARTx peripheral
 */
typedef struct
{
	uint8_t USART_mode;
	uint8_t USART_wordLen;
	uint32_t USART_baud;
	uint8_t USART_parity;
	uint8_t USART_stopBits;
	uint8_t USART_HWflowControl;
	uint8_t USART_OVER;
}USART_cnfg_t;

/*
 * Handle structure for USARTx peripheral
 */
typedef struct
{
	USART_cnfg_t USART_cfg;
	USART_Registers_t* USARTx;
	//These members are needed for the send/receive based on interrupt methods (Non blocking methods)
	uint32_t TXlength;
	uint32_t RXlength;
	uint32_t RXstate;
	uint32_t TXstate;
	uint8_t* RXbuffer;
	uint8_t* TXbuffer;
}USART_handle_t;

/*
 *@USART_mode possible values
 */
#define USART_MODE_ONLY_TX 0		//Only transmitter
#define USART_MODE_ONLY_RX 1		//Only receiver
#define USART_MODE_TXRX    2   		//Transmitter and receiver

/*
 *@USART_baud possible values
 */
#define USART_BAUD_1200					1200
#define USART_BAUD_2400					2400
#define USART_BAUD_9600					9600
#define USART_BAUD_19200 				19200
#define USART_BAUD_38400 				38400
#define USART_BAUD_57600 				57600
#define USART_BAUD_115200 				115200
#define USART_BAUD_230400 				230400
#define USART_BAUD_460800 				460800
#define USART_BAUD_921600 				921600
#define USART_BAUD_2M 					2000000
#define USART_BAUD_3M 					3000000

/*
 *@USART_parity possible values
 */
#define USART_ODD_PARITY_ENABLE   3
#define USART_EVEN_PARITY_ENABLE  1
#define USART_PARITY_DISABLE  	  0

/*
 *@USART_wordLen possible values
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_stopBits possible values
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWflowControl possible values
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_RTS    	1
#define USART_HW_FLOW_CTRL_CTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
 *@USART_OVER possible values
 */
#define USART_OVER_16				0
#define USART_OVER_8				1

//USART based interrupt possible states
#define USART_READY					 0
#define USART_RX_BUSY				 1
#define USART_TX_BUSY				 2

//USART application callback events
#define USART_EVT_RX_COMPLETE		 0
#define USART_EVT_TX_COMPLETE		 1

//USARTx peripheral clock control
void USART_clockControl(USART_Registers_t*, uint8_t);

//USARTx init and reset
void USART_init(USART_handle_t);
void USART_reset(void);

//USART peripheral: data send and receive: Blocking methods
void USART_dataSend(USART_handle_t*,uint8_t*, uint32_t);
void USART_dataReceive(USART_handle_t*,uint8_t*, uint32_t);

//USARTx peripheral: data send and receive: Interrupt based methods
uint8_t USART_dataSendIT(USART_handle_t*,uint8_t*, uint32_t);
uint8_t USART_dataReceiveIT(USART_handle_t*,uint8_t*, uint32_t);

//USARTx peripheral control
void USART_PeriControl(USART_Registers_t*, uint8_t);

//IRQ configuration
void USART_IRQcnfg(uint8_t, uint8_t);
void USART_IRQpriorityCnfg(uint8_t, uint8_t);
void USART_IRQhandle(USART_handle_t*);

//Event application callback
__attribute__((weak)) void USART_ApplicationEventCallback(USART_handle_t *,uint8_t);



#endif /* INC_STM32F407XX_USART_DRIVER_C_ */
