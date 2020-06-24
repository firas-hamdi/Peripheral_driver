/*
 * STM32F407xx_I2C_driver.h
 *
 *  Created on: 23 mai 2020
 *      Author: hp
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "STM32F407xx.h"

typedef struct
{
	uint32_t I2C_SCL;
	uint8_t I2C_deviceAddress;
	uint8_t I2C_Ack;
	uint8_t i2C_FMDutyCycle;
}I2C_cnfg_t;

typedef struct
{
	I2C_cnfg_t I2C_config;
    I2C_Registers_t* I2Cx;
    uint32_t TxLength;			//TX buffer length
    uint32_t RxLength;			//RX buffer length
    uint8_t* TxBuffer;			//TX buffer address
    uint8_t* RxBuffer;			//RX buffer address
    uint8_t SlaveAddress;		//Device address
    uint8_t rs;					//Repeated start value
    uint32_t RxSize;			//RX size
    uint8_t RxTxState;			//Communication state
}I2C_handle_t;

// @I2C_SCL possible values
#define I2C_SCL_SM_100Khz		100000UL			//SCl=100KHz; I2C mode: Standard mode
#define I2C_SCL_FM_200Khz		200000UL			//SCL=200KHz; I2C mode: Fast mode
#define I2C_SCL_FM_400Khz		400000UL			//SCL=400KHz; I2C mode: Fast mode

// @I2C_Ack possible values
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

// @I2C_FMDutyCycle possible values
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1

// @RxTxState possible values
#define I2C_READY				0					//I2C peripheral ready for communication
#define I2C_RX_BUSY				1					//I2C peripheral busy in RX
#define I2C_TX_BUSY				2					//I2C peripheral busy in TX

//I2C init and reset
void I2C_init(I2C_handle_t*);
void I2C_reset(I2C_Registers_t*);

//I2C peripheral clock setup
void I2C_clockControl(I2C_Registers_t*, uint8_t);

//I2C peripheral control
void I2C_PeriControl(I2C_Registers_t*, uint8_t);

//I2C master send/receive data
void I2C_MasterSendData(I2C_handle_t*, uint8_t *, uint32_t, uint8_t, uint8_t);
void I2C_MasterReceiveData(I2C_handle_t*, uint8_t *, uint32_t, uint8_t, uint8_t);
void I2C_SlaveSendData(I2C_Registers_t*,uint8_t);
uint8_t I2C_SlaveReceiveData(I2C_Registers_t*);


uint8_t I2C_MasterSendDataIT(I2C_handle_t*, uint8_t *, uint32_t, uint8_t, uint8_t);
uint8_t I2C_MasterReceiveDataIT(I2C_handle_t*, uint8_t *, uint32_t, uint8_t, uint8_t);

//IRQ configuration and ISR handling
void I2C_IRQcnfg(uint8_t, uint8_t);
void I2C_IRQpriorityCnfg(uint8_t, uint8_t);
void I2C_EV_IRQhandle(I2C_handle_t*);
void I2C_ERR_IRQhandle(I2C_handle_t*);

//Application callback
void I2C_ApplicationEventCallback(I2C_handle_t*,uint8_t);
void I2C_SlaveApplicationCallback(I2C_Registers_t*,uint8_t);

//I2C application event macros
#define I2C_EVT_TX_COMPLETE			0
#define I2C_EVT_RX_COMPLETE			1
#define I2C_EVT_STOP				2
#define I2C_EVT_ADDR_COMPLETE		3
#define I2C_EVT_SLAVE_SEND			4
#define I2C_EVT_SLAVE_RECEIVE		5
#define I2C_ERR_BUS					6
#define I2C_ERR_ARLO				7
#define I2C_ERR_AF					8
#define I2C_ERR_OVR					9
#define I2C_ERR_TIMEOUT				10

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
