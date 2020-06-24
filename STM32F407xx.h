/*
 * STM32F407xx.h
 *
 *  Created on: May 12, 2020
 *      Author: hp
 */
#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

//NVIC base address definition
#define NVIC_BASE_ADDRESS    (0xE000E100UL)

//NVIC Interrupt set enable registers base address definition
#define NVIC_ISER0_BASE_ADDRESS        ((volatile uint32_t*) NVIC_BASE_ADDRESS)
#define NVIC_ISER1_BASE_ADDRESS        ((volatile uint32_t*) (NVIC_BASE_ADDRESS+(0x04)))
#define NVIC_ISER2_BASE_ADDRESS        ((volatile uint32_t*) (NVIC_BASE_ADDRESS+(0x08)))
#define NVIC_ISER3_BASE_ADDRESS        ((volatile uint32_t*) (NVIC_BASE_ADDRESS+(0x0C)))

//NVIC Interrupt clear enable registers base address definition
#define NVIC_ICER0_BASE_ADDRESS        ((volatile uint32_t*)(NVIC_BASE_ADDRESS+(0x80)))
#define NVIC_ICER1_BASE_ADDRESS        ((volatile uint32_t*)(NVIC_BASE_ADDRESS+(0x84)))
#define NVIC_ICER2_BASE_ADDRESS        ((volatile uint32_t*)(NVIC_BASE_ADDRESS+(0x88)))
#define NVIC_ICER3_BASE_ADDRESS        ((volatile uint32_t*)(NVIC_BASE_ADDRESS+(0x8C)))

//NVIC interrupt priority registers base address definition
#define NVIC_IPR0_BASE_ADDRESS        ((volatile uint32_t*)(NVIC_BASE_ADDRESS+(0x0300)))
#define NVIC_IPR1_BASE_ADDRESS        ((volatile uint32_t*)(NVIC_BASE_ADDRESS+(0x0304)))
#define NVIC_IPR2_BASE_ADDRESS        ((volatile uint32_t*)(NVIC_BASE_ADDRESS+(0x0308)))

//*******************Memory and peripherals base address definition***************************//
//Memory addresses definition
#define FLASH_BASE_ADDRESS   (0x80000000UL)   //Base address of flash memory
#define SRAM1_BASE_ADDRESS	 (0x20000000UL)   //Base address of SRAM1 memory
#define SRAM2_BASE_ADDRESS   (0x2001C000UL)   //Base address of SRAM2 memory (SRAM1 base_addr +(0x)(112*1024))
#define SRAM3_BASE_ADDRESS   (0x20020000UL)   //Base address of SRAM3 memory (SRAM2 base_addr +(0x)(16*1024))
#define ROM_BASE_ADDRESS     (0x1FFF0000UL)   //Base address of ROM memory

// Bus domains definition
#define AHB2_PERIPH_BASE     (0x50000000UL)   //Base address of peripherals connected to the AHB2
#define AHB1_PERIPH_BASE     (0x40020000UL)   //Base address of peripherals connected to the AHB1
#define APB2_PERIPH_BASE     (0x40010000UL)   //Base address of peripherals connected to the APB2
#define APB1_PERIPH_BASE     (0x40000000UL)   //Base address of peripherals connected to the APB1

// GPIO Peripherals base address definition
#define GPIOA_BASE_ADDRESS   ((AHB1_PERIPH_BASE)+(0x0)) //Base address of GPIOA peripheral
#define GPIOB_BASE_ADDRESS   ((AHB1_PERIPH_BASE)+(0x0400)) //Base address of GPIOB peripheral
#define GPIOC_BASE_ADDRESS   ((AHB1_PERIPH_BASE)+(0x0800)) //Base address of GPIOC peripheral
#define GPIOD_BASE_ADDRESS   ((AHB1_PERIPH_BASE)+(0x0C00)) //Base address of GPIOD peripheral
#define GPIOE_BASE_ADDRESS   ((AHB1_PERIPH_BASE)+(0x1000)) //Base address of GPIOE peripheral
#define GPIOF_BASE_ADDRESS   ((AHB1_PERIPH_BASE)+(0x1400)) //Base address of GPIOF peripheral
#define GPIOG_BASE_ADDRESS   ((AHB1_PERIPH_BASE)+(0x1800)) //Base address of GPIOG peripheral
#define GPIOH_BASE_ADDRESS   ((AHB1_PERIPH_BASE)+(0x1C00)) //Base address of GPIOH peripheral
#define GPIOI_BASE_ADDRESS   ((AHB1_PERIPH_BASE)+(0x2000)) //Base address of GPIOI peripheral
#define GPIOJ_BASE_ADDRESS   ((AHB1_PERIPH_BASE)+(0x2400)) //Base address of GPIOJ peripheral
#define GPIOK_BASE_ADDRESS   ((AHB1_PERIPH_BASE)+(0x2800)) //Base address of GPIOK peripheral

//SPI peripherals base address definition
#define SPI1_BASE_ADDRESS    ((APB2_PERIPH_BASE)+(0x3000)) //Base address of SPI1 peripheral
#define SPI2_BASE_ADDRESS    ((APB1_PERIPH_BASE)+(0x3800)) //Base address of SPI2 peripheral
#define SPI3_BASE_ADDRESS    ((APB1_PERIPH_BASE)+(0x3C00)) //Base address of SPI3 peripheral
#define SPI4_BASE_ADDRESS    ((APB2_PERIPH_BASE)+(0x3400)) //Base address of SPI4 peripheral
#define SPI5_BASE_ADDRESS    ((APB2_PERIPH_BASE)+(0x5000)) //Base address of SPI5 peripheral
#define SPI6_BASE_ADDRESS    ((APB2_PERIPH_BASE)+(0x5400)) //Base address of SPI6 peripheral

//EXTI peripheral base address definition
#define EXTI_BASE_ADDRESS    ((APB2_PERIPH_BASE)+(0x3C00)) //Base address of EXTI peripheral

//SYSCFG peripheral base address definition
#define SYSCFG_BASE_ADDRESS  ((APB2_PERIPH_BASE)+(0x3800))  //Base address of SYSCFG peripheral

//RCC peripheral base address definition
#define RCC_BASE_ADDRESS    ((AHB1_PERIPH_BASE)+(0x3800)) //Base address of RCC peripheral

//I2C peripherals base address definition
#define I2C1_BASE_ADDRESS    ((APB1_PERIPH_BASE)+(0x5400)) //Base address of I2C1 peripheral
#define I2C2_BASE_ADDRESS    ((APB1_PERIPH_BASE)+(0x5800)) //Base address of I2C2 peripheral
#define I2C3_BASE_ADDRESS    ((APB1_PERIPH_BASE)+(0x5C00)) //Base address of I2C3 peripheral

//UART/USART peripherals base address definition
#define USART1_BASE_ADDRESS	 ((APB2_PERIPH_BASE)+(0x1000)) //Base address of USART1 peripheral
#define USART2_BASE_ADDRESS	 ((APB1_PERIPH_BASE)+(0x4400)) //Base address of USART2 peripheral
#define USART3_BASE_ADDRESS	 ((APB1_PERIPH_BASE)+(0x4800)) //Base address of USART3 peripheral
#define UART4_BASE_ADDRESS	 ((APB1_PERIPH_BASE)+(0x4C00)) //Base address of UART4 peripheral
#define UART5_BASE_ADDRESS	 ((APB1_PERIPH_BASE)+(0x5000)) //Base address of UART5 peripheral
#define USART6_BASE_ADDRESS	 ((APB2_PERIPH_BASE)+(0x1400)) //Base address of USART6 peripheral
#define UART7_BASE_ADDRESS	 ((APB1_PERIPH_BASE)+(0x7800)) //Base address of UART7 peripheral
#define UART8_BASE_ADDRESS	 ((APB1_PERIPH_BASE)+(0x7C00)) //Base address of UART8 peripheral


//********************Peripheral registers definition******************************//
//SPI1 peripheral registers definition
//specific to SPI1 peripheral
#define SPI2_CR1_BASE_ADDRESS      ((SPI2_BASE_ADDRESS)+(0x00))  //Base address of SPI2 peripheral's control register 1
#define SPI2_CR2_BASE_ADDRESS      ((SPI2_BASE_ADDRESS)+(0x04))  //Base address of SPI2 peripheral's control register 2
#define SPI2_SR_BASE_ADDRESS       ((SPI2_BASE_ADDRESS)+(0x08))  //Base address of SPI2 peripheral's status register
#define SPI2_DR_BASE_ADDRESS       ((SPI2_BASE_ADDRESS)+(0x0C))  //Base address of SPI2 peripheral's data register
#define SPI2_CRCPR_BASE_ADDRESS    ((SPI2_BASE_ADDRESS)+(0x10))  //Base address of SPI2 peripheral's CRC polynomial register
#define SPI2_RXCRC_BASE_ADDRESS    ((SPI2_BASE_ADDRESS)+(0x14))  //Base address of SPI2 peripheral's RX CRC register
#define SPI2_TXCRC_BASE_ADDRESS    ((SPI2_BASE_ADDRESS)+(0x18))  //Base address of SPI2 peripheral's TX CRC register
#define SPI2_I2SCFGR_BASE_ADDRESS  ((SPI2_BASE_ADDRESS)+(0x1C))  //Base address of SPI2 peripheral's I2S configuration register
#define SPI2_I2SPR_BASE_ADDRESS    ((SPI2_BASE_ADDRESS)+(0x20))  //Base address of SPI2 peripheral's I2S prescaler register

//GPIO peripherals structure definition
//Generic to all GPIO peripherals
//The structure includes all registers of the GPIO peripheral
typedef struct
{
	volatile uint32_t MODER ;   // GPIO mode register; Offset=0x00
	volatile uint32_t OTYPER;   // GPIO output type register; Offset=0x04
	volatile uint32_t OSPEEDR;  // GPIO output speed register; Offset=0x08
	volatile uint32_t PUPDR;    // GPIO pull-up/pull-down register; Offset=0x0C
	volatile uint32_t IDR;      // GPIO input data register; Offset=0x10
	volatile uint32_t ODR;      // GPIO output data register; Offset=0x14
	volatile uint32_t BSRR;     // GPIO bit set/reset register; Offset=0x18
	volatile uint32_t LCKR;     // GPIO configuration lock register; Offset=0x1C
	volatile uint32_t AFLR;     // GPIO alternate function low register; Offset=0x20;
	volatile uint32_t AFHR;     // GPIO alternate function high register; Offset=0x24;
}GPIO_Registers_t;

//RCC peripheral structure definition
//The structure includes all registers of the RCC peripheral
typedef struct
{
	volatile uint32_t RCC_CR ;    // RCC control register; Offset=0x00
	volatile uint32_t PLLCFGR;    // RCC PLL clock configuration register; Offset=0x04
	volatile uint32_t CFGR;       // RCC clock configuration register; Offset=0x08
	volatile uint32_t CIR;        // RCC clock interrupt register; Offset=0x0C
	volatile uint32_t AHB1RSTR;   // RCC AHB1 peripheral reset register; Offset=0x10
	volatile uint32_t AHB2RSTR;   // RCC AHB2 peripheral reset register; Offset=0x14
	volatile uint32_t AHB3RSTR;   // RCC AHB3 peripheral reset register; Offset=0x18
	const uint32_t reserved_1;    // Reserved; Offset=0x1C
	volatile uint32_t APB1RSTR;   // RCC APB1 peripheral reset register; Offset=0x20
	volatile uint32_t APB2RSTR;   // RCC APB2 peripheral reset register; Offset=0x24
	const uint32_t reserved_2;    // Reserved; Offset=0x28
	const uint32_t reserved_3;    // Reserved; Offset=0x2C
	volatile uint32_t AHB1ENR;    // RCC AHB1 peripheral enable register; Offset=0x30
	volatile uint32_t AHB2ENR;    // RCC AHB2 peripheral enable register; Offset=0x34
	volatile uint32_t AHB3ENR;    // RCC AHB3 peripheral enable register; Offset=0x38
	const uint32_t reserved_4;    // Reserved; Offset=0x3C
	volatile uint32_t APB1ENR;    // RCC APB1 peripheral enable register; Offset=0x40
	volatile uint32_t APB2ENR;    // RCC APB2 peripheral enable register; Offset=0x44
	const uint32_t reserved_5;    // Reserved; Offset=0x48
	const uint32_t reserved_6;    // Reserved; Offset=0x4C
	volatile uint32_t AHB1LPENR;  // RCC AHB1 peripheral low power mode enable register; Offset=0x50
	volatile uint32_t AHB2LPENR;  // RCC AHB2 peripheral low power mode enable register; Offset=0x54
	volatile uint32_t AHB3LPENR;  // RCC AHB3 peripheral low power mode enable register; Offset=0x58
	const uint32_t reserved_7;    // Reserved; Offset=0x5C
	volatile uint32_t APB1LPENR;  // RCC APB1 peripheral low power mode enable register; Offset=0x60
	volatile uint32_t APB2LPENR;  // RCC APB2 peripheral low power mode enable register; Offset=0x64
	const uint32_t reserved_8;    // Reserved; Offset=0x68
	const uint32_t reserved_9;    // Reserved; Offset=0x6C
	volatile uint32_t BDCR;       // RCC backup domain control register; Offset=0x70
	volatile uint32_t CSR;        // RCC control and status register; Offset=0x74
	const uint32_t reserved_10;   // Reserved; Offset=0x78
	const uint32_t reserved_11;   // Reserved; Offset=0x7C
	volatile uint32_t SSCGR;      // RCC spread spectrum clock generation register; Offset=0x80
	volatile uint32_t PLLI2SCFGR; // RCC PLLI2S configuration register; Offset=0x84
	volatile uint32_t PLLSAICFGR; // RCC PLLI2S configuration register; Offset=0x88
	volatile uint32_t DCKCFGR;    // RCC PLLI2S configuration register; Offset=0x8C
}RCC_Registers_t;

typedef struct
{
	volatile uint32_t IMR ;     // EXTI interrupt mask register; Offset=0x00
	volatile uint32_t EMR;      // EXTI event mask register; Offset=0x04
	volatile uint32_t RTSR;     // EXTI rising edge trigger selection register; Offset=0x08
	volatile uint32_t FTSR;    	// EXTI rising edge trigger selection register; Offset=0x0C
	volatile uint32_t SWIER;    // EXTI software interrupt event register; Offset=0x10
	volatile uint32_t PR;       // EXTI pending register; Offset=0x14
}EXTI_Registers_t;

typedef struct
{
	volatile uint32_t MEMRMP;      // SYSCFG memory re-map register; Offset=0x00
	volatile uint32_t PMC;         // SYSCFG peripheral mode configuration register; Offset=0x04
	volatile uint32_t EXTICR[4];   // SYSCFG external interrupt configuration register1-2-3-4; Offset=0x08-0x0C-0x10-0x14
	volatile uint32_t RESERVED[2]; // Reserved; Offset: 0x18-0x1C
	volatile uint32_t CMPCR;       // SYSCFG compensation cell control register; Offset=0x20
}SYSCFG_Registers_t;

typedef struct
{
	volatile uint32_t CR1;         // I2C control register 1; Offset=0x00
	volatile uint32_t CR2;         // I2C control register 2; Offset=0x04
	volatile uint32_t OAR1;   	   // I2C own address register 1; Offset=0x08
	volatile uint32_t OAR2;   	   // I2C own address register 2; Offset=0x0C
	volatile uint32_t DR; 	       // I2C data register; Offset: 0x10
	volatile uint32_t SR1;         // I2C status register 1; Offset=0x14
	volatile uint32_t SR2;         // I2C status register 2; Offset=0x18
	volatile uint32_t CCR;    	   // I2C clock control register; Offset=0x1C
	volatile uint32_t TRISE;       // I2C TRISE register; Offset=0x20
	volatile uint32_t FLTR;    	   // I2C DLTR register; Offset=0x24
}I2C_Registers_t;

typedef struct
{
	volatile uint32_t SR;		   // USART status register; Offset=0x00
	volatile uint32_t DR;		   // USART data register; Offset=0x04
	volatile uint32_t BRR;		   // USART baud rate register; Offset=0x08
	volatile uint32_t CR1;		   // USART control register 1; Offset=0x0C
	volatile uint32_t CR2;		   // USART control register 2; Offset=0x10;
	volatile uint32_t CR3;		   // USART control register 3; Offset=0x14;
	volatile uint32_t GTPR;		   // USART guard time and prescaler register ; Offset=0x18;
}USART_Registers_t;

//************************EXTI IRQ(interrupt request) numbers*****************************//
#define IRQ_EXTI0         6
#define IRQ_EXTI1         7
#define IRQ_EXTI2         8
#define IRQ_EXTI3         9
#define IRQ_EXTI4         10
#define IRQ_EXTI9_5       23
#define IRQ_EXTI15_10     40

//************************SPI IRQ(interrupt request) numbers*****************************//
#define SPI1_Interrupt        35
#define SPI2_Interrupt        36
#define SPI3_Interrupt        51

//************************I2C IRQ(interrupt request) numbers*****************************//
#define I2C1_EV_Interrupt        31
#define I2C1_ERR_Interrupt       32
#define I2C2_EV_Interrupt        33
#define I2C2_ERR_Interrupt       34
#define I2C3_EV_Interrupt        72
#define I2C3_ERR_Interrupt       73

//************************USART IRQ(interrupt request) numbers*****************************//
#define USART1_Interrupt         37
#define USART2_Interrupt         38
#define USART3_Interrupt         39
#define USART6_Interrupt         71

//************************Peripherals definition******************************//
//GPIO peripheral definition
#define GPIOA    (GPIO_Registers_t*) GPIOA_BASE_ADDRESS
#define GPIOB    (GPIO_Registers_t*) GPIOB_BASE_ADDRESS
#define GPIOC    (GPIO_Registers_t*) GPIOC_BASE_ADDRESS
#define GPIOD    (GPIO_Registers_t*) GPIOD_BASE_ADDRESS
#define GPIOE    (GPIO_Registers_t*) GPIOE_BASE_ADDRESS
#define GPIOF    (GPIO_Registers_t*) GPIOF_BASE_ADDRESS
#define GPIOG    (GPIO_Registers_t*) GPIOG_BASE_ADDRESS
#define GPIOH    (GPIO_Registers_t*) GPIOH_BASE_ADDRESS
#define GPIOI    (GPIO_Registers_t*) GPIOI_BASE_ADDRESS
#define GPIOJ    (GPIO_Registers_t*) GPIOJ_BASE_ADDRESS
#define GPIOK    (GPIO_Registers_t*) GPIOK_BASE_ADDRESS

//RCC peripheral definition
#define RCC       ((RCC_Registers_t*) RCC_BASE_ADDRESS)

//EXTI peripheral definition
#define EXTI      ((EXTI_Registers_t*) EXTI_BASE_ADDRESS)

//SYSCFG peripheral definition
#define SYSCFG    ((SYSCFG_Registers_t*) SYSCFG_BASE_ADDRESS)

//I2C peripheral definition
#define I2C1    ((I2C_Registers_t*) I2C1_BASE_ADDRESS)
#define I2C2    ((I2C_Registers_t*) I2C2_BASE_ADDRESS)
#define I2C3    ((I2C_Registers_t*) I2C3_BASE_ADDRESS)

//SPI peripheral registers definition
#define SPI2_CR1      ((uint32_t*) SPI2_CR1_BASE_ADDRESS)
#define SPI2_CR2      ((uint32_t*) SPI2_CR2_BASE_ADDRESS)
#define SPI2_SR       ((uint32_t*) SPI2_SR_BASE_ADDRESS)
#define SPI2_DR       ((uint32_t*) SPI2_DR_BASE_ADDRESS)
#define SPI2_CRCPR    ((uint32_t*) SPI2_CRCPR_BASE_ADDRESS)
#define SPI2_RXCRC    ((uint32_t*) SPI2_RXCRC_BASE_ADDRESS)
#define SPI2_TXCRC    ((uint32_t*) SPI2_TXCRC_BASE_ADDRESS)
#define SPI2_I2SCFGR  ((uint32_t*) SPI2_I2SCFGR_BASE_ADDRESS)
#define SPI2_I2SPR    ((uint32_t*) SPI2_I2SPR_BASE_ADDRESS)

//UART/USART peripheral registers definition
#define USART1		  ((USART_Registers_t*) USART1_BASE_ADDRESS)
#define USART2		  ((USART_Registers_t*) USART2_BASE_ADDRESS)
#define USART3		  ((USART_Registers_t*) USART3_BASE_ADDRESS)
#define UART4		  ((USART_Registers_t*) UART4_BASE_ADDRESS)
#define UART5		  ((USART_Registers_t*) UART5_BASE_ADDRESS)
#define USART6		  ((USART_Registers_t*) USART6_BASE_ADDRESS)
#define UART7		  ((USART_Registers_t*) UART7_BASE_ADDRESS)
#define UART8		  ((USART_Registers_t*) UART8_BASE_ADDRESS)

//**************************Peripherals bit position definitions******************************//
//SPI2_CR1 bit fields position definition
#define SPI2_CR1_CPHA        0
#define SPI2_CR1_CPOL        1
#define SPI2_CR1_MSTR        2
#define SPI2_CR1_BR          3
#define SPI2_CR1_SPE         6
#define SPI2_CR1_LSB_FIRST   7
#define SPI2_CR1_SSI         8
#define SPI2_CR1_SSM         9
#define SPI2_CR1_RX_ONLY     10
#define SPI2_CR1_DFF         11
#define SPI2_CR1_CRCNEXT     12
#define SPI2_CR1_CRCEN       13
#define SPI2_CR1_BIDIOE      14
#define SPI2_CR1_BIDIMODE    15

//SPI2_CR2 bit fields position definition
#define SPI2_CR2_RXDMAEN     0
#define SPI2_CR2_TXDMAEN     1
#define SPI2_CR2_SSOE        2
#define SPI2_CR2_FRF         4
#define SPI2_CR2_ERRIE       5
#define SPI2_CR2_RXNEIE      6
#define SPI2_CR2_TXEIE       7

//SPI2_SR bit fields position definition
#define SPI2_SR_RXNE         0
#define SPI2_SR_TXE          1
#define SPI2_SR_CHSIDE       2
#define SPI2_SR_UDR          3
#define SPI2_SR_CRCERR       4
#define SPI2_SR_MODF         5
#define SPI2_SR_OVR			 6
#define SPI2_SR_BSY			 7
#define SPI2_SR_FRE			 8

//I2C bit fields position definition
#define I2C_CCR_FS			 15
#define I2C_OAR1_ADDR		  1
#define I2C_CR1_ACK			 10
#define I2C_CR1_START		  8
#define I2C_CR1_STOP		  9
#define I2C_CR2_ITBUFEN		 10
#define I2C_CR2_ITEVTEN		  9
#define I2C_CR2_ITERREN		  8
#define I2C_CCR_FM_DUTY      14
#define I2C_SR1_SB			  0
#define I2C_SR1_ADDR		  1
#define I2C_SR1_TXE			  7
#define I2C_SR1_RXNE		  6
#define I2C_SR1_BTF			  2
#define I2C_SR1_STOPF		  4
#define I2C_SR1_BERR		  8
#define I2C_SR1_ARLO		  9
#define I2C_SR1_AF			 10
#define I2C_SR1_OVR			 11
#define I2C_SR1_TIMEOUT		 14
#define I2C_SR2_TRA			  2

//USART bit fields position definition
#define USART_CR1_UE		  13
#define USART_CR1_RE		  2
#define USART_CR1_TE		  3
#define USART_CR1_M			 12
#define USART_CR1_PS		  9
#define USART_CR1_PCE		 10
#define USART_CR1_OVER8		 15
#define USART_CR1_TXEIE		  7
#define USART_CR1_TCIE		  6
#define USART_CR1_RXNEIE	  5
#define USART_CR2_STOP		 12
#define USART_CR3_RTSE		  8
#define USART_CR3_CTSE		  9
#define USART_SR_TXE		  7
#define USART_SR_RXNE		  5
#define USART_SR_TC			  6
#define USART_BRR_MANTISSA	  4
#define USART_BRR_FRACTION	  0

//**************************Peripherals clock enable/disable******************************//
//GPIO peripherals clock enable
#define GPIOA_CLOCK_ENABLE()   (RCC->AHB1ENR |=(0x1))
#define GPIOB_CLOCK_ENABLE()   (RCC->AHB1ENR |=(0x1<<1))
#define GPIOC_CLOCK_ENABLE()   (RCC->AHB1ENR |=(0x1<<2))
#define GPIOD_CLOCK_ENABLE()   (RCC->AHB1ENR |=(0x1<<3))
#define GPIOE_CLOCK_ENABLE()   (RCC->AHB1ENR |=(0x1<<4))
#define GPIOF_CLOCK_ENABLE()   (RCC->AHB1ENR |=(0x1<<5))
#define GPIOG_CLOCK_ENABLE()   (RCC->AHB1ENR |=(0x1<<6))
#define GPIOH_CLOCK_ENABLE()   (RCC->AHB1ENR |=(0x1<<7))
#define GPIOI_CLOCK_ENABLE()   (RCC->AHB1ENR |=(0x1<<8))
#define GPIOJ_CLOCK_ENABLE()   (RCC->AHB1ENR |=(0x1<<9))
#define GPIOK_CLOCK_ENABLE()   (RCC->AHB1ENR |=(0x1<<10))

//GPIO peripherals clock disable
#define GPIOA_CLOCK_DISABLE()   (RCC->AHB1ENR &=~(0x1))
#define GPIOB_CLOCK_DISABLE()   (RCC->AHB1ENR &=~(0x1<<1))
#define GPIOC_CLOCK_DISABLE()   (RCC->AHB1ENR &=~(0x1<<2))
#define GPIOD_CLOCK_DISABLE()   (RCC->AHB1ENR &=~(0x1<<3))
#define GPIOE_CLOCK_DISABLE()   (RCC->AHB1ENR &=~(0x1<<4))
#define GPIOF_CLOCK_DISABLE()   (RCC->AHB1ENR &=~(0x1<<5))
#define GPIOG_CLOCK_DISABLE()   (RCC->AHB1ENR &=~(0x1<<6))
#define GPIOH_CLOCK_DISABLE()   (RCC->AHB1ENR &=~(0x1<<7))
#define GPIOI_CLOCK_DISABLE()   (RCC->AHB1ENR &=~(0x1<<8))
#define GPIOJ_CLOCK_DISABLE()   (RCC->AHB1ENR &=~(0x1<<9))
#define GPIOK_CLOCK_DISABLE()   (RCC->AHB1ENR &=~(0x1<<10))

//SYSCFG peripheral clock enable
#define SYSCFG_CLOCK_ENABLE()   (RCC->APB2ENR |= (0x1<<14))

//EXTI peripheral clock enable
#define EXTI_CLOCK_ENABLE()     (RCC->APB2ENR |= (0x1<<14))

//SPI1 peripheral clock enable
#define SPI2_CLOCK_ENABLE()     (RCC->APB1ENR |= (0x1<<14))

//SPI1 peripheral clock disable
#define SPI2_CLOCK_DISABLE()    (RCC->APB1ENR &=~ (0x1<<14))

//I2C peripherals clock enable
#define I2C1_CLOCK_ENABLE()     (RCC->APB1ENR |= (0x1<<21))
#define I2C2_CLOCK_ENABLE()     (RCC->APB1ENR |= (0x1<<22))
#define I2C3_CLOCK_ENABLE()     (RCC->APB1ENR |= (0x1<<23))

//I2C peripherals clock disable
#define I2C1_CLOCK_DISABLE()    (RCC->APB1ENR &=~ (0x1<<21))
#define I2C2_CLOCK_DISABLE()    (RCC->APB1ENR &=~ (0x1<<22))
#define I2C3_CLOCK_DISABLE()    (RCC->APB1ENR &=~ (0x1<<23))

//USART/UART peripherals clock enable
#define USART1_CLOCK_ENABLE()	(RCC->APB2ENR |= (0x1<<4))
#define USART2_CLOCK_ENABLE()	(RCC->APB1ENR |= (0x1<<17))
#define USART3_CLOCK_ENABLE()	(RCC->APB1ENR |= (0x1<<18))
#define UART4_CLOCK_ENABLE()	(RCC->APB1ENR |= (0x1<<19))
#define UART5_CLOCK_ENABLE()	(RCC->APB1ENR |= (0x1<<20))
#define USART6_CLOCK_ENABLE()	(RCC->APB2ENR |= (0x1<<5))
#define UART7_CLOCK_ENABLE()	(RCC->APB1ENR |= (0x1<<30))
#define UART8_CLOCK_ENABLE()	(RCC->APB1ENR |= (0x1<<31))

//USART/UART peripherals clock disable
#define USART1_CLOCK_DISABLE()	(RCC->APB2ENR &=~ (0x1<<4))
#define USART2_CLOCK_DISABLE()	(RCC->APB1ENR &=~ (0x1<<17))
#define USART3_CLOCK_DISABLE()	(RCC->APB1ENR &=~ (0x1<<18))
#define UART4_CLOCK_DISABLE()	(RCC->APB1ENR &=~ (0x1<<19))
#define UART5_CLOCK_DISABLE()	(RCC->APB1ENR &=~ (0x1<<20))
#define USART6_CLOCK_DISABLE()	(RCC->APB2ENR &=~ (0x1<<5))
#define UART7_CLOCK_DISABLE()	(RCC->APB1ENR &=~ (0x1<<30))
#define UART8_CLOCK_DISABLE()	(RCC->APB1ENR &=~ (0x1<<31))

//GPIO peripherals register reset
#define GPIOA_REGISTER_RESET()       do{ RCC->AHB1RSTR |= (0x1);     RCC->AHB1RSTR &=~(0x1);} while(0)
#define GPIOB_REGISTER_RESET()       do{ RCC->AHB1RSTR |= (0x1<<1);  RCC->AHB1RSTR &=~(0x1<<1);} while(0)
#define GPIOC_REGISTER_RESET()       do{ RCC->AHB1RSTR |= (0x1<<2);  RCC->AHB1RSTR &=~(0x1<<2);} while(0)
#define GPIOD_REGISTER_RESET()       do{ RCC->AHB1RSTR |= (0x1<<3);  RCC->AHB1RSTR &=~(0x1<<3);} while(0)
#define GPIOE_REGISTER_RESET()       do{ RCC->AHB1RSTR |= (0x1<<4);  RCC->AHB1RSTR &=~(0x1<<4);} while(0)
#define GPIOF_REGISTER_RESET()       do{ RCC->AHB1RSTR |= (0x1<<5);  RCC->AHB1RSTR &=~(0x1<<5);} while(0)
#define GPIOG_REGISTER_RESET()       do{ RCC->AHB1RSTR |= (0x1<<6);  RCC->AHB1RSTR &=~(0x1<<6);} while(0)
#define GPIOH_REGISTER_RESET()       do{ RCC->AHB1RSTR |= (0x1<<7);  RCC->AHB1RSTR &=~(0x1<<7);} while(0)
#define GPIOI_REGISTER_RESET()       do{ RCC->AHB1RSTR |= (0x1<<8);  RCC->AHB1RSTR &=~(0x1<<8);} while(0)
#define GPIOJ_REGISTER_RESET()       do{ RCC->AHB1RSTR |= (0x1<<9);  RCC->AHB1RSTR &=~(0x1<<9);} while(0)
#define GPIOK_REGISTER_RESET()       do{ RCC->AHB1RSTR |= (0x1<<10);  RCC->AHB1RSTR &=~(0x1<<10);} while(0)

// Generic macros
#define ENABLE          1
#define DISABLE 		0
#define SET     		ENABLE
#define RESET   		DISABLE
#define PIN_SET 		ENABLE
#define PIN_RESET		DISABLE
#define HIGH			1
#define LOW				0

#endif /* INC_STM32F407XX_H_ */
