/* *********************************************************************************
 * usart.h
 *
 * Created on 2014-02-04.
 * Copyright G.R.R. Systems 2014. All rights reserved.
 *
 * This file is encoded as UTF-8.
 *
 * $Id:
 * *********************************************************************************
 * Device family:   ATmega328P
 * *********************************************************************************
 * Description:     USART peripheral functions
 * *********************************************************************************/

#ifndef USART_H_
#define USART_H_

//**********************************************************************************//
//									Header files									//
//**********************************************************************************//
#include <stdint.h>     // Standard integer types
#include "user.h"

//**********************************************************************************//
//									Typedefs					                    //
//**********************************************************************************//

//**********************************************************************************//
//									Constants										//
//**********************************************************************************//
// Frame values
#define START_BYTE	0x5C
#define STOP_BYTE	0x7F
// Command states
enum
{
	S_BLE_BOND, // Radio advertising state
	S_ADR,      // Get device address
	S_ECH,      // Echo test
	S_ACT,      // Active State
	S_SLP,      // Sleep State
	S_RST,      // Reset State
	S_IOC,      // Interrupt-On-Change Enable/Disable State
	S_TIM0,     // Start/Stop timer0 State
	S_SCS,      // Switch oscillator type State
	S_IRCF,     // Change internal oscillator clock State
	S_SPI,      // Switch SPI clock State
	S_DAC,      // Switch DAC State
	S_VPDAC,    // Increment DAC State
	S_VMDAC,    // Decrement DAC State
	S_WAIT      // Waiting for next valid command State
};

//**************************************************************************//
//                                 Macros                                   //
//**************************************************************************//
// Compute baud rate value for register
#define BAUDRATE(fclk)  ( ( fclk / (_BAUDRATE  * 16UL) ) - 1)

//**********************************************************************************//
//								Functions Prototypes								//
//**********************************************************************************//
void USARTInit(uint16_t baud_rate);		// Initializes USART
void USARTPutc(char c);					// Send character to USART TXD
void USARTPuts(char *s);				// Send string to USART TXD
void USARTFlush(void);					// Flush receiver buffer
uint8_t USARTGetState(void);			// Get command state
void USARTUpdateState(uint8_t state);	// Update command state

#endif /* USART_H_ */