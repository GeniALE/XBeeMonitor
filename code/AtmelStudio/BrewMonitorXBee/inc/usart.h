/* *********************************************************************************
 * usart.h
 *
 * Created on 2014-02-04.
 * Copyright GeniALE 2014. All rights reserved.
 *
 * This file is encoded as UTF-8.
 *
 * $Id:
 * *********************************************************************************
 * Device family:   ATmega328P
 * *********************************************************************************
 * Description:     Data communication handling
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
	S_ACT,      // Active State
	S_SLP,      // Sleep State
	S_RST,      // Reset State
	S_GID,      // Global Interrupt Disable State
	S_PID,      // Peripheral Interrupt Disable State
	S_IOC,      // Interrupt-On-Change Enable/Disable State
	S_INIT,     // Initialize GPIO and peripherals State
	S_DINIT,    // De-Initialize GPIO and peripherals State
	S_TIM0,     // Start/Stop timer0 State
	S_SCS,      // Switch oscillator type State
	S_IRCF,     // Change internal oscillator clock State
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
void USARTInit(uint16_t baud_rate);		// Initializes UART
void USARTPutc(char c);					// Send character to UART TXD
void USARTPuts(char *s);				// Send string to UART TXD
// Send frame to UART TXD w/ START & STOP byte
void USARTSendFrame(char *s, char s_size);
void USARTFlush(void);					// Flush receiver buffer

#endif /* USART_H_ */