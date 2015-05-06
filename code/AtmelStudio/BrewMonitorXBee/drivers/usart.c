/* *********************************************************************************
 * usart.c
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

// Header files
#include "../inc/target.h"     // Target device header file
#include <string.h>

// Private variables
static uint8_t TxCounter;
static char *TxPtr;

/***********************************************************************************
 * @prototype       void USARTInit(uint16_t baud_rate)
 * @description     Initializes USART
 * @param           baud_rate: UART baud rate
 * @return          None
 ***********************************************************************************/
void USARTInit(uint16_t baud_rate)
{
	// Set baud rate
	//UBRR0 = BAUDRATE(F_CPU);
	
	uint8_t baud_select = (F_CPU / (9600 * 16L) - 1);
	
	UBRR0H = baud_select >> 8;
	UBRR0L = baud_select & 0x00FF;
	
	// USART control
	sbi(UCSR0B, RXCIE0);			// RX Complete Interrupt: Enabled
	cbi(UCSR0B, TXCIE0);			// TX Complete Interrupt: Disabled
	cbi(UCSR0B, UDRIE0);			// USART Data Register Empty Interrupt: Disabled
	sbi(UCSR0B, RXEN0);				// Receiver: Enabled
	sbi(UCSR0B, TXEN0);				// Transmitter: Enabled
	
	// Set frame format: 8 data, no parity, 1 stop bit
	cbi(UCSR0C, UMSEL01);			// USART Mode Select: Asynchronous Operation
	cbi(UCSR0C, UMSEL00);			// ""
	cbi(UCSR0C, UPM01);				// Parity mode: Disabled
	cbi(UCSR0C, UPM00);				// ""
	cbi(UCSR0C, USBS0);				// Stop Bit Slect: 1-bit
	sbi(UCSR0C, UCSZ01);			// Character Size: 8-bit
	sbi(UCSR0C, UCSZ00);			// ""
	cbi(UCSR0C, UCPOL0);			// Tx on rising edge, Rx on falling edge
}

/***********************************************************************************
 * @prototype       void UARTPutc(char c)
 * @description     Send character to UART TXD
 * @param           c: Character to send
 * @return          none
 ***********************************************************************************/
void USARTPutc(char c)
{
    while ( !( UCSR0A & (1<<UDRE0)) );	// Wait for empty transmit buffer
    UDR0 = c;
}

/***********************************************************************************
* @prototype		void USARTPuts(char *s, char s_size)
* @description		Send string to UART TXD
* @param			s: String to send
* @return			None
 ***********************************************************************************/
void USARTPuts(char *s)
{
	cbi(UCSR0B, RXCIE0);	// Disable RX Complete interrupt for a transmission
	TxPtr = s;				// Write first character to data buffer
	TxCounter = strlen(s);	// String size
	UDR0 = *TxPtr;
	sbi(UCSR0B, TXCIE0);	// Enable TX Complete interrupt for transmission
}

/***********************************************************************************
* @prototype		void USARTSendFrame(char s)
* @description		Send frame to UART TXD w/ START & STOP byte
* @param			s: String to send
* @return			None
 ***********************************************************************************/
void USARTSendFrame(char *s, char s_size)
{
	cbi(UCSR0B, RXCIE0);		// Disable RX Complete interrupt for a transmission
	TxPtr = s;					// Write first character to data buffer
	TxCounter = s_size + 2;		// String size + start + stop byte
	TxPtr[SENSOR_NB]=STOP_BYTE;	// Add stop byte
	UDR0 = START_BYTE;			// Send start byte
	TxPtr--;					// Decrement for start byte
	sbi(UCSR0B, TXCIE0);		// Enable TX Complete interrupt for transmission
}

/***********************************************************************************
* @prototype		void USARTFlush(void)
* @description		Flush receiver buffer (when the buffer has to be flushed  during 
*					normal operation - note that the receiver buffer FIFO will be 
*					flushed when the Receiver is disabled
* @param			None
* @return			None
 ***********************************************************************************/
void USARTFlush(void)
{
	unsigned char dummy;
	
	(void)dummy;	// Prevent compiler warnings
	
	while (UCSR0A & (1<<RXC0))
	{
		dummy = UDR0;
	}
}

/***********************************************************************************
 * @prototype       ISR(USART_TX_vect)
 * @description     Interrupt Handler for USART TxD complete
 * @param           none
 * @return          none
 ***********************************************************************************/
ISR(USART_TX_vect)
{
	//wdt_reset();					// Reset Watchdog Timer
	
	if (--TxCounter)
	{
		TxPtr++;
		UDR0 = *TxPtr;			// Write byte to data buffer
	}
	else
	{
		cbi(UCSR0B, TXCIE0);	// Disable TX Complete interrupt after transmission
		sbi(UCSR0B, RXCIE0);	// Enable RX Complete interrupt after transmission
	}
}
