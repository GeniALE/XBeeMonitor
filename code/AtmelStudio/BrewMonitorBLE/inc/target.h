/* *************************************************************************
 * target.h
 *
 * Created on 2014-05-14.
 * Copyright G.R.R. Systems 2014. All rights reserved.
 *
 * This file is encoded as UTF-8.
 *
 * $Id:
 * *************************************************************************
 * Device family: ATmega328P
 * *************************************************************************
 * Description: Device configuration file
 * *************************************************************************/

#ifndef TARGET_H
#define	TARGET_H

//**********************************************************************************//
//									CPU / CLOCKS									//
//**********************************************************************************//
// CPU Frequency
#ifndef F_CPU
#define F_CPU		16000000UL			// Crystal 16.000 MHz
#endif

//**********************************************************************************//
//									Header files									//
//**********************************************************************************//
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "timer.h"
#include "uart.h"
#include "spi.h"

//**************************************************************************//
//                          Constant definitions                            //
//**************************************************************************//
// General values
#define SUCCESS     0x0
#define ERROR       0x2
#define TRUE		0x1
#define FALSE		0x0
#define SET         0x1
#define CLEAR       0x0
// I/O config
#define P_IN		0x00				// Configure Port as input
#define P_OUT		0xFF				// Configure Port as output
#define IN			0x0					// Configure Pin Port as input
#define OUT			0x1					// Configure Pin Port as output
#define PU			0x1					// Configure Pin Port as input w/ pull-up
#define NPU			0x0					// Configure Pin Port as input w/ no pull-up
// I/O level
#define EN			0x1					// Enabled State
#define DIS			0x0					// Disabled State
#define P_HI		0xFF
#define P_LO		0x00
#define HI			0x1
#define LO			0x0
#define LEDS_ON		0x00				// Level to set port to turn 8xLEDs on
#define LEDS_OFF	0xFF				// Level to set port to turn 8xLEDs off
#define LED_ON		0x0					// Level to set port to turn LED on
#define LED_OFF		0x1					// Level to set port to turn LED off

// User defined I/Os
// LEDs
#define LED_PORTx	PORTC
#define LED_DDRx	DDRC
#define LED_PINx	PINC0
// Buttons
// #define BTN_TRISx TRISA3
// #define BTN_LATx LATA3
// #define BTN_IOCxP IOCAP3
// #define BTN_IOCxF IOCAF3
// SPI port
#define SPI_PORTx		PORTB
#define SPI_DDRx		DDRB
#define	SCK				PINB5
#define	MISO			PINB4
#define	MOSI			PINB3
// BLE specific
#define BLE_REQN_PORTx	PORTC
#define BLE_REQN_DDRx	DDRC
#define	BLE_REQN_PINx	PINC
#define	BLE_REQN		PINC2
#define	BLE_RST_PORTx	PORTB
#define	BLE_RST_DDRx	DDRB
#define	BLE_RST_PINx	PINB
#define	BLE_RST			PINB1
#define	BLE_RDYN_PORTx	PORTB
#define	BLE_RDYN_DDRx	DDRB
#define	BLE_RDYN_PINx	PINB
#define	BLE_RDYN		PINB0
#define	BLE_CE_PORTx	PORTD
#define	BLE_CE_DDRx		DDRD
#define	BLE_CE_PINx		PIND
#define	BLE_CE			PIND7

//**************************************************************************//
//                                 Macros                                   //
//**************************************************************************//
// Read Port Pin State
#define pin_read(Port,Pin)				((Port & (1 << Pin))) >> Pin
// Write Port Pin State
#define pin_state(Port,Pin,Val)			(Val==1)? (Port |= (Val << Pin)): (Port &= ~(128 >> (7 - Pin)))
// Read Register bit
#define reg_read(Reg,Bit)				((Reg & (1 << Bit))) >> Bit
// Set Bit
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
// Clear Bit
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

//**************************************************************************//
//                          Function prototypes                             //
//**************************************************************************//
/// Initializes the device
void SystemInit(void);
/// Initializes GPIO and peripheral
void GPIOInit(void);

#endif	/* TARGET_H */

