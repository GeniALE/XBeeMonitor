/* *********************************************************************************
 * target.h
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
 * Description:     This file gives the possibility to configure several
 *					parameters affecting the target device itself.
 *					Refer to Device's IO reference spreadsheet for details
 * *********************************************************************************/

#ifndef TARGET_H_
#define TARGET_H_

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
#include "usart.h"
#include "adc.h"

//**********************************************************************************//
//								Constant definitions								//
//**********************************************************************************//
// General values
#define SUCCESS     0x0
#define ERROR       0x2
#define TRUE		0x1
#define FALSE		0x0
#define SET         0x1
#define CLEAR       0x0
#define MAX_CHAR	60		// Maximum characters to print
// Ports & Pins constant values
#define P_IN		0x00				// Configure Port as input
#define P_OUT		0xFF				// Configure Port as output
#define IN			0x0					// Configure Pin Port as input
#define OUT			0x1					// Configure Pin Port as output
#define IN_PU		0x2					// Configure Pin Port as input w/ pull-up
#define ON			0x1
#define OFF			0x0
#define P_HI		0xFF
#define P_LO		0x00
#define HI			0x1
#define LO			0x0
#define LEDS_ON		0x00				// Level to set port to turn 8xLEDs on
#define LEDS_OFF	0xFF				// Level to set port to turn 8xLEDs off
#define LED_ON		0x0					// Level to set port to turn LED on
#define LED_OFF		0x1					// Level to set port to turn LED off
// ADC port
#define ADC0		0x00				// ADC channel 0
#define ADC1		0x01				// ADC channel 1
#define ADC2		0x02				// ADC channel 2
#define ADC3		0x03				// ADC channel 3
#define ADC4		0x04				// ADC channel 4
#define ADC5		0x05				// ADC channel 5
#define ADC6		0x06				// ADC channel 6
#define ADC7		0x07				// ADC channel 7

//**********************************************************************************//
//										Macros										//
//**********************************************************************************//
// Read Port Pin State
#define pin_read(PortPin,Pin)			((PortPin & (1 << Pin))) >> Pin
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

//**********************************************************************************//
//								Function prototypes									//
//**********************************************************************************//
void SystemInit(void);	// Initializes the device
void GPIOInit(void);	// Initializes GPIO and peripheral

#endif /* TARGET_H_ */
