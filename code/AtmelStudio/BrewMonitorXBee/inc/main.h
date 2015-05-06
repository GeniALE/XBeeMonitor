/* *********************************************************************************
 * main.h
 *
 * Created on 2014-02-04.
 * Copyright GeniALE 2014. All rights reserved.
 *
 * This file is encoded as UTF-8.
 *
 * $Id:
 * *********************************************************************************
 * Description:     Standard libraries and target dependent includes
 *					Main privates
 * *********************************************************************************/

#ifndef MAIN_H_
#define MAIN_H_

//**********************************************************************************//
//									Header files									//
//**********************************************************************************//
#include "target.h"		// ATMEGA configuration and defines
#include <inttypes.h>	// Integer types
#include <string.h>     // String lib
#include <stdio.h>      // Standard I/O lib
#include <stdint.h>     // Standard integer types
#include <stdbool.h>    // For true/false definition
#include <util/delay.h>	// Delay functions

//**********************************************************************************//
//								Constant definitions								//
//**********************************************************************************//
// Thermistors circuit values
#define VREF		5								// 5V voltage reference
#define R10K		10000							// 10K resistor in series with thermistors
// Steinhart-Hart coefficients
#define A_c			0.00113929600457259				// Steinhart-Hart coefficient A
#define B_c			0.000231949467390149			// Steinhart-Hart coefficient B
#define C_c			0.000000105992476218967			// Steinhart-Hart coefficient C
#define D_c			-0.0000000000667898975192618	// Steinhart-Hart coefficient D
#define T_K			273.15							// Kelvin constant for conversion in Celsius
#define Bm40_0		3723							// Beta coefficient for temperature range [-40	0  ]°C
#define B0_50		3892							// Beta coefficient for temperature range [ 0 	50 ]°C
#define B50_100		4029	//3950?					// Beta coefficient for temperature range [ 50	100]°C

//**********************************************************************************//
//								 Type definitions									//
//**********************************************************************************//

//**********************************************************************************//
//								 Function prototypes								//
//**********************************************************************************//
// Read sensors
int8_t Read_Sensors(char *brew_frame);

#endif // MAIN_H_
