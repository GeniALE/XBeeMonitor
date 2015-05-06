/* *********************************************************************************
 * target_config.c
 *
 * Created on 2014-05-29.
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

// Header files
#include "inc/target.h"     // Target device header file

/***********************************************************************************
 * @prototype       void SystemInit(void)
 * @description     Initializes the device
 * @param           None
 * @return          None
 ***********************************************************************************/
void SystemInit(void)
{
    // Initialize GPIO and peripheral
    GPIOInit();

    // Initialize Timers
    Timer0Init();
	Timer1Init();

    // Initialize UART
    USARTInit(_BAUDRATE);
	
	// Initialize ADC
	ADC_Init();

	// Enable interrupts
    sei();
}

/***********************************************************************************
 * @prototype       void GPIOInit(void)
 * @description     Initializes GPIOs
 * @param           none
 * @return          none
 ***********************************************************************************/
void GPIOInit(void)
{
	pin_state(DDRB,PINB1,OUT);
}
