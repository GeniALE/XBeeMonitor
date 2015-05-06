/* *************************************************************************
 * target_config.c
 *
 * Created on 2014-05-14.
 * Copyright G.R.R. Systems 2014. All rights reserved.
 *
 * This file is encoded as UTF-8.
 *
 * $Id: target_config.c 24481 2014-06-03 19:26:29Z marcandre $
 * *************************************************************************
 * Device family: ATmega328P
 * *************************************************************************
 * Description: Device configuration file
 * *************************************************************************/

// Header files
#include "inc/target.h"     // Target device header file

/***************************************************************************
 * @prototype       void SystemInit(void)
 * @description     Initializes the device
 * @param           none
 * @return          none
 ***************************************************************************/
void SystemInit(void)
{
    // Initialize GPIOs
    GPIOInit();

    // Initialize Timers
    Timer0Init();

    // Initialize UART
    USARTInit(_BAUDRATE);
   
    // Enable interrupts
    sei();
}

/***************************************************************************
 * @prototype       void GPIOInit(void)
 * @description     Initializes GPIOs
 * @param           none
 * @return          none
 ***************************************************************************/
void GPIOInit(void)
{
	// Pull-Ups enabled
	cbi(MCUCR, PUD);
}
