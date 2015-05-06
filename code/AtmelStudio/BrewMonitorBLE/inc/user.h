/* *************************************************************************
 * user.h
 *
 * Created on 2014-05-13.
 * Copyright G.R.R Systems 2014. All rights reserved.
 *
 * This file is encoded as UTF-8.
 *
 * $Id: user.h 24481 2014-06-03 19:26:29Z marcandre $
 * *************************************************************************
 * Description:     User configuration file
 * *************************************************************************/

#ifndef USER_H
#define	USER_H

/****************************************************************************/
/* DEBUG CONFIGURATION                                                      */
/* ====================                                                     */
#define _DEBUG_UART 1               // Enable UART TXD
#define _DEBUG_DELAY_MSG 50         // Delay duration
#define _DEBUG_DELAY_MSG_LONG 500   // Delay duration
#define _BAUDRATE 9600              // Baudrate for UART COM
#define _DEFAULT_SYS_CLK 16000000UL // Default system clock
#define MAX_CHAR 50                 // Maximum characters to print

#endif	/* USER_H */

