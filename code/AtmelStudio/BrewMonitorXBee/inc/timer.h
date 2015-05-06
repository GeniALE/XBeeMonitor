/* *********************************************************************************
 * timer.h
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
 * Description:     Timing handling
 * *********************************************************************************/

#ifndef TIMER_H_
#define TIMER_H_

//**********************************************************************************//
//									Header files									//
//**********************************************************************************//
#include <stdint.h>     // Standard integer types

//**********************************************************************************//
//									Typedefs										//
//**********************************************************************************//

//**********************************************************************************//
//									Constants										//
//**********************************************************************************//
// Timer 0 prescalers
#define TMR0_OFF 0x00
#define TMR0_PRESCALER_1_1 0x01
#define TMR0_PRESCALER_1_8 0x02
#define TMR0_PRESCALER_1_64 0x03
#define TMR0_PRESCALER_1_256 0x04
#define TMR0_PRESCALER_1_1024 0x05
#define TMR0_EXT_CLK_FALLING 0x06
#define TMR0_EXT_CLK_RISING 0x07
// Timer 1 prescalers
#define TMR1_OFF 0x00
#define TMR1_PRESCALER_1_1 0x01
#define TMR1_PRESCALER_1_8 0x02
#define TMR1_PRESCALER_1_64 0x03
#define TMR1_PRESCALER_1_256 0x04
#define TMR1_PRESCALER_1_1024 0x05
#define TMR1_EXT_CLK_FALLING 0x06
#define TMR1_EXT_CLK_RISING 0x07
// Timer 2 prescalers
#define TMR2_OFF 0x00
#define TMR2_PRESCALER_1_1 0x01
#define TMR2_PRESCALER_1_8 0x02
#define TMR2_PRESCALER_1_32 0x03
#define TMR2_PRESCALER_1_64 0x04
#define TMR2_PRESCALER_1_128 0x05
#define TMR2_PRESCALER_1_256 0x06
#define TMR2_PRESCALER_1_1024 0x07

//**********************************************************************************//
//								Macros definitions									//
//**********************************************************************************//
// Watchdog Timer Reset
#define wdt_reset()				__asm__ __volatile__ ("wdr")

// Enable timers
#define TMR0_StartTimer()		(TIMSK0 |= (128 >> (7 - TOIE0)))
#define TMR1_StartTimer()		(TIMSK1 |= (128 >> (7 - TOIE1)))
#define TMR2_StartTimer()		(TIMSK2 |= (128 >> (7 - TOIE2)))

// Disable timers
#define TMR0_StopTimer()		(TIMSK0 &= ~(1 << TOIE0))
#define TMR1_StopTimer()		(TIMSK1 &= ~(1 << TOIE1))
#define TMR2_StopTimer()		(TIMSK2 &= ~(1 << TOIE2))

// Set timer 0 prescaler
#define TMR0_SetPrescaler(PSx)	(TCCR0B = (TCCR0B & 0xF8) | (PSx & 0x07))

// ((2^8-reload_value)/((F_CPU)/prescaler))*n -> 2^8 - F_CPU/prescaler
#define TMR0_SetReload_ms(prescaler) ( 256 - ( (F_CPU / 1000) / prescaler ) )

// Set timer 1 prescaler
#define TMR1_SetPrescaler(PSx)	(TCCR1B = (TCCR1B & 0xF8) | (PSx & 0x07))

// ((2^16-reload_value)/((F_CPU)/prescaler))*n -> 2^16 - F_CPU/prescaler
#define TMR1_SetReload_ms(prescaler) ( 65536 - ( (F_CPU / 1000) / prescaler ) )

#define TIM1_INIT	59286	// Timer 1 default counting value -> (1/(16L/256))*(2^16-59286) = 0.1 s

//**********************************************************************************//
//								Functions Prototypes								//
//**********************************************************************************//
void Timer0Init(void);
void Timer1Init(void);
void Delay(uint16_t ms);

#endif /* TIMER_H_ */