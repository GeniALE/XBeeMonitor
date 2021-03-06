/* *********************************************************************************
 * timer.c
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

// Header files
#include "../inc/target.h"     // Target device header file

// Private variables
volatile uint8_t Timer0ReloadVal, msFlag;
volatile uint16_t CallBackFactor;

/***********************************************************************************
 * @prototype       void Timer0Init(void)
 * @description     Initializes Timer 0
 * @param           None
 * @return          None
 ***********************************************************************************/
void Timer0Init(void)
{
	// Timer 0 -----------------------------
	TMR0_SetPrescaler(TMR0_PRESCALER_1_64);

	// Preset timer 0 register and load the TMR value to reload variable
	TCNT0 = TMR0_SetReload_ms(64);
	Timer0ReloadVal = TCNT0;

	sbi(TCCR0A, WGM01);		// Configure timer 0 in Fast PWM mode, TOV Flag set on MAX
	sbi(TCCR0A, WGM00);		// Configure timer 0 in Fast PWM mode, TOV Flag set on MAX
}

/***********************************************************************************
 * @prototype       void Timer1Init(void)
 * @description     Initializes Timer 1
 * @param           None
 * @return          None
 ***********************************************************************************/
void Timer1Init(void)
{	
	TMR1_SetPrescaler(TMR1_PRESCALER_1_256);
	
	TCNT1 = TIM1_INIT;		// Set timer 1 counter (1/(16L/256))*(2^16-TIM1_INIT)

	TMR1_StartTimer();
}

/***********************************************************************************
 * @prototype       void Delay(unsigned long ms)
 * @description     Provides a delay in milliseconds
 * @param           ms: Milliseconds delay
 * @return          none
 ***********************************************************************************/
void Delay(uint16_t ms)
{
    TMR0_StartTimer();
    CallBackFactor = ms;

    while (!msFlag)
    {
    }
    msFlag = CLEAR;
    TMR0_StopTimer();
}

/***********************************************************************************
 * @prototype       ISR(TIMER0_OVF_vect)
 * @description     Used for delay functions
 * @param           none
 * @return          none
 ***********************************************************************************/
ISR(TIMER0_OVF_vect)
{
	static volatile unsigned int CountCallBack = 0;
	
	TCNT0 = Timer0ReloadVal;     // Reload timer 0 counter

	if (++CountCallBack >= CallBackFactor)
	{
		msFlag = SET;
		CountCallBack = 0;		// Reset ticker counter
	}
}

