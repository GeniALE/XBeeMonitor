/* *********************************************************************************
 * main.c
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
 * Description:     Temperature, density & level capture
 *					Transmit data to main receiver
 * *********************************************************************************/

// Header files
#include "inc/main.h"       // Main header file
#include "inc/user.h"       // User config file

// Private variables
static volatile uint8_t sensors_rdy_flag;
static uint8_t tx_timeout = 50;

/***********************************************************************************
 * @prototype       void main(void)
 * @description     Main task
 * @param           None
 * @return          None
 ***********************************************************************************/
int main(void)
{
	// Main variables --------------------------------------------------------------
	char brew_frame[SENSOR_NB];
	sensors_rdy_flag = CLEAR;
	
	// Device initialization -------------------------------------------------------
	SystemInit();
	Delay(100);
	
	// Enable watchdog, time-out @ 0.52s
	//WDTCR = 0x0D;

  	USARTPuts("\rBrew_Monitor v1.3\r");

	while (1)
	{
		if (sensors_rdy_flag)									////// SHOULD LET TEMP IN KELVIN FOR PRECISION & SIMPLICITY
		{
			sensors_rdy_flag = CLEAR;
 			Read_Sensors(brew_frame);
 			USARTSendFrame(brew_frame,sizeof(brew_frame));
		}
	}
}

/***********************************************************************************
 * @prototype       int8_t Read_Sensors(char *brew_frame)
 * @description     Read sensors
 * @param           brew_frame: Brew parameters data frame
 * @return          status
 ***********************************************************************************/
int8_t Read_Sensors(char *brew_frame)
{
	int8_t status = CLEAR;
	
	// Get averaged sensor values
	float temp1_B  = ADC_Read(TEMP1);	Delay(1);	// delay in between reads
	float temp2_B  = ADC_Read(TEMP2);	Delay(1);	// for stability
	float temp3_B  = ADC_Read(TEMP3);	Delay(1);
	float level1_B = ADC_Read(LEVEL1);	Delay(1);
	// Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
	float temp1_V = temp1_B * (VREF / 1023.0);
	float temp2_V = temp2_B * (VREF / 1023.0);
	float temp3_V = temp3_B * (VREF / 1023.0);
	// Retrieve thermistors (resistance) values
	float temp1_R = R10K * ((VREF / temp1_V) - 1);
	float temp2_R = R10K * ((VREF / temp2_V) - 1);
	float temp3_R = R10K * ((VREF / temp3_V) - 1);

	double T1 = 0, R1 = 0, beta = 0;		// float

	// Temp 1 measurement
	if ((temp1_R <= 338388) && (temp1_R >= 32650))		// Beta for [-50 0]°C
	{
		T1	 = 233.15;
		R1	 = 338388;
		beta = Bm40_0;
	}
	else if ((temp1_R <= 32650) && (temp1_R >= 3602))	// Beta for [0 50]°C		//	if (V_sensor1 <= 3.6)
	{
		T1	 = 273.15;
		R1	 = 32650;
		beta = B0_50;
	}
	else if ((temp1_R <= 3602) && (temp1_R >= 586))		// Beta for [50 100]°C
	{
		T1	 = 323.15;
		R1	 = 3602;
		beta = B50_100;
	}
	else
	{
		status |= 0x01;	// Error code temp1
		brew_frame[TEMP1] = 0;
	}
	if(!(status & 0x01))
		brew_frame[TEMP1] = (beta * T1)/(beta + log(temp1_R/R1) * T1) - T_K;
	
	// Temp 2 measurement
	if ((temp2_R <= 338388) && (temp2_R >= 32650))		// Beta for [-50 0]°C
	{
		T1	 = 233.15;
		R1	 = 338388;
		beta = Bm40_0;
	}
	else if ((temp2_R <= 32650) && (temp2_R >= 3602))	// Beta for [0 50]°C
	{
		T1	 = 273.15;
		R1	 = 32650;
		beta = B0_50;
	}
	else if ((temp2_R <= 3602) && (temp2_R >= 586))		// Beta for [50 100]°C
	{
		T1	 = 323.15;
		R1	 = 3602;
		beta = B50_100;
	}
	else
	{
		status |= 0x02;	// Error code temp2
		brew_frame[TEMP2] = 0;
	}
	if(!(status & 0x02))
		brew_frame[TEMP2] = (beta * T1)/(beta + log(temp2_R/R1) * T1) - T_K;
	
	// Temp 3 measurement
	if ((temp3_R <= 338388) && (temp3_R >= 32650))		// Beta for [-50 0]°C
	{
		T1	 = 233.15;
		R1	 = 338388;
		beta = Bm40_0;
	}
	else if ((temp3_R <= 32650) && (temp3_R >= 3602))	// Beta for [0 50]°C
	{
		T1	 = 273.15;
		R1	 = 32650;
		beta = B0_50;
	}
	else if ((temp3_R <= 3602) && (temp3_R >= 586))		// Beta for [50 100]°C
	{
		T1	 = 323.15;
		R1	 = 3602;
		beta = B50_100;
	}
	else
	{
		status |= 0x04;	// Error code temp3
		brew_frame[TEMP3] = 0;
	}
	if(!(status & 0x04))
		brew_frame[TEMP3] = (beta * T1)/(beta + log(temp3_R/R1) * T1) - T_K;
	
	status |= 0x08;	// Error code level1
	brew_frame[LEVEL1] = level1_B * (16.0/1023.0) + 7;
	
	return status;
}

/***********************************************************************************
 * @prototype       ISR(TIMER1_OVF_vect)
 * @description     Interrupt Handler for Timer/Counter1 Overflow
 *					Overflow ~ 0.1 sec -> (1/(16L/1024))*(2^16-TIM1_INIT) = 0.1 s
 * @param           none
 * @return          none
 ***********************************************************************************/
ISR(TIMER1_OVF_vect)
{
	static uint8_t adc_channel, t_tim1;
	
	TCNT1 = TIM1_INIT;				// Set timer 1 counter
	
	// Overflow ~ 0.1 sec -> (1/(16L/1024))*(2^16-TIM1_INIT) = 0.1 s
	adc_channel %= SENSOR_NB;		// Channel cycle
	ADMUX &= ~ADMUX_MSK;			// Clear last selected ADC channel
	ADMUX |= adc_channel;			// Select new channel
	adc_channel ++;					// Prepare next channel
	ADCSRA |= (1 << ADSC);			// Start ADC conversion

	t_tim1 ++;
	if (t_tim1 >= tx_timeout)		// Each sensor is sampled n time where
	{								// n = MAX_ADC_SAMPLE/SENSOR_NB
		t_tim1 = 0;
		sensors_rdy_flag = SET;
	}
}

/***********************************************************************************
 * @prototype       ISR(USART_RX_vect)
 * @description     Interrupt Handler for USART RxD complete
 * @param           none
 * @return          none
 ***********************************************************************************/
ISR(USART_RX_vect)
{
	static uint8_t i;
	static uint8_t brew_frame_cmd[2];
	
	brew_frame_cmd[i] = UDR0;						// Read byte from USART data buffer

	if ((brew_frame_cmd[0] == START_BYTE) || (i > 0))	// Normal command received or if command already started
	{
		i++;
		if (i > 2)
		{
			tx_timeout = brew_frame_cmd[1];
			UDR0 = brew_frame_cmd[1];
			i = CLEAR;
		}
	}
}