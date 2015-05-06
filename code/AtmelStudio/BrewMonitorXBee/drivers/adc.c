/* *********************************************************************************
 * adc.c
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
 * Description:     Sensor measurements handling
 * *********************************************************************************/

// Header files
#include "../inc/target.h"     // Target device header file

// Private variables
static uint16_t adc_temp[SENSOR_NB];
static uint8_t  adc_temp_cntr[SENSOR_NB];

/***********************************************************************************
 * @prototype       void ADC_Init(void)
 * @description     Initializes ADC module, 10-bit result in ADCH & ADC
 * @param           None
 * @return          None
 ***********************************************************************************/
void ADC_Init(void)
{
	// ADC config -----------------------------------------------------
	cbi(ADMUX, REFS1);				// Voltage reference selection:
	cbi(ADMUX, REFS0);				// AREF, Internal Vref turned off
	
	cbi(ADMUX, ADLAR);				// Result is right adjusted, LSB in ADCL , 2 MSB in ADCH
	
	sbi(ADCSRA, ADPS2);				// Set ADC prescaler factor to 128
	sbi(ADCSRA, ADPS1);				// 16 MHz / 128 = 125 KHz
	sbi(ADCSRA, ADPS0);

	sbi(ADCSRA, ADEN);				// Enable ADC conversions
	sbi(ADCSRA, ADIE);				// ADC Interrupt Enabled
}

/***********************************************************************************
 * @prototype       float ADC_Read(uint8_t sensor)
 * @description     Reads average of 10-bit conversion results
 * @param           *adcl_av: Pointer to left motor measured speed result (10-bit)
 *					*adcr_av: Pointer to right motor measured speed result (10-bit)
 * @return          None
 ***********************************************************************************/
float ADC_Read(uint8_t sensor)
{
	char cSREG = SREG;			// Store SREG value
	cli();						// Disable interrupts during timed sequence
	
	float sensorValue = ((float)adc_temp[sensor])/((float)adc_temp_cntr[sensor]);
	
	adc_temp[sensor] = CLEAR;
	adc_temp_cntr[sensor] = CLEAR;
	
	SREG = cSREG;				// Restore SREG value (I-bit)
	
	return sensorValue;
}

/***********************************************************************************
 * @prototype       ISR(USART_RX_vect)
 * @description     Interrupt Handler for ADC Conversion Complete
 * @param           none
 * @return          none
 ***********************************************************************************/
ISR(ADC_vect)
{
	uint8_t admux_state;
	
	admux_state = ADMUX & ADMUX_MSK;
	
	switch (admux_state)
	{
		case ADC0:
			adc_temp_cntr[TEMP1] ++;
			adc_temp[TEMP1] += ADC;
			break;
		case ADC1:
			adc_temp_cntr[TEMP2] ++;
			adc_temp[TEMP2] += ADC;
			break;
		case ADC2:
			adc_temp_cntr[TEMP3] ++;
			adc_temp[TEMP3] += ADC;
			break;
		case ADC3:
			adc_temp_cntr[LEVEL1] ++;
			adc_temp[LEVEL1] += ADC;
			break;
		default:
			break;
	}
}
