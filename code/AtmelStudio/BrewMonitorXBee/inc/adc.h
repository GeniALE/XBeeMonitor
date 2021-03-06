/* *********************************************************************************
 * adc.h
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

#ifndef ADC_H_
#define ADC_H_

//**********************************************************************************//
//									Header files									//
//**********************************************************************************//

//**********************************************************************************//
//								 Type definitions									//
//**********************************************************************************//

//**********************************************************************************//
//									Constants										//
//**********************************************************************************//
// Sensors
#define TEMP1		0x00							// On ADC0
#define TEMP2		0x01							// On ADC1
#define TEMP3		0x02							// On ADC2
#define LEVEL1		0x03							// On ADC3

//**********************************************************************************//
//								Macros definitions									//
//**********************************************************************************//
// ADC channels mask
#define ADMUX_MSK		0x0F
#define SENSOR_NB		4

//**********************************************************************************//
//								Functions Prototypes								//
//**********************************************************************************//
void ADC_Init(void);
float ADC_Read(uint8_t sensor);

#endif /* ADC_H_ */