/* *********************************************************************************
 * spi.h
 *
 * Created on 2014-03-24.
 * Copyright G.R.R. Systems 2014. All rights reserved.
 *
 * This file is encoded as UTF-8.
 *
 * $Id:
 * *********************************************************************************
 * Device family:   ATmega328P
 * *********************************************************************************
 * Description:     SPI handling
 * *********************************************************************************/

#ifndef SPI_H_
#define SPI_H_

//**********************************************************************************//
//									Header files									//
//**********************************************************************************//

//**********************************************************************************//
//									Private defines									//
//**********************************************************************************//
// SPI Clock divides
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV32 0x06
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
//#define SPI_CLOCK_DIV64 0x07
// SPI modes
#define SPI_MODE0 0x00	// Sample on Leading rising edge, setup on trailing falling edge
#define SPI_MODE1 0x04	// Setup on Leading rising edge, sample on trailing falling edge
#define SPI_MODE2 0x08	// Sample on Leading falling edge, setup on trailing rising edge
#define SPI_MODE3 0x0C	// Setup on Leading falling edge, sample on trailing rising edge
// SPI masks
#define SPI_MODE_MASK 0x0C		// CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK 0x03		// SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01	// SPI2X = bit 0 on SPSR

//**********************************************************************************//
//								Macros definitions									//
//**********************************************************************************//
// Disable SPI
#define SPI_disable()		SPCR &= ~(1 << SPE)
// Set SPI data mode
#define SPI_dataMode(mode)	SPCR = (SPCR & ~SPI_MODE_MASK) | mode
// Set SPI clock divider
#define SPI_setClockDivider(rate){													\
			SPCR = (SPCR & ~SPI_CLOCK_MASK)   | (rate & SPI_CLOCK_MASK);			\
			SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((rate >> 2) & SPI_2XCLOCK_MASK);}

//**********************************************************************************//
//								Functions Prototypes								//
//**********************************************************************************//
/// Initialize SPI module in master mode
void SPI_MasterInit(void);
/// Initialize SPI module in slave mode
void SPI_SlaveInit(void);
/// Transmits or receives 'bufLength' number of bytes through SPI
void SPIExchangeFrame(uint8_t *txBufPtr, uint8_t *rcvBufPtr, uint8_t bufLength);
/// Transmits and/or receives single byte of data
uint8_t SPIExchangeByte(uint8_t data);

#endif /* SPI_H_ */