/* *************************************************************************
 * spi.c
 *
 * Created on 2014-05-22.
 * Copyright G.R.R. Systems Research 2014. All rights reserved.
 *
 * This file is encoded as UTF-8.
 *
 * $Id: 
 * *************************************************************************
 * Device family: ATmega328P
 * *************************************************************************
 * Description: SPI peripheral functions
 * *************************************************************************/

// Header files
#include "../inc/target.h"     // Target device header file

// Private variables

/***********************************************************************************
 * @prototype       void SPI_MasterInit(void)
 * @description     Initialize SPI module in master mode
 * @param           None
 * @return          None
 ***********************************************************************************/
void SPI_MasterInit(void)
{
	// SPI control config
	// Warning: if the SS pin ever becomes a LOW INPUT then SPI automatically switches 
	// to Slave, so the data direction of the SS pin MUST be kept as OUTPUT
	sbi(SPCR,SPE);		// Enable SPI
	sbi(SPCR,DORD);		// Data Order: LSB transmitted first
	sbi(SPCR,MSTR);		// Master mode
	
	// Set clock divider
	SPI_setClockDivider(SPI_CLOCK_DIV16);
	
	// Sample on Leading rising edge, setup on trailing falling edge
	SPI_dataMode(SPI_MODE0);
	
	// Set I/Os
	pin_state(SPI_DDRx,SCK,OUT);
	pin_state(SPI_DDRx,MOSI,OUT);
	pin_state(SPI_DDRx,MISO,IN);
	// Set SS to high so a connected chip will be "deselected" by default
	pin_state(SPI_PORTx,BLE_REQN,HI);
	pin_state(SPI_DDRx,BLE_REQN,OUT);
}

/***********************************************************************************
 * @prototype       void SPI_SlaveInit(void)
 * @description     Initialize SPI module in slave mode
 * @param           None
 * @return          None
 ***********************************************************************************/
void SPI_SlaveInit(void)
{
	// SPI control config
	sbi(SPCR,SPE);		// Enable SPI
	cbi(SPCR,MSTR);		// Slave mode
	
	// Set I/Os
	pin_state(SPI_DDRx,SCK,IN);
	pin_state(SPI_DDRx,MOSI,IN);
	pin_state(SPI_DDRx,MISO,OUT);
}

/***************************************************************************
 * @prototype       void SPIExchangeFrame(uint8_t *txBufPtr, uint8_t *rcvBufPtr, uint8_t bufLength)
 * @description     Transmits or receives 'bufLength' number of bytes through MSSP
 * @param           *txBufPtr:  pointer to transmit buffer
 *                  *rcvBufPtr: pointer to receive buffer
 *                  bufLength: number of bytes to be transmitted or received over SPI
 * @return          none
 ***************************************************************************/
void SPIExchangeFrame(uint8_t *txBufPtr, uint8_t *rcvBufPtr, uint8_t bufLength)
{
    while (bufLength)
    {
        // Write the data byte in the buffer
        SPDR = *txBufPtr;

        // Wait for transmission/reception complete
        while(!(SPSR & (1<<SPIF)))
        {
        }

        // Read the received byte
        *rcvBufPtr = SPDR;

        txBufPtr++;
        rcvBufPtr++;
        bufLength--;
    }
}

/***************************************************************************
 * @prototype       uint8_t SPIExchangeByte(uint8_t data)
 * @description     Transmits and/or receives single byte of data
 * @param           data: A single byte to be transmitted or received over SPI
 * @return          return SSP1BUF
 ***************************************************************************/
uint8_t SPIExchangeByte(uint8_t data)
{
    // Write the data byte in the buffer
    SPDR = data;

    // Wait for transmission/reception complete
    while(!(SPSR & (1<<SPIF)))
    {
    }

    // Return the received byte and clear BF flag
    return (SPDR);
}


