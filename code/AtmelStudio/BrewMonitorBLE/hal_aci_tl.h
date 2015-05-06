/* *************************************************************************
 * hal_aci_tl.h
 *
 * Created on 2014-05-29.
 * Copyright Rogue Research 2014. All rights reserved.
 *
 * This file is encoded as UTF-8.
 *
 * $Id: hal_aci_tl.h 24481 2014-06-03 19:26:29Z marcandre $
 * *************************************************************************
 * Description:     Interface for hal_aci_tl
 *                  Adapted from Nordic Semiconductor SDK v1.7
 * *************************************************************************/

/** @file
 * @brief Interface for hal_aci_tl.
 */
 
/** @defgroup hal_aci_tl hal_aci_tl
@{
@ingroup hal
 
@brief Module for the ACI Transport Layer interface
@details This module is responsible for sending and receiving messages over the ACI interface of the nRF8001 chip.
 The hal_aci_tl_send_cmd() can be called directly to send ACI commands.

The RDYN line is hooked to an interrupt on the MCU when the level is low.
The SPI master clocks in the interrupt context.
The ACI Command is taken from the head of the command queue is sent over the SPI
and the received ACI event is placed in the tail of the event queue.
*/
 
#ifndef HAL_ACI_TL_H__
#define HAL_ACI_TL_H__

//**************************************************************************//
//                              Header files                                //
//**************************************************************************//
#include "hal_platform.h"
#include "aci.h"

//**************************************************************************//
//                               Constants                                  //
//**************************************************************************//
#ifndef HAL_ACI_MAX_LENGTH
#define HAL_ACI_MAX_LENGTH 31
#endif

//**************************************************************************//
//                               Typedefs                                   //
//**************************************************************************//
/** Data type for ACI commands and events */
typedef struct
{
    uint8_t status_byte;
    uint8_t buffer[HAL_ACI_MAX_LENGTH+1];
} _aci_packed_ hal_aci_data_t;

ACI_ASSERT_SIZE(hal_aci_data_t, HAL_ACI_MAX_LENGTH + 2);

//**************************************************************************//
//                          Function prototypes                             //
//**************************************************************************//
/** @brief ACI Transport Layer initialization.
 *  @details
 *  This function initializes the transport layer, including configuring the SPI, creating
 *  message queues for Commands and Events and setting up interrupt if required.
 *  @param a_pins Pins on the MCU used to connect to the nRF8001
 *  @param bool True if debug printing should be enabled on the Serial.
 */
void hal_aci_tl_init(void);

/** @brief Sends an ACI command to the radio.
 *  @details
 *  This function sends an ACI command to the radio. This queue up the message to send and 
 *  lower the request line. When the device lowers the ready line, @ref m_aci_spi_transfer()
 *  will send the data.
 *  @param aci_buffer Pointer to the message to send.
 *  @return True if the data was successfully queued for sending, 
 *  false if there is no more space to store messages to send.
 */
bool hal_aci_tl_send(hal_aci_data_t *aci_buffer);

/** @brief Process pending transactions.
 *  @details 
 *  The library code takes care of calling this function to check if the nRF8001 RDYN line indicates a
 *  pending transaction. It will send a pending message if there is one and return any receive message
 *  that was pending.
 *  @return Points to data buffer for received data. Length byte in buffer is 0 if no data received.
 */
hal_aci_data_t * hal_aci_tl_poll_get(void);

/** @brief Get an ACI event from the event queue
 *  @details 
 *  Call this function from the main context to get an event from the ACI event queue
 *  This is called by lib_aci_event_get
 */
bool hal_aci_tl_event_get(hal_aci_data_t *p_aci_data);

/** @brief Peek an ACI event from the event queue
 *  @details
 *  Call this function from the main context to peek an event from the ACI event queue.
 *  This is called by lib_aci_event_peek
 */
bool hal_aci_tl_event_peek(hal_aci_data_t *p_aci_data);

/** @brief Return full status of transmit queue
 *  @details
 *
 */
 bool hal_aci_tl_rx_q_full(void);
 
 /** @brief Return empty status of receive queue
 *  @details
 *
 */
 bool hal_aci_tl_rx_q_empty(void);

/** @brief Return full status of receive queue
 *  @details
 *
 */
 bool hal_aci_tl_tx_q_full(void);
 
 /** @brief Return empty status of transmit queue
 *  @details
 *
 */
 bool hal_aci_tl_tx_q_empty(void);

/** @brief Flush the ACI command Queue and the ACI Event Queue
 *  @details
 *  Call this function in the main thread
 */
void hal_aci_tl_q_flush(void);

#endif // HAL_ACI_TL_H__
/** @} */
