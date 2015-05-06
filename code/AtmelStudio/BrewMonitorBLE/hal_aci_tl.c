/* *************************************************************************
 * hal_aci_tl.c
 *
 * Created on 2014-05-29.
 * Copyright Rogue Research 2014. All rights reserved.
 *
 * This file is encoded as UTF-8.
 *
 * $Id: hal_aci_tl.c 24481 2014-06-03 19:26:29Z marcandre $
 * *************************************************************************
 * Device family:   PIC XLP 16F
 * *************************************************************************
 * Description:     Implementation of the ACI transport layer module
 *                  Adapted from Nordic Semiconductor SDK v1.7
 * *************************************************************************/

// Header files ------------------------------------------------------------
#include "inc/target.h"     // Target device header file
#include "hal_platform.h"
#include "hal_aci_tl.h"
#include "aci_queue.h"

// Private variables -------------------------------------------------------
aci_queue_t    aci_tx_q;
aci_queue_t    aci_rx_q;

//**************************************************************************//
//                        Private function prototypes                       //
//**************************************************************************//
static void m_aci_data_print(hal_aci_data_t *p_data);
static void m_aci_event_check(void);
static bool m_aci_spi_transfer(hal_aci_data_t * data_to_send, hal_aci_data_t * received_data);

/***************************************************************************
 * @prototype       static void m_aci_data_print(hal_aci_data_t *p_data)
 * @description
 * @param
 * @return          None
 ***************************************************************************/
static void m_aci_data_print(hal_aci_data_t *p_data)
{
    char msg[MAX_CHAR];
    uint8_t length = p_data->buffer[0];

    sprintf(msg, "[Length=%d]:\r",length);  USARTPuts(msg);
    Delay(_DEBUG_DELAY_MSG);

    for (uint8_t i = 1; i <= length; i++)
    {
        sprintf(msg, "%x ",p_data->buffer[i]);    USARTPuts(msg);
        Delay(_DEBUG_DELAY_MSG);
    }
    USARTPutc('\r');
}

/***************************************************************************
 * @prototype       static void m_aci_event_check(void)
 * @description     Checks the RDYN line and runs the SPI transfer if required.
 * @param           None
 * @return          None
 ***************************************************************************/
static void m_aci_event_check(void)
{
    hal_aci_data_t data_to_send;
    hal_aci_data_t received_data;

    // No room to store incoming messages
    if (aci_queue_is_full(&aci_rx_q))
    {
        return;
    }

    // If the ready line is disabled and we have pending messages outgoing we enable the request line
    if (HI == reg_read(BLE_RDYN_PINx,BLE_RDYN))
    {
        if (!aci_queue_is_empty(&aci_tx_q))
        {
			pin_state(BLE_REQN_PORTx,BLE_REQN,LO);		// REQN Enabled;
        }
        return;
    }

    // Receive from queue
    if (!aci_queue_dequeue(&aci_tx_q, &data_to_send))
    {
        // Queue was empty, nothing to send
        data_to_send.status_byte = 0;
        data_to_send.buffer[0] = 0;
    }

    // Receive and/or transmit data
    m_aci_spi_transfer(&data_to_send, &received_data);

    // If there are messages to transmit, and we can store the reply, we request a new transfer
    if (!aci_queue_is_full(&aci_rx_q) && !aci_queue_is_empty(&aci_tx_q))
    {
		pin_state(BLE_REQN_PORTx,BLE_REQN,LO);	// REQN Enabled;
    }

    // Check if we received data
    if (received_data.buffer[0] > 0)
    {
        if (!aci_queue_enqueue(&aci_rx_q, &received_data))
        {
            // Receive Buffer full (should never happen)
            while(1);
        }
    }
    return;
}

/***************************************************************************
 * @prototype       static bool m_aci_spi_transfer(hal_aci_data_t * data_to_send, hal_aci_data_t * received_data)
 * @description
 * @param
 * @return
 ***************************************************************************/
static bool m_aci_spi_transfer(hal_aci_data_t * data_to_send, hal_aci_data_t * received_data)
{
    uint8_t byte_cnt;
    uint8_t byte_sent_cnt;
    uint8_t max_bytes;

    pin_state(BLE_REQN_PORTx,BLE_REQN,LO);		// REQN Enabled;

    // Send length, receive header
    byte_sent_cnt = 0;
    received_data->status_byte = SPIExchangeByte(data_to_send->buffer[byte_sent_cnt++]);
    // Send first byte, receive length from slave
    received_data->buffer[0] = SPIExchangeByte(data_to_send->buffer[byte_sent_cnt++]);
    if (0 == data_to_send->buffer[0])
    {
        max_bytes = received_data->buffer[0];
    }
    else
    {
        // Set the maximum to the biggest size. One command byte is already sent
        max_bytes = (received_data->buffer[0] > (data_to_send->buffer[0] - 1))
                                              ? received_data->buffer[0]
                                              : (data_to_send->buffer[0] - 1);
    }

    if (max_bytes > HAL_ACI_MAX_LENGTH)
    {
        max_bytes = HAL_ACI_MAX_LENGTH;
    }

    // Transmit/receive the rest of the packet
    for (byte_cnt = 0; byte_cnt < max_bytes; byte_cnt++)
    {
        received_data->buffer[byte_cnt+1] =  SPIExchangeByte(data_to_send->buffer[byte_sent_cnt++]);
    }

	pin_state(BLE_REQN_PORTx,BLE_REQN,HI);	// RDYN should follow the REQN line in approx 100ns

    return (max_bytes > 0);
}

/***************************************************************************
 * @prototype       void hal_aci_tl_init(void)
 * @description     ACI Transport Layer initialization
 * @param           None
 * @return          None
 ***************************************************************************/
void hal_aci_tl_init(void)
{
    // Initialize SPI (Master)
    SPI_MasterInit();

    // Initialize the ACI Command queue. This must be called after the delay above.
    aci_queue_init(&aci_tx_q);
    aci_queue_init(&aci_rx_q);
	
    // Configure the I/O lines
	pin_state(BLE_RDYN_DDRx,BLE_RDYN,IN);
	pin_state(BLE_RDYN_PORTx,BLE_RDYN,PU);
	pin_state(BLE_REQN_DDRx,BLE_REQN,OUT);
	pin_state(BLE_CE_DDRx,BLE_CE,IN);
    // Debug output
    //pin_state(LED_DDRx,LED_PINx,OUT);

    // Pin reset the nRF8001, required when the nRF8001 setup is being changed
	pin_state(BLE_RST_DDRx,BLE_RST,OUT);
	pin_state(BLE_RST_PORTx,BLE_RST,HI);
	pin_state(BLE_RST_PORTx,BLE_RST,LO);
	pin_state(BLE_RST_PORTx,BLE_RST,HI);

    // Set the nRF8001 to a known state as required by the datasheet
	pin_state(SPI_PORTx,MISO,LO);
	pin_state(SPI_PORTx,MOSI,LO);
	pin_state(BLE_REQN_PORTx,BLE_REQN,HI);
	pin_state(SPI_PORTx,SCK,LO);
    
    // Wait for the nRF8001 to get hold of its lines - the lines float for a few ms after the reset
    Delay(30);
}

/***************************************************************************
 * @prototype       bool hal_aci_tl_send(hal_aci_data_t *p_aci_cmd)
 * @description     Sends an ACI command to the radio.
 * @param           aci_buffer Pointer to the message to send.
 * @return          True if the data was successfully queued for sending,
 *                  false if there is no more space to store messages to send.
 ***************************************************************************/
bool hal_aci_tl_send(hal_aci_data_t *p_aci_cmd)
{
    //const uint8_t length = p_aci_cmd->buffer[0];
    uint8_t length = p_aci_cmd->buffer[0];
    bool ret_val = false;

    if (length > HAL_ACI_MAX_LENGTH)
    {
        return false;
    }

    ret_val = aci_queue_enqueue(&aci_tx_q, p_aci_cmd);
    if (ret_val)
    {
        if(!aci_queue_is_full(&aci_rx_q))
        {
            pin_state(BLE_REQN_PORTx,BLE_REQN,LO);		// REQN Enabled;
        }
#if _DEBUG_UART
            USARTPuts("\rACI_CMD ");
            Delay(_DEBUG_DELAY_MSG);
            m_aci_data_print(p_aci_cmd);
#endif
    }

    return ret_val;
}

/***************************************************************************
 * @prototype       bool hal_aci_tl_event_get(hal_aci_data_t *p_aci_data)
 * @description     Get an ACI event from the event queue
 * @param           
 * @return          
 ***************************************************************************/
bool hal_aci_tl_event_get(hal_aci_data_t *p_aci_data)
{
    // Check if queue is not full
    if (!aci_queue_is_full(&aci_rx_q))
    {
        m_aci_event_check();
    }
    if (aci_queue_dequeue(&aci_rx_q, p_aci_data))
    {
#if _DEBUG_UART
        USARTPuts("\rACI_EVENT ");
        Delay(_DEBUG_DELAY_MSG);
        m_aci_data_print(p_aci_data);
#endif
        // Attempt to pull REQN LOW since we've made room for new messages
        if (!aci_queue_is_full(&aci_rx_q) && !aci_queue_is_empty(&aci_tx_q))
        {
			pin_state(BLE_REQN_PORTx,BLE_REQN,LO);		// REQN Enabled;
        }
        return true;
    }
    return false;
}

/***************************************************************************
 * @prototype       bool hal_aci_tl_event_peek(hal_aci_data_t *p_aci_data)
 * @description     Peek an ACI event from the event queue
 * @param
 * @return
 ***************************************************************************/
bool hal_aci_tl_event_peek(hal_aci_data_t *p_aci_data)
{
    m_aci_event_check();

    if (aci_queue_peek(&aci_rx_q, p_aci_data))
    {
        return true;
    }

    return false;
}

/***************************************************************************
 * @prototype       bool hal_aci_tl_rx_q_full (void)
 * @description     Return full status of transmit queue
 * @param           None
 * @return
 ***************************************************************************/
bool hal_aci_tl_rx_q_full (void)
{
    return aci_queue_is_full(&aci_rx_q);
}

/***************************************************************************
 * @prototype       bool hal_aci_tl_rx_q_empty (void)
 * @description     Return empty status of receive queue
 * @param           None
 * @return
 ***************************************************************************/
bool hal_aci_tl_rx_q_empty (void)
{
    return aci_queue_is_empty(&aci_rx_q);
}

/***************************************************************************
 * @prototype       bool hal_aci_tl_tx_q_full (void)
 * @description     Return full status of receive queue
 * @param           None
 * @return
 ***************************************************************************/
bool hal_aci_tl_tx_q_full (void)
{
    return aci_queue_is_full(&aci_tx_q);
}

/***************************************************************************
 * @prototype       bool hal_aci_tl_tx_q_empty (void)
 * @description     Return empty status of transmit queue
 * @param           None
 * @return
 ***************************************************************************/
bool hal_aci_tl_tx_q_empty (void)
{
    return aci_queue_is_empty(&aci_tx_q);
}

/***************************************************************************
 * @prototype       void hal_aci_tl_q_flush (void)
 * @description     Flush the ACI command Queue and the ACI Event Queue
 * @param           None
 * @return          None
 ***************************************************************************/
void hal_aci_tl_q_flush (void)
{
    cli();	// Disable interrupts
    /* re-initialize aci cmd queue and aci event queue to flush them*/
    aci_queue_init(&aci_tx_q);
    aci_queue_init(&aci_rx_q);
    sei();	// Enable interrupts
}
