/* *************************************************************************
 * aci_queue.c
 *
 * Created on 2014-05-29.
 * Copyright Rogue Research 2014. All rights reserved.
 *
 * This file is encoded as UTF-8.
 *
 * $Id: aci_queue.c 24481 2014-06-03 19:26:29Z marcandre $
 * *************************************************************************
 * Description:     Implementation of a circular queue for ACI data
 *                  Adapted from Nordic Semiconductor SDK v1.7
 * *************************************************************************/

 /** @file
@brief Implementation of a circular queue for ACI data
*/

// Header files
#include "hal_aci_tl.h"
#include "aci_queue.h"
#include "ble_assert.h"

/***************************************************************************
 * @prototype       void aci_queue_init(aci_queue_t *aci_q)
 * @description     Flush the ACI command Queue and the ACI Event Queue
 * @param           
 * @return          None
 ***************************************************************************/
void aci_queue_init(aci_queue_t *aci_q)
{
    uint8_t loop;

    ble_assert(NULL != aci_q);

    aci_q->head = 0;
    aci_q->tail = 0;
    for(loop=0; loop<ACI_QUEUE_SIZE; loop++)
    {
        aci_q->aci_data[loop].buffer[0] = 0x00;
        aci_q->aci_data[loop].buffer[1] = 0x00;
    }
}

/***************************************************************************
 * @prototype       bool aci_queue_dequeue(aci_queue_t *aci_q, hal_aci_data_t *p_data)
 * @description
 * @param
 * @return
 ***************************************************************************/
bool aci_queue_dequeue(aci_queue_t *aci_q, hal_aci_data_t *p_data)
{
    ble_assert(NULL != aci_q);
    ble_assert(NULL != p_data);

    if (aci_queue_is_empty(aci_q))
    {
        return false;
    }

    memcpy((uint8_t *)p_data, (uint8_t *)&(aci_q->aci_data[aci_q->head]), sizeof(hal_aci_data_t));
    aci_q->head = (aci_q->head + 1) % ACI_QUEUE_SIZE;

    return true;
}

/***************************************************************************
 * @prototype       bool aci_queue_enqueue(aci_queue_t *aci_q, hal_aci_data_t *p_data)
 * @description
 * @param
 * @return
 ***************************************************************************/
bool aci_queue_enqueue(aci_queue_t *aci_q, hal_aci_data_t *p_data)
{
    uint8_t length = p_data->buffer[0];

    ble_assert(NULL != aci_q);
    ble_assert(NULL != p_data);

    if (aci_queue_is_full(aci_q))
    {
        return false;
    }

    aci_q->aci_data[aci_q->tail].status_byte = 0;
    memcpy((uint8_t *)&(aci_q->aci_data[aci_q->tail].buffer[0]), (uint8_t *)&p_data->buffer[0], length + 1);
    aci_q->tail = (aci_q->tail + 1) % ACI_QUEUE_SIZE;

    return true;
}

/***************************************************************************
 * @prototype       bool aci_queue_is_empty(aci_queue_t *aci_q)
 * @description     Return empty status of receive queue
 * @param
 * @return
 ***************************************************************************/
bool aci_queue_is_empty(aci_queue_t *aci_q)
{
    bool state = false;

    ble_assert(NULL != aci_q);

    //Critical section
    cli();	// Disable interrupts
    if (aci_q->head == aci_q->tail)
    {
        state = true;
    }
    sei();	// Enable interrupts

    return state;
}

/***************************************************************************
 * @prototype       bool aci_queue_is_full(aci_queue_t *aci_q)
 * @description     Return full status of receive queue
 * @param
 * @return
 ***************************************************************************/
bool aci_queue_is_full(aci_queue_t *aci_q)
{
    uint8_t next;
    bool state;

    ble_assert(NULL != aci_q);

    //This should be done in a critical section
    cli();	// Disable interrupts
    next = (aci_q->tail + 1) % ACI_QUEUE_SIZE;

    if (next == aci_q->head)
    {
        state = true;
    }
    else
    {
        state = false;
    }
    sei();	// Enable interrupts

    return state;
}

/***************************************************************************
 * @prototype       bool aci_queue_peek(aci_queue_t *aci_q, hal_aci_data_t *p_data)
 * @description     Return full status of receive queue
 * @param
 * @return
 ***************************************************************************/
bool aci_queue_peek(aci_queue_t *aci_q, hal_aci_data_t *p_data)
{
    ble_assert(NULL != aci_q);
    ble_assert(NULL != p_data);

    if (aci_queue_is_empty(aci_q))
    {
        return false;
    }

    memcpy((uint8_t *)p_data, (uint8_t *)&(aci_q->aci_data[aci_q->head]), sizeof(hal_aci_data_t));

    return true;
}