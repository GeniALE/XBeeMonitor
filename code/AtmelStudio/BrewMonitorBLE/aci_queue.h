/* *************************************************************************
 * aci_queue.h
 * 
 * Created on 2014-05-29.
 * Copyright Rogue Research 2014. All rights reserved.
 * 
 * This file is encoded as UTF-8.
 *
 * $Id: aci_queue.h 24481 2014-06-03 19:26:29Z marcandre $
 * *************************************************************************
 * Description:     Implementation of a circular queue for ACI data
 *                  Adapted from Nordic Semiconductor SDK v1.7
 * *************************************************************************/

/** @file
 * @brief Interface for buffer.
 */

/** @defgroup aci_queue aci_queue
@{
@ingroup aci_queue
*/

#ifndef ACI_QUEUE_H__
#define ACI_QUEUE_H__

//**************************************************************************//
//                              Header files                                //
//**************************************************************************//
#include "aci.h"
#include "hal_aci_tl.h"

//**************************************************************************//
//                               Constants                                  //
//**************************************************************************//
/* The ACI_QUEUE_SIZE determines the memory usage of the system.
 * Successfully tested to a ACI_QUEUE_SIZE of 4 (interrupt) and 4 (polling)
 */
#define ACI_QUEUE_SIZE  4

//**************************************************************************//
//                               Typedefs                                   //
//**************************************************************************//
/** Data type for queue of data packets to send/receive from radio.
 *
 *  A FIFO queue is maintained for packets. New packets are added (enqueued)
 *  at the tail and taken (dequeued) from the head. The head variable is the
 *  index of the next packet to dequeue while the tail variable is the index
 *  of where the next packet should be queued.
 */
typedef struct
{
    hal_aci_data_t           aci_data[ACI_QUEUE_SIZE];
    uint8_t                  head;
    uint8_t                  tail;
} aci_queue_t;

//**************************************************************************//
//                          Function prototypes                             //
//**************************************************************************//
void aci_queue_init(aci_queue_t *aci_q);
bool aci_queue_dequeue(aci_queue_t *aci_q, hal_aci_data_t *p_data);
bool aci_queue_enqueue(aci_queue_t *aci_q, hal_aci_data_t *p_data);
bool aci_queue_is_empty(aci_queue_t *aci_q);
bool aci_queue_is_full(aci_queue_t *aci_q);
bool aci_queue_peek(aci_queue_t *aci_q, hal_aci_data_t *p_data);

#endif /* ACI_QUEUE_H__ */
/** @} */