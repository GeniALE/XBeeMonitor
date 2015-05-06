/* *************************************************************************
 * aci_setup.h
 *
 * Created on 2014-06-04.
 * Copyright Rogue Research 2014. All rights reserved.
 *
 * This file is encoded as UTF-8.
 *
 * $Id:
 * *************************************************************************
 * Description:     ACI Setup
 *                  Adapted from Nordic Semiconductor SDK v1.7
 * *************************************************************************/
  
#ifndef H_ACI_SETUP
#define H_ACI_SETUP

//**************************************************************************//
//                               Constants                                  //
//**************************************************************************//
#define SETUP_SUCCESS                        0
#define SETUP_FAIL_COMMAND_QUEUE_NOT_EMPTY   1
#define SETUP_FAIL_EVENT_QUEUE_NOT_EMPTY     2
#define SETUP_FAIL_TIMEOUT                   3
#define SETUP_FAIL_NOT_SETUP_EVENT           4
#define SETUP_FAIL_NOT_COMMAND_RESPONSE      5

//**************************************************************************//
//                          Function prototypes                             //
//**************************************************************************//
// Setup the nRF8001 device
uint8_t do_aci_setup(aci_state_t *aci_stat);
// Utility function to fill the the ACI command queue
static bool aci_setup_fill(aci_state_t *aci_stat, uint8_t *num_cmd_offset);

#endif
