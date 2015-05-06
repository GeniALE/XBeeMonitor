/* *************************************************************************
 * lib_aci.c
 *
 * Created on 2014-05-29.
 * Copyright Rogue Research 2014. All rights reserved.
 *
 * This file is encoded as UTF-8.
 *
 * $Id: lib_aci.c 24481 2014-06-03 19:26:29Z marcandre $
 * *************************************************************************
 * Description:     Implementation of the ACI library
 *                  Adapted from Nordic Semiconductor SDK v1.7
 * *************************************************************************/

/** @file
  @brief Implementation of the ACI library.
 */

// Header files ------------------------------------------------------------
#include "hal_platform.h"
#include "aci.h"
#include "aci_cmds.h"
#include "aci_evts.h"
#include "aci_protocol_defines.h"
#include "acilib_defs.h"
#include "acilib_if.h"
#include "hal_aci_tl.h"
#include "aci_queue.h"
#include "lib_aci.h"

// Private defines ---------------------------------------------------------
#define LIB_ACI_DEFAULT_CREDIT_NUMBER   1

// Private variables -------------------------------------------------------
hal_aci_data_t  msg_to_send;    // Global additionally used used in aci_setup

static services_pipe_type_mapping_t * p_services_pipe_type_map;
static hal_aci_data_t *               p_setup_msgs;

static bool is_request_operation_pending;
static bool is_indicate_operation_pending;
static bool is_open_remote_pipe_pending;
static bool is_close_remote_pipe_pending;

static uint8_t request_operation_pipe = 0;
static uint8_t indicate_operation_pipe = 0;

// The following structure (aci_cmd_params_open_adv_pipe) will be used to store the complete command
// including the pipes to be opened.
static aci_cmd_params_open_adv_pipe_t aci_cmd_params_open_adv_pipe;

// Public variables --------------------------------------------------------
extern aci_queue_t    aci_rx_q;
extern aci_queue_t    aci_tx_q;

/***************************************************************************
 * @prototype       bool lib_aci_is_pipe_available(aci_state_t *aci_stat, uint8_t pipe)
 * @description     Checks if a given pipe is available
 * @param           ACI state structure
 *                  pipe: Pipe to check
 * @return          True if the pipe is available, otherwise false
 ***************************************************************************/
bool lib_aci_is_pipe_available(aci_state_t *aci_stat, uint8_t pipe)
{
    uint8_t byte_idx;

    byte_idx = pipe / 8;
    if (aci_stat->pipes_open_bitmap[byte_idx] & (0x01 << (pipe % 8)))
    {
        return(true);
    }
    return(false);
}

/***************************************************************************
 * @prototype       bool lib_aci_is_pipe_closed(aci_state_t *aci_stat, uint8_t pipe)
 * @description     Checks if a given pipe is closed
 * @param           ACI state structure
 *                  pipe: Pipe to check
 * @return          True if the pipe is close, otherwise false
 ***************************************************************************/
bool lib_aci_is_pipe_closed(aci_state_t *aci_stat, uint8_t pipe)
{
    uint8_t byte_idx;

    byte_idx = pipe / 8;
    if (aci_stat->pipes_closed_bitmap[byte_idx] & (0x01 << (pipe % 8)))
    {
        return(true);
    }
    return(false);
}

/***************************************************************************
 * @prototype       bool lib_aci_is_discovery_finished(aci_state_t *aci_stat)
 * @description     Checks if the discovery operation is finished.
 * @param           ACI state structure
 * @return          True if the discovery is finished
 ***************************************************************************/
bool lib_aci_is_discovery_finished(aci_state_t *aci_stat)
{
    return(aci_stat->pipes_open_bitmap[0]&0x01);
}

/***************************************************************************
 * @prototype       void lib_aci_board_init(aci_state_t *aci_stat)
 * @description     Initialization function
 *                  This function shall be used to initialize/reset ACI Library and also Resets the
 *                  nRF8001 by togging the reset pin of the nRF8001. This function will reset
 *                  all the variables locally used by ACI library to their respective default values
 * @param           ACI state structure
 * @return          None
 ***************************************************************************/
void lib_aci_board_init(aci_state_t *aci_stat)
{
    hal_aci_evt_t *aci_data = NULL;
    aci_data = (hal_aci_evt_t *)&msg_to_send;

    // Send the soft reset command to the nRF8001 to get the nRF8001 to a known state.
    lib_aci_radio_reset();

    while (1)
    {
        // Wait for the command response of the radio reset command as the nRF8001 will be in
        // either SETUP or STANDBY after the ACI Reset Radio is processed
        if (true == lib_aci_event_get(aci_stat, aci_data))
        {
            aci_evt_t * aci_evt;
            aci_evt = &(aci_data->evt);

            if (ACI_EVT_CMD_RSP == aci_evt->evt_opcode)
            {
                if (ACI_STATUS_ERROR_DEVICE_STATE_INVALID == aci_evt->params.cmd_rsp.cmd_status) //in SETUP
                {
                    // Inject a Device Started Event Setup to the ACI Event Queue
                    msg_to_send.buffer[0] = 4;    // Length
                    msg_to_send.buffer[1] = 0x81; // Device Started Event
                    msg_to_send.buffer[2] = 0x02; // Setup
                    msg_to_send.buffer[3] = 0;    // Hardware Error -> None
                    msg_to_send.buffer[4] = 2;    // Data Credit Available
                    aci_queue_enqueue(&aci_rx_q, &msg_to_send);
                }
                else if (ACI_STATUS_SUCCESS == aci_evt->params.cmd_rsp.cmd_status) //We are now in STANDBY
                {
                    // Inject a Device Started Event Standby to the ACI Event Queue
                    msg_to_send.buffer[0] = 4;    // Length
                    msg_to_send.buffer[1] = 0x81; // Device Started Event
                    msg_to_send.buffer[2] = 0x03; // Standby
                    msg_to_send.buffer[3] = 0;    // Hardware Error -> None
                    msg_to_send.buffer[4] = 2;    // Data Credit Available
                    aci_queue_enqueue(&aci_rx_q, &msg_to_send);
                }
                else if (ACI_STATUS_ERROR_CMD_UNKNOWN == aci_evt->params.cmd_rsp.cmd_status) //We are now in TEST
                {
                    //Inject a Device Started Event Test to the ACI Event Queue
                    msg_to_send.buffer[0] = 4;    // Length
                    msg_to_send.buffer[1] = 0x81; // Device Started Event
                    msg_to_send.buffer[2] = 0x01; // Test
                    msg_to_send.buffer[3] = 0;    // Hardware Error -> None
                    msg_to_send.buffer[4] = 0;    // Data Credit Available
                    aci_queue_enqueue(&aci_rx_q, &msg_to_send);
                }
                break;  // Break out of the while loop
            }
            else
            {
                // Discard any other ACI Events
            }
        }
    }
}

/***************************************************************************
 * @prototype       void lib_aci_init(aci_state_t *aci_stat)
 * @description     Initialization function
 *                  This function shall be used to initialize/reset ACI Library and also Resets the
 *                  nRF8001 by togging the reset pin of the nRF8001. This function will reset
 *                  all the variables locally used by ACI library to their respective default values
 * @param           ACI state structure
 * @return          None
 ***************************************************************************/
void lib_aci_init(aci_state_t *aci_stat)
{
    uint8_t i;

    for (i = 0; i < PIPES_ARRAY_SIZE; i++)
    {
        aci_stat->pipes_open_bitmap[i]          = 0;
        aci_stat->pipes_closed_bitmap[i]        = 0;
        aci_cmd_params_open_adv_pipe.pipes[i]   = 0;
    }

    is_request_operation_pending     = false;
    is_indicate_operation_pending    = false;
    is_open_remote_pipe_pending      = false;
    is_close_remote_pipe_pending     = false;

    request_operation_pipe           = 0;
    indicate_operation_pipe          = 0;

    p_services_pipe_type_map = aci_stat->aci_setup_info.services_pipe_type_mapping;
    p_setup_msgs             = aci_stat->aci_setup_info.setup_msgs;

    // TODO: lib_aci_board_init(aci_stat);
}

/***************************************************************************
 * @prototype       uint8_t lib_aci_get_nb_available_credits(aci_state_t *aci_stat)
 * @description     Gets the number of currently available ACI credits
 * @param           ACI state structure
 * @return          Number of ACI credits
 ***************************************************************************/
uint8_t lib_aci_get_nb_available_credits(aci_state_t *aci_stat)
{
    return aci_stat->data_credit_available;
}

/***************************************************************************
 * @prototype       uint16_t lib_aci_get_cx_interval_ms(aci_state_t *aci_stat)
 * @description     Gets the connection interval in milliseconds
 * @param           ACI state structure
 * @return          Connection interval in milliseconds
 ***************************************************************************/
uint16_t lib_aci_get_cx_interval_ms(aci_state_t *aci_stat)
{
    uint32_t cx_rf_interval_ms_32bits;
    uint16_t cx_rf_interval_ms;

    cx_rf_interval_ms_32bits  = aci_stat->connection_interval;
    cx_rf_interval_ms_32bits *= 125;                      // the connection interval is given in multiples of 0.125 milliseconds
    cx_rf_interval_ms         = cx_rf_interval_ms_32bits / 100;

    return cx_rf_interval_ms;
}

/***************************************************************************
 * @prototype       uint16_t lib_aci_get_cx_interval(aci_state_t *aci_stat)
 * @description     Gets the connection interval in multiple of 1.25&nbsp;ms
 * @param           ACI state structure
 * @return          Connection interval in multiple of 1.25&nbsp;ms
 ***************************************************************************/
uint16_t lib_aci_get_cx_interval(aci_state_t *aci_stat)
{
    return aci_stat->connection_interval;
}

/***************************************************************************
 * @prototype       uint16_t lib_aci_get_slave_latency(aci_state_t *aci_stat)
 * @description     Gets the current slave latency
 * @param           ACI state structure
 * @return          Current slave latency
 ***************************************************************************/
uint16_t lib_aci_get_slave_latency(aci_state_t *aci_stat)
{
    return aci_stat->slave_latency;
}

/***************************************************************************
 * @prototype       bool lib_aci_set_app_latency(uint16_t latency, aci_app_latency_mode_t latency_mode)
 * @description     Sets a given application latency
 *                  Sends a @c setApplicationLatency command
 * @param           latency: Latency to set
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_set_app_latency(uint16_t latency, aci_app_latency_mode_t latency_mode)
{
    aci_cmd_params_set_app_latency_t aci_set_app_latency;

    aci_set_app_latency.mode    = latency_mode;
    aci_set_app_latency.latency = latency;
    acil_encode_cmd_set_app_latency(&(msg_to_send.buffer[0]), &aci_set_app_latency);

    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_test(aci_test_mode_change_t enter_exit_test_mode)
 * @description     Sets the radio in test mode
 *                  This function sends a @c Test command to the radio. There are two
 *                  Test modes available:
 *                  - UART: DTM commands are received over UART
 *                  - ACI: DTM commands are received over ACI
 *                  The same command is used to exit the test mode When receiving
 *                  a @c DeviceStartedEvent the radio has entered the new mode
 * @param           enter_exit_test_mode: Enter a Test mode, or exit Test mode
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_test(aci_test_mode_change_t enter_exit_test_mode)
{
    aci_cmd_params_test_t aci_cmd_params_test;
    aci_cmd_params_test.test_mode_change = enter_exit_test_mode;
    acil_encode_cmd_set_test_mode(&(msg_to_send.buffer[0]), &aci_cmd_params_test);
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_sleep(void)
 * @description     Sets the radio in sleep mode
 *                  The function sends a @c sleep command to the radio.
 *                  If the radio is advertising or connected, it sends back an error, then use lib_aci_radio_reset
 *                  if advertising or disconnect if in a connection.
 * @param           None
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_sleep(void)
{
    acil_encode_cmd_sleep(&(msg_to_send.buffer[0]));
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_radio_reset(void)
 * @description     Resets the radio
 *                  The function sends a @c BasebandReset command to the radio
 * @param           None
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_radio_reset(void)
{
    acil_encode_baseband_reset(&(msg_to_send.buffer[0]));
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_direct_connect(void)
 * @description     Radio starts directed advertising to bonded device
 *                  The function sends a @c DirectedConnect command to the radio
 * @param           None
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_direct_connect(void)
{
    acil_encode_direct_connect(&(msg_to_send.buffer[0]));
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_device_version(void)
 * @description     Gets the device address
 *                  This function sends a @c GetDeviceAddress command
 * @param           None
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_device_version(void)
{
    acil_encode_cmd_get_device_version(&(msg_to_send.buffer[0]));
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_set_local_data(aci_state_t *aci_stat, uint8_t pipe, uint8_t *p_value, uint8_t size)
 * @description     Sets Local Data
 *                  This command updates the value of the characteristic value or the characteristic descriptor stored locally on the device
 *                  Can be called for all types of pipes as long as the data is stored locally
 * @param           ACI state structure
 *                  pipe: Pipe number on which the data should be set
 *                  value: Pointer to the data to set
 *                  size: Size of the data to set
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_set_local_data(aci_state_t *aci_stat, uint8_t pipe, uint8_t *p_value, uint8_t size)
{
    aci_cmd_params_set_local_data_t aci_cmd_params_set_local_data;

    if ((p_services_pipe_type_map[pipe-1].location != ACI_STORE_LOCAL) || (size > ACI_PIPE_TX_DATA_MAX_LEN))
    {
        return false;
    }
    aci_cmd_params_set_local_data.tx_data.pipe_number = pipe;
    memcpy(&(aci_cmd_params_set_local_data.tx_data.aci_data[0]), p_value, size);
    acil_encode_cmd_set_local_data(&(msg_to_send.buffer[0]), &aci_cmd_params_set_local_data, size);
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_connect(uint16_t run_timeout, uint16_t adv_interval)
 * @description     Tries to connect to a peer device
 *                  This function sends a @c Connect command to the radio.
 * @param           run_timeout: Maximum advertising time in seconds (0 means infinite).
 *                  adv_interval: Advertising interval (in multiple of 0.625&nbsp;ms).
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_connect(uint16_t run_timeout, uint16_t adv_interval)
{
    aci_cmd_params_connect_t aci_cmd_params_connect;
    aci_cmd_params_connect.timeout      = run_timeout;
    aci_cmd_params_connect.adv_interval = adv_interval;
    acil_encode_cmd_connect(&(msg_to_send.buffer[0]), &aci_cmd_params_connect);
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_disconnect(aci_state_t *aci_stat, aci_disconnect_reason_t reason)
 * @description     Disconnects from peer device.
 *                  This function sends a @c Disconnect command to the radio
 * @param           ACI state structure
 *                  reason: Reason for disconnecting
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_disconnect(aci_state_t *aci_stat, aci_disconnect_reason_t reason)
{
    bool ret_val;
    uint8_t i;
    aci_cmd_params_disconnect_t aci_cmd_params_disconnect;
    aci_cmd_params_disconnect.reason = reason;
    acil_encode_cmd_disconnect(&(msg_to_send.buffer[0]), &aci_cmd_params_disconnect);
    ret_val = hal_aci_tl_send(&msg_to_send);
    // If we have actually sent the disconnect
    if (ret_val)
    {   // Update pipes immediately so that while the disconnect is happening,
        // the application can't attempt sending another message
        // If the application sends another message before we updated this
        // a ACI Pipe Error Event will be received from nRF8001
        for (i=0; i < PIPES_ARRAY_SIZE; i++)
        {
            aci_stat->pipes_open_bitmap[i] = 0;
            aci_stat->pipes_closed_bitmap[i] = 0;
        }
    }
    return ret_val;
}

/***************************************************************************
 * @prototype       bool lib_aci_bond(uint16_t run_timeout, uint16_t adv_interval)
 * @description     Tries to bond with a peer device
 *                  This function sends a @c Bond command to the radio.
 * @param           run_timeout: Maximum advertising time in seconds (0 means infinite)
 *                  adv_interval: Advertising interval (in multiple of 0.625&nbsp;ms)
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_bond(uint16_t run_timeout, uint16_t adv_interval)
{
    aci_cmd_params_bond_t aci_cmd_params_bond;
    aci_cmd_params_bond.timeout = run_timeout;
    aci_cmd_params_bond.adv_interval = adv_interval;
    acil_encode_cmd_bond(&(msg_to_send.buffer[0]), &aci_cmd_params_bond);
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_wakeup(void)
 * @description     Wakes up the radio
 *                  This function sends a @c Wakeup command to wake up the radio from
 *                  sleep mode. When woken up the radio sends a @c DeviceStartedEvent and
 *                  a @c CommandResponseEvent
 * @param           None
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_wakeup(void)
{
    acil_encode_cmd_wakeup(&(msg_to_send.buffer[0]));
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_set_tx_power(aci_device_output_power_t tx_power)
 * @description     Sets the radio's TX power
 *                  This function sends a @c SetTxPower command.
 * @param           tx_power: TX power to be used by the radio
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_set_tx_power(aci_device_output_power_t tx_power)
{
    aci_cmd_params_set_tx_power_t aci_cmd_params_set_tx_power;
    aci_cmd_params_set_tx_power.device_power = tx_power;
    acil_encode_cmd_set_radio_tx_power(&(msg_to_send.buffer[0]), &aci_cmd_params_set_tx_power);
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_get_address(void)
 * @description     Gets the device address
 *                  This function sends a @c GetDeviceAddress command
 * @param           None
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_get_address(void)
{
    acil_encode_cmd_get_address(&(msg_to_send.buffer[0]));
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_get_temperature(void)
 * @description     Gets the temperature
 *                  This function sends a @c GetTemperature command. lib_aci
 *                  calls the @ref lib_aci_transaction_finished_hook() function when the temperature is received
 * @param           None
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_get_temperature(void)
{
    acil_encode_cmd_temparature(&(msg_to_send.buffer[0]));
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_get_battery_level(void)
 * @description     Gets the battery level
 *                  This function sends a @c GetBatteryLevel command
 * @param           None
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_get_battery_level(void)
{
  acil_encode_cmd_battery_level(&(msg_to_send.buffer[0]));
  return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_send_data(uint8_t pipe, uint8_t *p_value, uint8_t size)
 * @description     Sends data on a given pipe
 *                  This function sends a @c SendData command with application data to
 *                  the radio. This function memorizes credit use, and checks that
 *                  enough credits are available
 * @param           pipe: Pipe number on which the data should be sent
 *                  value: Pointer to the data to send
 *                  size: Size of the data to send
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_send_data(uint8_t pipe, uint8_t *p_value, uint8_t size)
{
    bool ret_val = false;
    aci_cmd_params_send_data_t aci_cmd_params_send_data;

    if(!((p_services_pipe_type_map[pipe-1].pipe_type == ACI_TX) ||
        (p_services_pipe_type_map[pipe-1].pipe_type == ACI_TX_ACK)))
    {
        return false;
    }

    if (size > ACI_PIPE_TX_DATA_MAX_LEN)
    {
        return false;
    }
    {
        aci_cmd_params_send_data.tx_data.pipe_number = pipe;
        memcpy(&(aci_cmd_params_send_data.tx_data.aci_data[0]), p_value, size);
        acil_encode_cmd_send_data(&(msg_to_send.buffer[0]), &aci_cmd_params_send_data, size);

        ret_val = hal_aci_tl_send(&msg_to_send);
    }
    return ret_val;
}

/***************************************************************************
 * @prototype       bool lib_aci_request_data(aci_state_t *aci_stat, uint8_t pipe)
 * @description     Requests data from a given pipe
 *                  This function sends a @c RequestData command to the radio. This
 *                  function memorizes credit uses, and check that enough credits are available.
 *                  After this command, the radio sends back either a @c DataReceivedEvent
 *                  or a @c PipeErrorEvent
 * @param           ACI state structure
 *                  pipe: Pipe number on which the data is requested
 * @return          True if an ACI Event was copied to the pointer.
 ***************************************************************************/
bool lib_aci_request_data(aci_state_t *aci_stat, uint8_t pipe)
{
    bool ret_val = false;
    aci_cmd_params_request_data_t aci_cmd_params_request_data;

    if(!((p_services_pipe_type_map[pipe-1].location == ACI_STORE_REMOTE)&&(p_services_pipe_type_map[pipe-1].pipe_type == ACI_RX_REQ)))
    {
    return false;
    }


    {

    {
      aci_cmd_params_request_data.pipe_number = pipe;
      acil_encode_cmd_request_data(&(msg_to_send.buffer[0]), &aci_cmd_params_request_data);

      ret_val = hal_aci_tl_send(&msg_to_send);
    }
    }
    return ret_val;
}

/***************************************************************************
 * @prototype       bool lib_aci_change_timing(uint16_t minimun_cx_interval, uint16_t maximum_cx_interval, uint16_t slave_latency, uint16_t timeout)
 * @description     Sends a L2CAP change connection parameters request.
 *                  This function sends a @c ChangeTiming command to the radio.  This command triggers a "L2CAP change connection parameters" request
 *                  to the master. If the master rejects or accepts but doesn't change the connection parameters within
 *                  30 seconds, a timing event with the unchanged connection parameters is sent by the radio.
 *                  If the request is accepted and the master changes connection parameters, a timing event with
 *                  the new connection parameters is sent by the radio.
 *                  If the master doesn't reply to the request within 60 seconds, the radio disconnects
 * @param           minimun_cx_interval: Minimum connection interval requested, in multiple of 1.25&nbsp;ms.
 *                  maximum_cx_interval: Maximum connection interval requested, in multiple of 1.25&nbsp;ms.
 *                  slave_latency: requested slave latency.
 *                  timeout: requested slave timeout, in multiple of 10&nbsp;ms.
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_change_timing(uint16_t minimun_cx_interval, uint16_t maximum_cx_interval, uint16_t slave_latency, uint16_t timeout)
{
    aci_cmd_params_change_timing_t aci_cmd_params_change_timing;
    aci_cmd_params_change_timing.conn_params.min_conn_interval = minimun_cx_interval;
    aci_cmd_params_change_timing.conn_params.max_conn_interval = maximum_cx_interval;
    aci_cmd_params_change_timing.conn_params.slave_latency     = slave_latency;
    aci_cmd_params_change_timing.conn_params.timeout_mult      = timeout;
    acil_encode_cmd_change_timing_req(&(msg_to_send.buffer[0]), &aci_cmd_params_change_timing);
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_change_timing_GAP_PPCP(void)
 * @description     Sends a L2CAP change connection parameters request with the connection predefined preffered connection parameters.
 *                  This function sends a @c ChangeTiming command to the radio. This command triggers a "L2CAP change connection parameters" request
 *                  to the master. If the master rejects or accepts but doesn't change the connection parameters within
 *                  30 seconds, a timing event with the unchanged connection parameters is sent by the radio.
 *                  If the request is accepted and the master changes connection parameters, a timing event with
 *                  the new connection parameters is sent by the radio.
 *                  If the master doesn't reply to the request within 60 seconds, the radio disconnects.
 *                  The timing parameters used are the Timing parameters in the GAP settings in the nRFgo Studio.
 *                  The Timing parameters as stored as the GAP Preferred Peripheral Connection Parameters.
 * @param           None
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_change_timing_GAP_PPCP(void)
{
    acil_encode_cmd_change_timing_req_GAP_PPCP(&(msg_to_send.buffer[0]));
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_open_remote_pipe(aci_state_t *aci_stat, uint8_t pipe)
 * @description     Opens a remote pipe
 *                  This function sends an @c OpenRemotePipe command
 * @param           ACI state structure
 *                  pipe Number of the pipe to open
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_open_remote_pipe(aci_state_t *aci_stat, uint8_t pipe)
{
    bool ret_val = false;
    aci_cmd_params_open_remote_pipe_t aci_cmd_params_open_remote_pipe;

    if(!((p_services_pipe_type_map[pipe-1].location == ACI_STORE_REMOTE)&&
        ((p_services_pipe_type_map[pipe-1].pipe_type == ACI_RX)||
         (p_services_pipe_type_map[pipe-1].pipe_type == ACI_RX_ACK_AUTO)||
         (p_services_pipe_type_map[pipe-1].pipe_type == ACI_RX_ACK))))
    {
        return false;
    }
  
    {
        is_request_operation_pending = true;
        is_open_remote_pipe_pending = true;
        request_operation_pipe = pipe;
        aci_cmd_params_open_remote_pipe.pipe_number = pipe;
        acil_encode_cmd_open_remote_pipe(&(msg_to_send.buffer[0]), &aci_cmd_params_open_remote_pipe);
        ret_val = hal_aci_tl_send(&msg_to_send);
    }
    return ret_val;
}

/***************************************************************************
 * @prototype       bool lib_aci_close_remote_pipe(aci_state_t *aci_stat, uint8_t pipe)
 * @description     Closes a remote pipe
 *                  This function sends an @c CloseRemotePipe command
 * @param           ACI state structure
 *                  pipe: Pipe number to close
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_close_remote_pipe(aci_state_t *aci_stat, uint8_t pipe)
{
    bool ret_val = false;
    aci_cmd_params_close_remote_pipe_t aci_cmd_params_close_remote_pipe;

    if(!((p_services_pipe_type_map[pipe-1].location == ACI_STORE_REMOTE)&&
        ((p_services_pipe_type_map[pipe-1].pipe_type == ACI_RX)||
         (p_services_pipe_type_map[pipe-1].pipe_type == ACI_RX_ACK_AUTO)||
         (p_services_pipe_type_map[pipe-1].pipe_type == ACI_RX_ACK))))
    {
        return false;
    }

    {
        is_request_operation_pending = true;
        is_close_remote_pipe_pending = true;
        request_operation_pipe = pipe;
        aci_cmd_params_close_remote_pipe.pipe_number = pipe;
        acil_encode_cmd_close_remote_pipe(&(msg_to_send.buffer[0]), &aci_cmd_params_close_remote_pipe);
        ret_val = hal_aci_tl_send(&msg_to_send);
    }
    return ret_val;
}

/***************************************************************************
 * @prototype       bool lib_aci_set_key(aci_key_type_t key_rsp_type, uint8_t *key, uint8_t len)
 * @description     Set the key requested by the 8001.
 *                  This function sends an @c SetKey command to the radio.
 * @param           key_rsp_type Type of key.
 *                  key: Pointer to the key to set.
 *                  len: Length of the key.
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_set_key(aci_key_type_t key_rsp_type, uint8_t *key, uint8_t len)
{
    aci_cmd_params_set_key_t aci_cmd_params_set_key;
    aci_cmd_params_set_key.key_type = key_rsp_type;
    memcpy((uint8_t*)&(aci_cmd_params_set_key.key), key, len);
    acil_encode_cmd_set_key(&(msg_to_send.buffer[0]), &aci_cmd_params_set_key);
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_echo_msg(uint8_t msg_size, uint8_t *p_msg_data)
 * @description     Sends an echo message
 *                  This function sends an @c Echo command to the radio. lib_aci
 *                  places the Echp ACI command in the ACI command queue
 * @param           message_size: Length of the data to send
 *                  message_data: Pointer to the data to send
 * @return          True if an ACI Event was copied to the pointer
 ***************************************************************************/
bool lib_aci_echo_msg(uint8_t msg_size, uint8_t *p_msg_data)
{
    aci_cmd_params_echo_t aci_cmd_params_echo;
    if(msg_size > (ACI_ECHO_DATA_MAX_LEN))
    {
        return false;
    }

    if (msg_size > (ACI_ECHO_DATA_MAX_LEN))
    {
        msg_size = ACI_ECHO_DATA_MAX_LEN;
    }

    memcpy(&(aci_cmd_params_echo.echo_data[0]), p_msg_data, msg_size);
    acil_encode_cmd_echo_msg(&(msg_to_send.buffer[0]), &aci_cmd_params_echo, msg_size);

    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_bond_request(void)
 * @description     Sends a SMP Security Request
 *                  This function send a @c BondRequest command to the radio
 *                  This command triggers a SMP Security Request to the master. If the
 *                  master rejects with a pairing failed or if the bond timer expires the connection is closed
 * @param           None
 * @return          True if the transaction is successfully initiated
 ***************************************************************************/
bool lib_aci_bond_request(void)
{
    acil_encode_cmd_bond_security_request(&(msg_to_send.buffer[0]));
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_event_peek(hal_aci_evt_t *p_aci_evt_data)
 * @description     Peeks an ACI event from the ACI Event Queue
 *                  This function peeks at the top event in the ACI event queue
 *                  In polling mode, this function will query the nRF8001 for pending events, but unlike
 *                  lib_aci_event_get() it will not dequeue the event from the local queue, but will instead
 *                  only peek at it
 * @param
 * @return          True if an ACI Event was copied to the pointer
 ***************************************************************************/
bool lib_aci_event_peek(hal_aci_evt_t *p_aci_evt_data)
{
    return hal_aci_tl_event_peek((hal_aci_data_t *)p_aci_evt_data);
}

/***************************************************************************
 * @prototype       bool lib_aci_event_get(aci_state_t *aci_stat, hal_aci_evt_t *p_aci_evt_data)
 * @description     Gets an ACI event from the ACI Event Queue
 * @param           aci_stat: pointer to the state of the ACI
 *                  p_aci_data: pointer to the ACI Event. The ACI Event received will be copied into this pointer
 * @return          True if an ACI Event was copied to the pointer
 ***************************************************************************/
bool lib_aci_event_get(aci_state_t *aci_stat, hal_aci_evt_t *p_aci_evt_data)
{
    bool status = false;

    status = hal_aci_tl_event_get((hal_aci_data_t *)p_aci_evt_data);

/** Update the state of the ACI with the
    ACI Events -> Pipe Status, Disconnected, Connected, Bond Status, Pipe Error */
    if (true == status)
    {
        aci_evt_t * aci_evt;

        aci_evt = &p_aci_evt_data->evt;

        switch(aci_evt->evt_opcode)
        {
            case ACI_EVT_PIPE_STATUS:
            {
                uint8_t i = 0;

                for (i=0; i < PIPES_ARRAY_SIZE; i++)
                {
                  aci_stat->pipes_open_bitmap[i]   = aci_evt->params.pipe_status.pipes_open_bitmap[i];
                  aci_stat->pipes_closed_bitmap[i] = aci_evt->params.pipe_status.pipes_closed_bitmap[i];
                }
                break;
            }
            case ACI_EVT_DISCONNECTED:
            {
                uint8_t i = 0;

                for (i = 0; i < PIPES_ARRAY_SIZE; i++)
                {
                  aci_stat->pipes_open_bitmap[i] = 0;
                  aci_stat->pipes_closed_bitmap[i] = 0;
                }
                aci_stat->confirmation_pending = false;
                aci_stat->data_credit_available = aci_stat->data_credit_total;
                break;
            }
            case ACI_EVT_TIMING:
                aci_stat->connection_interval = aci_evt->params.timing.conn_rf_interval;
                aci_stat->slave_latency       = aci_evt->params.timing.conn_slave_rf_latency;
                aci_stat->supervision_timeout = aci_evt->params.timing.conn_rf_timeout;
                break;

            default:
                break;
        }
    }
    return status;
}

/***************************************************************************
 * @prototype       bool lib_aci_send_ack(aci_state_t *aci_stat, const uint8_t pipe)
 * @description     Sends acknowledgement message to peer
 *                  This function sends @c SendDataAck command to radio. The radio is expected
 *                  to send either Handle Value Confirmation or Write response depending
 *                  on whether the data is stored remotely or locally
 * @param           ACI state structure
 *                  pipe: Pipe number for which the acknowledgement is to be sent
 * @return          True if the ack was sent successfully. False otherwise
 ***************************************************************************/
bool lib_aci_send_ack(aci_state_t *aci_stat, const uint8_t pipe)
{
    bool ret_val = false;
    {
        acil_encode_cmd_send_data_ack(&(msg_to_send.buffer[0]), pipe);
        ret_val = hal_aci_tl_send(&msg_to_send);
    }
    return ret_val;
}

/***************************************************************************
 * @prototype       bool lib_aci_send_nack(aci_state_t *aci_stat, const uint8_t pipe, const uint8_t error_code)
 * @description     Sends negative acknowledgement message to peer
 *                  This function sends @c SendDataNack command to radio. The radio is expected
 *                  to send Error Response to the peer
 * @param           ACI state structure
 *                  pipe: Pipe number for which the nack is to be sent
 *                  error_code: Error code to be sent in the NACk
 * @return          True if the command was sent successfully through the ACI. False otherwise
 ***************************************************************************/
bool lib_aci_send_nack(aci_state_t *aci_stat, const uint8_t pipe, const uint8_t error_code)
{
    bool ret_val = false;
    {
        acil_encode_cmd_send_data_nack(&(msg_to_send.buffer[0]), pipe, error_code);
        ret_val = hal_aci_tl_send(&msg_to_send);
    }
    return ret_val;
}

/***************************************************************************
 * @prototype       bool lib_aci_broadcast(const uint16_t timeout, const uint16_t adv_interval)
 * @description     Sends Broadcast message to the radio
 *                  The Broadcast message starts advertisement procedure
 *                  using the given interval with the intention of broadcasting data to a peer device
 * @param           timeout Time: In seconds, to advertise before exiting to standby mode (0 means infinite).
 *                      Valid values: 0 to 16383.
 *                  adv_interval: Advertising interval (in multiple of 0.625&nbsp;ms).
 *                      Valid values: 160 to 16384 (which corresponds to an interval from 100 ms to 10.24 s).
 * @return          True if the broadcast message is sent successfully to the radio
 ***************************************************************************/
bool lib_aci_broadcast(const uint16_t timeout, const uint16_t adv_interval)
{
    aci_cmd_params_broadcast_t aci_cmd_params_broadcast;
    if (timeout > 16383)
    {
        return false;
    }

    // The adv_interval should be between 160 and 16384 (which translates to the advertisement
    // interval values 100 ms and 10.24 s.
    if ((160 > adv_interval) || (adv_interval > 16384))
    {
        return false;
    }

    aci_cmd_params_broadcast.timeout = timeout;
    aci_cmd_params_broadcast.adv_interval = adv_interval;
    acil_encode_cmd_broadcast(&(msg_to_send.buffer[0]), &aci_cmd_params_broadcast);
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_open_adv_pipes(const uint8_t * const adv_service_data_pipes)
 * @description     Sends a command to the radio to set the pipes to be placed in Advertisement Service Data.
 *                  This function will send a command to the radio that will set the pipes to be placed in
 *                  advertisement Service Data.  To start advertising service data, this function should be called before
 *                  Connecting, Broadcasting or Bonding to peer. This function can be called during
 *                  advertising to enable/disable broadcast pipes. Use this as an alternative to @ref lib_aci_open_adv_pipe
 *                  to avoid multiple function calls for placing multiple pipes in the adv data.
 * @param           adv_service_data_pipes: Pipe bitmap, where '1' indicates that the corresponding
 *                  Valid Values: 0000000000000000 to FEFFFFFFFFFFFF7F (See the ACI Pipe Status Evt bitmap in the nRF8001 datasheet
 *                  TX_BROADCAST pipe data is to be placed in Advertising Service Data fields
 * @return          True if the Open Adv Pipe message was sent successfully to the radio
 ***************************************************************************/
bool lib_aci_open_adv_pipes(const uint8_t * const adv_service_data_pipes)
{
    uint8_t i;

    for (i = 0; i < PIPES_ARRAY_SIZE; i++)
    {
        aci_cmd_params_open_adv_pipe.pipes[i] = adv_service_data_pipes[i];
    }

    acil_encode_cmd_open_adv_pipes(&(msg_to_send.buffer[0]), &aci_cmd_params_open_adv_pipe);
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_open_adv_pipe(const uint8_t pipe)
 * @description     Sends a command to the radio to set the input pipe to be placed in Advertisement Service Data.
 *                  This function sends a command to the radio that places the pipe in
 *                  advertisement service data.  To start advertising service data, call this function before
 *                  Connecting, Broadcasting or Bonding to peer. The data that should be sent in the advertisement packets
 *                  must be set using the @c lib_aci_set_local_data function. This function can be called during
 *                  advertising to enable/disable broadcast pipes.
 * @param           pipe: The pipe that has to be placed in advertising service data
 * @return          True if the Open Adv Pipe message is sent successfully to the radio
 ***************************************************************************/
bool lib_aci_open_adv_pipe(const uint8_t pipe)
{
    uint8_t byte_idx = pipe / 8;

    aci_cmd_params_open_adv_pipe.pipes[byte_idx] |= (0x01 << (pipe % 8));
    acil_encode_cmd_open_adv_pipes(&(msg_to_send.buffer[0]), &aci_cmd_params_open_adv_pipe);
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_read_dynamic_data(void)
 * @description     Sends ReadDynamicData command to the host
 *                  This function sends @c ReadDynamicData command to host. The host is expected
 *                  to send @c CommandResponse back with the dynamic data. The application is expected to
 *                  call this function in a loop until all the dynamic data is read out from the host.
 *                  As long as there is dynamic data to be read from the host, the command response
 *                  for this message has its status field set to ACI_STATUS_TRANSACTION_CONTINUE (0x01).
 *                  The application may chose to store this read out data in a non-volatile memory location
 *                  and later chose to write it back using the function lib_aci_write_dynamic_data.
 * @param           None
 * @return          True if the command was sent successfully through the ACI. False otherwise
 ***************************************************************************/
bool lib_aci_read_dynamic_data(void)
{
    acil_encode_cmd_read_dynamic_data(&(msg_to_send.buffer[0]));
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_write_dynamic_data(uint8_t sequence_number, uint8_t* dynamic_data, uint8_t length)
 * @description     Sends WriteDynamicData command to the host
 *                  This function sends @c WriteDynamicData command to host. The host is expected
 *                  to send @c CommandResponse with the status of this operation. As long as the status field
 *                  in the @c CommandResponse is ACI_STATUS_TRANSACTION_CONTINUE (0x01), the hosts expects
 *                  more dynamic data to be written. This function should ideally be called in a cycle,
 *                  until all the stored dynamic data is sent to the host. This function should be
 *                  called with the dynamic data obtained from the response to a @c ReadDynamicData
 *                  (see @c lib_aci_read_dynamic_data) command
 * @param           sequence_number: Sequence number of the dynamic data to be sent.
 *                  dynamic_data: Pointer to the dynamic data.
 *                  length: Length of the dynamic data.
 * @return          True if the command was sent successfully through the ACI. False otherwise
 ***************************************************************************/
bool lib_aci_write_dynamic_data(uint8_t sequence_number, uint8_t* dynamic_data, uint8_t length)
{
    acil_encode_cmd_write_dynamic_data(&(msg_to_send.buffer[0]), sequence_number, dynamic_data, length);
    return hal_aci_tl_send(&msg_to_send);
}

/***************************************************************************
 * @prototype       bool lib_aci_dtm_command(uint8_t dtm_command_msbyte, uint8_t dtm_command_lsbyte)
 * @description     Sends an DTM command
 *                  This function sends an @c DTM command to the radio.
 * @param           dtm_command_msbyte: Most significant byte of the DTM command
 *                  dtm_command_lsbyte: Least significant byte of the DTM command
 * @return          True if an ACI Event was copied to the pointer
 ***************************************************************************/
bool lib_aci_dtm_command(uint8_t dtm_command_msbyte, uint8_t dtm_command_lsbyte)
{
    aci_cmd_params_dtm_cmd_t aci_cmd_params_dtm_cmd;
    aci_cmd_params_dtm_cmd.cmd_msb = dtm_command_msbyte;
    aci_cmd_params_dtm_cmd.cmd_lsb = dtm_command_lsbyte;
    acil_encode_cmd_dtm_cmd(&(msg_to_send.buffer[0]), &aci_cmd_params_dtm_cmd);
    return hal_aci_tl_send(&msg_to_send);
}
