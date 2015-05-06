/* *************************************************************************
 * main.c
 *
 * Created on 2014-05-29.
 * Copyright Rogue Research 2014. All rights reserved.
 *
 * This file is encoded as UTF-8.
 *
 * $Id: main.c 24481 2014-06-03 19:26:29Z marcandre $
 * *************************************************************************
 * Device family:   PIC XLP 16F
 * *************************************************************************
 * Description:     Echo project to verify the SPI/ACI connectivity
 *                  This project is a test project to verify the SPI/ACI.
 *                  The data in the ACI echo command send and the data
 *                  received in the ACI echo event should be the same.
 * *************************************************************************/

// Header files ------------------------------------------------------------
#include "lib_aci.h"
#include "services.h"
#include "hal_platform.h"
#include "inc/user.h"       // User config file

// Private variables -------------------------------------------------------
/* aci_struct that will contain:
 * total initial credits
 * current credit
 * current state of the aci (setup/standby/active/sleep)
 * open remote pipe pending
 * close remote pipe pending
 * Current pipe available bitmap
 * Current pipe closed bitmap
 * Current connection interval, slave latency and link supervision timeout
 * Current State of the the GATT client (Service Discovery)
 * Status of the bond (R) Peer address */
static struct aci_state_t aci_state;
static hal_aci_evt_t aci_data;
static uint8_t echo_data[] = { 0x00, 0xaa, 0x55, 0xff, 0x77, 0x55, 0x33, 0x22, 0x11, 0x44, 0x66, 0x88, 0x99, 0xbb, 0xdd, 0xcc, 0x00, 0xaa, 0x55, 0xff };

// TODO:
//#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
//    static services_pipe_type_mapping_t
//        services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
//#else
//    #define NUMBER_OF_PIPES 0
//    static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
//#endif
//// Store the setup for the nRF8001 in the flash of the AVR to save on RAM
//static hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] = SETUP_MESSAGES_CONTENT;

// Main function prototypes ------------------------------------------------

/***************************************************************************
 * @prototype       void __ble_assert(const char *file, uint16_t line)
 * @description     Define how assert should function in the BLE library
 * @param
 * @return          None
 ***************************************************************************/
void __ble_assert(const char *file, uint16_t line)
{
#if _DEBUG_UART
    char msg[MAX_CHAR];
    sprintf(msg, "\rERROR %s: %d\r",file,line); USARTPuts(msg);
#endif
    while(1);
}

/***************************************************************************
 * @prototype       void main(void)
 * @description     Main task
 * @param           None
 * @return          None
 ***************************************************************************/
int main(void)
{
    // Main local variables ------------------------------------------------
#if _DEBUG_UART
    char msg[MAX_CHAR];
#endif

    // Initialization state ------------------------------------------------
    SystemInit();   // Device initialization
	
#if _DEBUG_UART
    USARTPuts("\r***ATmega328P setup***\r");
    Delay(_DEBUG_DELAY_MSG);
    USARTPuts("\rSet line ending to newline to send data from the serial monitor\r");
    Delay(_DEBUG_DELAY_MSG);
#endif

//    // Point ACI data structures to the the setup data that the nRFgo studio generated for the nRF8001
//    if (NULL != services_pipe_type_mapping)
//    {
//        aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
//    }
//    else
//    {
//        aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
//    }
//    aci_state.aci_setup_info.number_of_pipes    = NUMBER_OF_PIPES;
//    aci_state.aci_setup_info.setup_msgs         = setup_msgs;
//    aci_state.aci_setup_info.num_setup_msgs     = NB_SETUP_MESSAGES;

    // Initialize ACI
    lib_aci_init(&aci_state);
	hal_aci_tl_init();

#if _DEBUG_UART
    USARTPuts("\r***nRF8001 Reset done***\r");
    Delay(_DEBUG_DELAY_MSG);
#endif

    // Main loop -----------------------------------------------------------
    while (1)
    {
        // We enter the if statement only when there is a ACI event available to be processed
        if (lib_aci_event_get(&aci_state, &aci_data))
        {
            aci_evt_t* aci_evt;
            aci_evt = &aci_data.evt;

            switch(aci_evt->evt_opcode)
            {   // As soon as you reset the nRF8001 you will get an ACI Device Started Event
                case ACI_EVT_DEVICE_STARTED:
                {
                    aci_state.data_credit_available = aci_evt->params.device_started.credit_available;
                    switch(aci_evt->params.device_started.device_mode)
                    {
                        case ACI_DEVICE_SETUP:
#if _DEBUG_UART
                            USARTPuts("\rEvt Device Started: Setup\r");
                            Delay(_DEBUG_DELAY_MSG);
#endif
                            lib_aci_test(ACI_TEST_MODE_DTM_UART);
                            break;
                        case ACI_DEVICE_STANDBY:
#if _DEBUG_UART
                            USARTPuts("\rEvt Device Started: Standby\r");
                            Delay(_DEBUG_DELAY_MSG);
#endif
                            break;
                        case ACI_DEVICE_TEST:
                        {
#if _DEBUG_UART
                            USARTPuts("\rEvt Device Started: infinite Echo test\r");
                            Delay(_DEBUG_DELAY_MSG_LONG);
                            USARTPuts("\rRepeat the test with all bytes in echo_data inverted\r");
                            Delay(_DEBUG_DELAY_MSG_LONG);
                            USARTPuts("\rWaiting 4 seconds before the test starts....\r");
#endif
                            Delay(4000);
                            lib_aci_echo_msg(sizeof(echo_data), &echo_data[0]);
							break;
                        }
						default:
							break;
                    }
                    break;
                }
                case ACI_EVT_CMD_RSP:
                    // If an ACI command response event comes with an error -> stop
                    if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status)
                    {   // ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
                        // TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
                        // All other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command
#if _DEBUG_UART
                        sprintf(msg, "\rACI Command 0x%x\r",aci_evt->params.cmd_rsp.cmd_opcode);    USARTPuts(msg);
                        USARTPuts("Evt Cmd respone: Error. Stuck in a while(1); loop");
#endif
                        while (1);
                    }
                    break;
                case ACI_EVT_ECHO:
                    if (0 != memcmp(&echo_data[0], &(aci_evt->params.echo.echo_data[0]), sizeof(echo_data)))
                    {
#if _DEBUG_UART
                      USARTPuts("\rError: Echo loop test failed, verify SPI connectivity\r");
                      Delay(_DEBUG_DELAY_MSG);
#endif
                    }
                    else
                    {
#if _DEBUG_UART
                      USARTPuts("\r***Echo test succeeded***\r");
                      Delay(_DEBUG_DELAY_MSG);
#endif
                    }
                    lib_aci_echo_msg(sizeof(echo_data), &echo_data[0]);
                    break;
				default:
					break;
            }
        }
        else
        {
            USARTPuts("\rWaiting for an ACI event\r");
            Delay(_DEBUG_DELAY_MSG);
            // Should go to sleep and wake up from RDYN line
        }
    }
}