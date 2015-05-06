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
#include "aci_setup.h"
#include "hal_platform.h"
#include "inc/user.h"       // User config file
// Put the nRF8001 setup in the RAM of the nRF8001.
#include "services.h"

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
static uint8_t echo_data[] = { 0xDE, 0xAD, 0xBE, 0xEF };

#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
    static services_pipe_type_mapping_t services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
    #define NUMBER_OF_PIPES 0
    static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif

// Store the setup for the nRF8001 in the flash of the AVR to save on RAM
static const hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;

// Main function prototypes ------------------------------------------------
void CheckUSARTCmd(void);

/***************************************************************************
 * @prototype       void __ble_assert(const char *file, uint16_t line)
 * @description     Define how assert should function in the BLE library
 * @param
 * @return          None
 ***************************************************************************/
void __ble_assert(const char *file, uint16_t line)
{
    char msg[MAX_CHAR];

    sprintf(msg, "\r  ERROR %s: %d\r",file,line); USARTPuts(msg);
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
    char msg[MAX_CHAR];

    // Initialization state ------------------------------------------------
	SystemInit();
	Delay(100);	//////// compare with mine

    USARTPuts("\r***ATmega328P setup***\r");
    Delay(_DEBUG_DELAY_MSG);
    sprintf(msg, "\rCLOCK: %dKHZ",_DEFAULT_SYS_CLK/1000);   USARTPuts(msg);
    Delay(_DEBUG_DELAY_MSG);
    sprintf(msg, "\rUSART: %d\r",_BAUDRATE); USARTPuts(msg);
    Delay(_DEBUG_DELAY_MSG);

    // Point ACI data structures to the the setup data that the nRFgo studio generated for the nRF8001
    if (NULL != services_pipe_type_mapping)
    {
        aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
    }
    else
    {
        aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
    }
    aci_state.aci_setup_info.number_of_pipes = NUMBER_OF_PIPES;
    aci_state.aci_setup_info.setup_msgs = setup_msgs;
    aci_state.aci_setup_info.num_setup_msgs = NB_SETUP_MESSAGES;

    // Initialize the data structures required to set up the nRF8001 device.
    lib_aci_init(&aci_state);
    // Initialize HAL
    hal_aci_tl_init();
    
    // Main loop -----------------------------------------------------------
    while (1)
    {
        CheckUSARTCmd();

        static bool setup_required = false;
        
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
                            USARTPuts("\r  Evt Device Started: Setup\r");
                            Delay(_DEBUG_DELAY_MSG);
                            setup_required = true;
                            break;
                        case ACI_DEVICE_STANDBY:
                            USARTPuts("\r  Evt Device Started: Standby\r");
                            Delay(_DEBUG_DELAY_MSG);
                            // Looking for an iPhone by sending radio advertisements
                            // When an iPhone connects to us we will get an ACI_EVT_CONNECTED event from the nRF8001
                            if (aci_evt->params.device_started.hw_error)
                            {
                                Delay(20);    // Handle the HW error event correctly.
                            }
                            else
                            {
                                lib_aci_connect(5/* 5 seconds */, 0x0050 /* advertising interval 50ms*/);
                                USARTPuts("\rAdvertising started : Tap Connect on the nRF USART app\r");
                                Delay(_DEBUG_DELAY_MSG);
                            }
                            break;
                        case ACI_DEVICE_TEST:
                        {
                            USARTPuts("\r  Evt Device Started: Test\r");
                            Delay(_DEBUG_DELAY_MSG_LONG);
                            lib_aci_echo_msg(sizeof(echo_data), &echo_data[0]);
							break;
                        }
						default:
							break;
                    }
                    break;
                }
                case ACI_EVT_CMD_RSP:
                    switch (aci_evt->params.cmd_rsp.cmd_status)
                    {
                        case ACI_STATUS_SUCCESS:
                            switch (aci_evt->params.cmd_rsp.cmd_opcode)
                            {
                                case ACI_CMD_GET_DEVICE_ADDRESS:
                                    sprintf(msg,"\r  Evt Command Response: Addess [%x]\r",aci_evt->params.cmd_rsp.params.get_device_address);  USARTPuts(msg);
                                    Delay(_DEBUG_DELAY_MSG);
                                    break;
                                default:
                                    USARTPuts("\r  Evt Command Response: Unknown\r");
                                    Delay(_DEBUG_DELAY_MSG);
                                    break;
                            }
                            break;
                        case ACI_STATUS_TRANSACTION_CONTINUE:   // From ACI ReadDynamicData and ACI WriteDynamicData
                            break;
                        case ACI_STATUS_TRANSACTION_COMPLETE:   // From ACI ReadDynamicData and ACI WriteDynamicData
                            break;
                        case ACI_STATUS_ERROR_CMD_UNKNOWN:
                            USARTPuts("\r  Evt Cmd response: Error\r  (Unknown command)\r");
                            Delay(_DEBUG_DELAY_MSG_LONG);
                            break;
                        case ACI_STATUS_ERROR_DEVICE_STATE_INVALID:
                            USARTPuts("\r  Evt Cmd response: Error\r  (Command invalid in the current device state)\r");
                            Delay(_DEBUG_DELAY_MSG_LONG);
                            break;
                        default:
                            sprintf(msg, "\r  Evt Cmd response: Error\r  (ACI Command 0x%x)\r",aci_evt->params.cmd_rsp.cmd_opcode);    USARTPuts(msg);
                            Delay(_DEBUG_DELAY_MSG_LONG);
                            USARTUpdateState(S_RST);
                            break;
                    }
                    break;
                case ACI_EVT_ECHO:
                    if (0 != memcmp(&echo_data[0], &(aci_evt->params.echo.echo_data[0]), sizeof(echo_data)))
                    {
                        USARTPuts("\r  Error: Echo loop test failed, verify SPI connectivity\r");
                        Delay(_DEBUG_DELAY_MSG);
                        sprintf(msg,"\r  Received echo_data size: %d\r",sizeof(aci_evt->params.echo.echo_data[0]));    USARTPuts(msg);
                        Delay(_DEBUG_DELAY_MSG);
                    }
                    else
                    {
                      USARTPuts("\r  ***Echo test succeeded***\r");
                      Delay(_DEBUG_DELAY_MSG);
                    }
                    lib_aci_test(ACI_TEST_MODE_EXIT);
                    break;
				default:
					break;
            }
        }
        else
        {
//             USARTPuts("\r  Waiting for an ACI event\r");
// 			Delay(_DEBUG_DELAY_MSG);
            // Should go to sleep and wake up from RDYN line
        }
        
        // setup_required is set to true when the device starts up and enters setup mode.
        // It indicates that do_aci_setup() should be called. The flag should be cleared if
        // do_aci_setup() returns ACI_STATUS_TRANSACTION_COMPLETE
        if(setup_required)
        {
            uint8_t setup_status = do_aci_setup(&aci_state);
            switch (setup_status)
            {
                case SETUP_SUCCESS:
                    USARTPuts("\rSETUP SUCCESS\r");
                    setup_required = false;
                    break;
                case SETUP_FAIL_COMMAND_QUEUE_NOT_EMPTY:
                    USARTPuts("\rSETUP FAILED: CMD QUEUE NOT EMPTY\r");
                    break;
                case SETUP_FAIL_EVENT_QUEUE_NOT_EMPTY:
                    USARTPuts("\rSETUP FAILED: EVENT QUEUE NOT EMPTY\r");
                    break;
                case SETUP_FAIL_TIMEOUT:
                    USARTPuts("\rSETUP FAILED: TIMEOUT\r");
                    break;
                case SETUP_FAIL_NOT_SETUP_EVENT:
                    USARTPuts("\rSETUP FAILED: NO SETUP EVENT\r");
                    break;
                case SETUP_FAIL_NOT_COMMAND_RESPONSE:
                    USARTPuts("\rSETUP FAILED: NO CMD RESPONSE\r");
                    break;
                default:
                    break;
            }
            Delay(_DEBUG_DELAY_MSG);
        }
    }
}

/***************************************************************************
 * @prototype       void CheckUSARTCmd(void)
 * @description     Perform action upon a USART command
 * @param           None
 * @return          None
 ***************************************************************************/
void CheckUSARTCmd(void)
{
    uint8_t state;
    static uint8_t STimFlag = CLEAR,
                   SSPIClock = CLEAR;

    state = USARTGetState();
    switch (state)
    {
        case S_BLE_BOND:    // Start radio advertising
            USARTUpdateState(S_WAIT);
            USARTPuts("\r#STARTING RADIO ADVERTISING\r");
            Delay(_DEBUG_DELAY_MSG_LONG);
            if (lib_aci_bond(0,160))
            {
                USARTPuts("\r##RADIO ADVERTISING SUCCESSFUL\r");
                Delay(_DEBUG_DELAY_MSG_LONG);
            }
            else
            {
                USARTPuts("\r##START RADIO ADVERTISING FAILED\r");
                Delay(_DEBUG_DELAY_MSG_LONG);
            }
            break;
        case S_ADR: // Get device address
            USARTUpdateState(S_WAIT);
            USARTPuts("\r#GETTING NRF8001 ADDRESS\r");
            Delay(_DEBUG_DELAY_MSG_LONG);
            lib_aci_get_address();
            break;
        case S_ECH:
            USARTUpdateState(S_WAIT);
            USARTPuts("\r#TESTING TRANSPORT LAYER\r");
            Delay(_DEBUG_DELAY_MSG_LONG);
            lib_aci_test(ACI_TEST_MODE_DTM_ACI);
            break;
        case S_ACT: // Active State
            USARTUpdateState(S_WAIT);
            USARTPuts("\r#DEVICE ACTIVE\r");
            Delay(_DEBUG_DELAY_MSG_LONG);
            break;
        case S_TIM0:    // Start/Stop timer0 State
            USARTUpdateState(S_WAIT);
            STimFlag = !STimFlag;
            if (STimFlag)
            {
                USARTPuts("\r#TIMER 0 STARTED\r");
                Delay(_DEBUG_DELAY_MSG_LONG);
                TMR0_StartTimer();
            }
            else
            {
                USARTPuts("\r#TIMER 0 STOPPED\r");
                Delay(_DEBUG_DELAY_MSG_LONG);
                TMR0_StopTimer();
            }
            break;
        case S_SPI: // Switch SPI clock State
            USARTUpdateState(S_WAIT);
            USARTPuts("\r#CHANGING SPI CLOCK\r");
            Delay(_DEBUG_DELAY_MSG_LONG);
            SSPIClock = !SSPIClock;
            if (SSPIClock)
            {
                SPI_setClockDivider(SPI_CLOCK_DIV4);
                USARTPuts("\r##CHANGED TO FOSC/4");
                Delay(_DEBUG_DELAY_MSG_LONG);
            }
            else
            {
                SPI_setClockDivider(SPI_CLOCK_DIV16);
                USARTPuts("\r##CHANGED TO FOSC/16");
                Delay(_DEBUG_DELAY_MSG_LONG);
            }
            break;
        default:    // No state pending
            break;
    }
}