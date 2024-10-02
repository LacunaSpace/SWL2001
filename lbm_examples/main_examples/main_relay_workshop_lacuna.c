/*!
 * \file      main_periodical_uplink.c
 *
 * \brief     main program for periodical example
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "main.h"

#include "smtc_modem_api.h"
#include "smtc_modem_utilities.h"

#include "smtc_modem_hal.h"
#include "smtc_hal_dbg_trace.h"

#include "example_options.h"

#include "smtc_hal_mcu.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_watchdog.h"

#include "modem_pinout.h"
#include "smtc_modem_relay_api.h"
#include <string.h>

#include "smtc_hal_uart.h"
#include "smtc_hal_rtc.h"

#include <stdlib.h>
#include <time.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Returns the minimum value between a and b
 *
 * @param [in] a 1st value
 * @param [in] b 2nd value
 * @retval Minimum value
 */
#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

/*!
 * @brief Stringify constants
 */
#define xstr( a ) str( a )
#define str( a ) #a

/*!
 * @brief Concatenate strings
 */

#define CCC(x,y) x ## y
#define WCC(x,y) CCC(x,y)

/*!
 * @brief Helper macro that returned a human-friendly message if a command does not return SMTC_MODEM_RC_OK
 *
 * @remark The macro is implemented to be used with functions returning a @ref smtc_modem_return_code_t
 *
 * @param[in] rc  Return code
 */

#define ASSERT_SMTC_MODEM_RC( rc_func )                                                         \
    do                                                                                          \
    {                                                                                           \
        smtc_modem_return_code_t rc = rc_func;                                                  \
        if( rc == SMTC_MODEM_RC_NOT_INIT )                                                      \
        {                                                                                       \
            SMTC_HAL_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                  xstr( SMTC_MODEM_RC_NOT_INIT ) );                             \
        }                                                                                       \
        else if( rc == SMTC_MODEM_RC_INVALID )                                                  \
        {                                                                                       \
            SMTC_HAL_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                  xstr( SMTC_MODEM_RC_INVALID ) );                              \
        }                                                                                       \
        else if( rc == SMTC_MODEM_RC_BUSY )                                                     \
        {                                                                                       \
            SMTC_HAL_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                  xstr( SMTC_MODEM_RC_BUSY ) );                                 \
        }                                                                                       \
        else if( rc == SMTC_MODEM_RC_FAIL )                                                     \
        {                                                                                       \
            SMTC_HAL_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                  xstr( SMTC_MODEM_RC_FAIL ) );                                 \
        }                                                                                       \
        else if( rc == SMTC_MODEM_RC_NO_TIME )                                                  \
        {                                                                                       \
            SMTC_HAL_TRACE_WARNING( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__, \
                                    xstr( SMTC_MODEM_RC_NO_TIME ) );                            \
        }                                                                                       \
        else if( rc == SMTC_MODEM_RC_INVALID_STACK_ID )                                         \
        {                                                                                       \
            SMTC_HAL_TRACE_ERROR( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,   \
                                  xstr( SMTC_MODEM_RC_INVALID_STACK_ID ) );                     \
        }                                                                                       \
        else if( rc == SMTC_MODEM_RC_NO_EVENT )                                                 \
        {                                                                                       \
            SMTC_HAL_TRACE_INFO( "In %s - %s (line %d): %s\n", __FILE__, __func__, __LINE__,    \
                                 xstr( SMTC_MODEM_RC_NO_EVENT ) );                              \
        }                                                                                       \
    } while( 0 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * Device id value for ABP personalisation
 */
#ifndef WORKSHOP_DEVICE_ID
#define WORKSHOP_DEVICE_ID 00
#define PRE_PROVISIONED 0
#else
#define PRE_PROVISIONED 1
#endif

/**
 * Stack id value (multistacks modem is not yet available)
 */
#define STACK_ID 0


/**
 * @brief Stack credentials
 */
#if !defined( USE_LR11XX_CREDENTIALS )
static const uint8_t user_dev_eui[8]      = USER_LORAWAN_DEVICE_EUI;
static const uint8_t user_join_eui[8]     = USER_LORAWAN_JOIN_EUI;
static const uint8_t user_gen_app_key[16] = USER_LORAWAN_GEN_APP_KEY;
static const uint8_t user_app_key[16]     = USER_LORAWAN_APP_KEY;
#endif 

#if (PRE_PROVISIONED)
static uint32_t abp_dev_addr = WCC(0x27AAAA,WORKSHOP_DEVICE_ID); // simple and predictable personalisation
static uint8_t abp_nwk_key[16] = {0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, WCC(0x,WORKSHOP_DEVICE_ID)};
static uint8_t abp_app_key[16] = {0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, WCC(0x,WORKSHOP_DEVICE_ID)};
static uint8_t abp_wor_key[16] = {0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, WCC(0x,WORKSHOP_DEVICE_ID)};
#else
static uint32_t abp_dev_addr = WCC(0x000000,WORKSHOP_DEVICE_ID); // simple and predictable personalisation
static uint8_t abp_nwk_key[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, WCC(0x,WORKSHOP_DEVICE_ID)};
static uint8_t abp_app_key[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, WCC(0x,WORKSHOP_DEVICE_ID)};
static uint8_t abp_wor_key[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, WCC(0x,WORKSHOP_DEVICE_ID)};
#endif
/**
 * @brief Watchdog counter reload value during sleep (The period must be lower than MCU watchdog period (here 32s))
 */
#define WATCHDOG_RELOAD_PERIOD_MS 20000

/**
 * @brief Periodical uplink alarm delay in seconds
 */
#ifndef PERIODICAL_TIME_UPDATE_DELAY_S
#define PERIODICAL_TIME_UPDATE_DELAY_S 60
#endif

#ifndef DELAY_FIRST_MSG_AFTER_JOIN
#define DELAY_FIRST_MSG_AFTER_JOIN 60
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static uint8_t                  rx_payload[SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH] = { 0 };  // Buffer for rx payload
static uint8_t                  rx_payload_size = 0;      // Size of the payload in the rx_payload buffer
static smtc_modem_dl_metadata_t rx_metadata     = { 0 };  // Metadata of downlink
static uint8_t                  rx_remaining    = 0;      // Remaining downlink payload in modem

static volatile bool user_button_is_press = false;  // Flag for button status

#if defined( USE_RELAY_TX )
static smtc_modem_relay_tx_config_t relay_config = { 0 };
#endif
/**
 * @brief Internal credentials
 */
#if defined( USE_LR11XX_CREDENTIALS )
static uint8_t chip_eui[SMTC_MODEM_EUI_LENGTH] = { 0 };
static uint8_t chip_pin[SMTC_MODEM_PIN_LENGTH] = { 0 };
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static uint8_t cli_buffer[100], cli_idx=0;

static bool modem_busy = false;

static uint32_t rtc_offset=0;

/**
 * @brief User callback for modem event
 *
 *  This callback is called every time an event ( see smtc_modem_event_t ) appears in the modem.
 *  Several events may have to be read from the modem when this callback is called.
 */
static void modem_event_callback( void );

/**
 * @brief User callback for button EXTI
 *
 * @param context Define by the user at the init
 */
static void user_button_callback( void* context );

static bool modem_join( void );

static void send_message( uint8_t port, uint8_t* data );

static bool cli_parse_hex_value( uint8_t* buffer, uint8_t* hex, uint8_t n );

static uint32_t cli_parse_date( uint8_t* date );

static void cli_print_time( void );

static void cli_handle_uart_errors( void );

static void cli_process_input( uint8_t c );

// too much hassle to include wake_on_radio.h
void wor_debug_set_root_skey ( uint32_t dev_addr, uint8_t *wor_skey );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Example to send a user payload on an external event
 *
 */
void main_periodical_uplink( void )
{
    // Disable IRQ to avoid unwanted behavior during init
    hal_mcu_disable_irq( );

    // Configure all the µC periph (clock, gpio, timer, ...)
    hal_mcu_init( );

    // Init the modem and use modem_event_callback as event callback, please note that the callback will be
    // called immediately after the first call to smtc_modem_run_engine because of the reset detection
    smtc_modem_init( &modem_event_callback );

    // Configure Nucleo blue button as EXTI
    hal_gpio_irq_t nucleo_blue_button = {
        .pin      = EXTI_BUTTON,
        .context  = NULL,                  // context pass to the callback - not used in this example
        .callback = user_button_callback,  // callback called when EXTI is triggered
    };
    hal_gpio_init_in( EXTI_BUTTON, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_FALLING, &nucleo_blue_button );

    // Init done: enable interruption
    hal_mcu_enable_irq( );

    SMTC_HAL_TRACE_INFO( "Lacuna TTC2024 workshop demo starting with %s device firmware\n", ((PRE_PROVISIONED)?"PRE-PROVISIONED":"UNPROVISIONED") );

    uint8_t c;
	uint32_t cli_timeout=0;

    while( 1 )
    {
 		// Check & clear possible CLI UART errors
        cli_handle_uart_errors(); // buffer overflows will block the UART
        
        // Check CLI    
        if ( trace_uart_getc( &c ) == true )
        {
            if (!modem_busy) // ignore cli input while modem is sending a message
            {
                cli_process_input( c );
                
                cli_timeout=hal_rtc_get_time_ms()+2000;
            }
        }
      
        // Check button
        if( user_button_is_press == true )
        {
            user_button_is_press = false;

            if (modem_busy)
            {
                SMTC_HAL_TRACE_WARNING( "Modem is busy ignoring button press\n" );    
            }
            else
            {
                send_message( 100, NULL );
            }    
        }

        // Modem process launch
        if (hal_rtc_get_time_ms()>cli_timeout) // poor mans solution to prevent UART buffer overruns 
        {    
            smtc_modem_run_engine();
        }

        // sleep remodved to keep UART enabled
        hal_watchdog_reload( );

    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void modem_event_callback( void )
{
    SMTC_HAL_TRACE_MSG_COLOR( "Modem event callback\n", HAL_DBG_TRACE_COLOR_BLUE );

    smtc_modem_event_t current_event;
    uint8_t            event_pending_count;
    uint8_t            stack_id = STACK_ID;

    // Continue to read modem event until all event has been processed
    do
    {
        // Read modem event
        ASSERT_SMTC_MODEM_RC( smtc_modem_get_event( &current_event, &event_pending_count ) );

        switch( current_event.event_type )
        {
        case SMTC_MODEM_EVENT_RESET:
            SMTC_HAL_TRACE_INFO( "Event received: RESET\n" );

#if !defined( USE_LR11XX_CREDENTIALS )
            // Set user credentials
            ASSERT_SMTC_MODEM_RC( smtc_modem_set_deveui( stack_id, user_dev_eui ) );
            ASSERT_SMTC_MODEM_RC( smtc_modem_set_joineui( stack_id, user_join_eui ) );
            ASSERT_SMTC_MODEM_RC( smtc_modem_set_appkey( stack_id, user_gen_app_key ) );
            ASSERT_SMTC_MODEM_RC( smtc_modem_set_nwkkey( stack_id, user_app_key ) );
#else
            // Get internal credentials
            ASSERT_SMTC_MODEM_RC( smtc_modem_get_chip_eui( stack_id, chip_eui ) );
            SMTC_HAL_TRACE_ARRAY( "CHIP_EUI", chip_eui, SMTC_MODEM_EUI_LENGTH );
            ASSERT_SMTC_MODEM_RC( smtc_modem_get_pin( stack_id, chip_pin ) );
            SMTC_HAL_TRACE_ARRAY( "CHIP_PIN", chip_pin, SMTC_MODEM_PIN_LENGTH );
#endif
            // Set user region
            ASSERT_SMTC_MODEM_RC( smtc_modem_set_region( stack_id, MODEM_EXAMPLE_REGION ) );
// Schedule a Join LoRaWAN network
#if defined( USE_RELAY_TX )
            // by default when relay mode is activated , CSMA is also activated by default to at least protect the WOR
            // transmission
            // if you want to disable the csma please uncomment the next line
            // ASSERT_SMTC_MODEM_RC(smtc_modem_csma_set_state (stack_id,false));

            relay_config.second_ch_enable = false;

            // The RelayModeActivation field indicates how the end-device SHOULD manage the relay mode.
            relay_config.activation = SMTC_MODEM_RELAY_TX_ACTIVATION_MODE_DYNAMIC;

            // number_of_miss_wor_ack_to_switch_in_nosync_mode  field indicates that the
            // relay mode SHALL be restart in no sync mode when it does not receive a WOR ACK frame after
            // number_of_miss_wor_ack_to_switch_in_nosync_mode consecutive uplinks.
            relay_config.number_of_miss_wor_ack_to_switch_in_nosync_mode = 3;

            // smart_level field indicates that the
            // relay mode SHALL be enabled if the end-device does not receive a valid downlink after smart_level
            // consecutive uplinks.
            relay_config.smart_level = 8;

            // The BackOff field indicates how the end-device SHALL behave when it does not receive
            // a WOR ACK frame.
            // BackOff Description
            // 0 Always send a LoRaWAN uplink
            // 1..63 Send a LoRaWAN uplink after X WOR frames without a WOR ACK
            relay_config.backoff = 4;
            ASSERT_SMTC_MODEM_RC( smtc_modem_relay_tx_enable( stack_id, &relay_config ) );
#endif
            // ASSERT_SMTC_MODEM_RC( smtc_modem_join_network( stack_id ) );
            ASSERT_SMTC_MODEM_RC( smtc_modem_alarm_start_timer( PERIODICAL_TIME_UPDATE_DELAY_S ) );
            break;

        case SMTC_MODEM_EVENT_ALARM:
            SMTC_HAL_TRACE_INFO( "Event received: ALARM\n" ); 
            cli_print_time();
            // Send periodical uplink on port 101
            // send_uplink_counter_on_port( 101 ); // no periodical uplink for the workshop demo 
            // Restart periodical uplink alarm
            ASSERT_SMTC_MODEM_RC( smtc_modem_alarm_start_timer( PERIODICAL_TIME_UPDATE_DELAY_S ) );
            break;

        case SMTC_MODEM_EVENT_JOINED:
            SMTC_HAL_TRACE_INFO( "Event received: JOINED\n" );
            SMTC_HAL_TRACE_INFO( "Modem is now joined \n" );
            // Send first periodical uplink on port 101
            // send_uplink_counter_on_port( 101 ); // no uplink on join 
            // start periodical uplink alarm
            // ASSERT_SMTC_MODEM_RC( smtc_modem_alarm_start_timer( DELAY_FIRST_MSG_AFTER_JOIN ) );
            break;

        case SMTC_MODEM_EVENT_TXDONE:
            SMTC_HAL_TRACE_INFO( "Event received: TXDONE\n" );
            SMTC_HAL_TRACE_INFO( "Transmission done \n" );
            modem_busy = false;
            break;

        case SMTC_MODEM_EVENT_DOWNDATA:
            SMTC_HAL_TRACE_INFO( "Event received: DOWNDATA\n" );
            // Get downlink data
            ASSERT_SMTC_MODEM_RC(
                smtc_modem_get_downlink_data( rx_payload, &rx_payload_size, &rx_metadata, &rx_remaining ) );
            SMTC_HAL_TRACE_PRINTF( "Data received on port %u\n", rx_metadata.fport );
            SMTC_HAL_TRACE_ARRAY( "Received payload", rx_payload, rx_payload_size );
            break;

        case SMTC_MODEM_EVENT_JOINFAIL:
            SMTC_HAL_TRACE_INFO( "Event received: JOINFAIL\n" );
            break;

        case SMTC_MODEM_EVENT_ALCSYNC_TIME:
            SMTC_HAL_TRACE_INFO( "Event received: ALCSync service TIME\n" );
            break;

        case SMTC_MODEM_EVENT_LINK_CHECK:
            SMTC_HAL_TRACE_INFO( "Event received: LINK_CHECK\n" );
            break;

        case SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO:
            SMTC_HAL_TRACE_INFO( "Event received: CLASS_B_PING_SLOT_INFO\n" );
            break;

        case SMTC_MODEM_EVENT_CLASS_B_STATUS:
            SMTC_HAL_TRACE_INFO( "Event received: CLASS_B_STATUS\n" );
            break;

        case SMTC_MODEM_EVENT_LORAWAN_MAC_TIME:
            SMTC_HAL_TRACE_WARNING( "Event received: LORAWAN MAC TIME\n" );
            break;

        case SMTC_MODEM_EVENT_LORAWAN_FUOTA_DONE:
        {
            bool status = current_event.event_data.fuota_status.successful;
            if( status == true )
            {
                SMTC_HAL_TRACE_INFO( "Event received: FUOTA SUCCESSFUL\n" );
            }
            else
            {
                SMTC_HAL_TRACE_WARNING( "Event received: FUOTA FAIL\n" );
            }
            break;
        }

        case SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_C:
            SMTC_HAL_TRACE_INFO( "Event received: MULTICAST CLASS_C STOP\n" );
            break;

        case SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_B:
            SMTC_HAL_TRACE_INFO( "Event received: MULTICAST CLASS_B STOP\n" );
            break;

        case SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_C:
            SMTC_HAL_TRACE_INFO( "Event received: New MULTICAST CLASS_C \n" );
            break;

        case SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_B:
            SMTC_HAL_TRACE_INFO( "Event received: New MULTICAST CLASS_B\n" );
            break;

        case SMTC_MODEM_EVENT_FIRMWARE_MANAGEMENT:
            SMTC_HAL_TRACE_INFO( "Event received: FIRMWARE_MANAGEMENT\n" );
            if( current_event.event_data.fmp.status == SMTC_MODEM_EVENT_FMP_REBOOT_IMMEDIATELY )
            {
                smtc_modem_hal_reset_mcu( );
            }
            break;

        case SMTC_MODEM_EVENT_STREAM_DONE:
            SMTC_HAL_TRACE_INFO( "Event received: STREAM_DONE\n" );
            break;

        case SMTC_MODEM_EVENT_UPLOAD_DONE:
            SMTC_HAL_TRACE_INFO( "Event received: UPLOAD_DONE\n" );
            break;

        case SMTC_MODEM_EVENT_DM_SET_CONF:
            SMTC_HAL_TRACE_INFO( "Event received: DM_SET_CONF\n" );
            break;

        case SMTC_MODEM_EVENT_MUTE:
            SMTC_HAL_TRACE_INFO( "Event received: MUTE\n" );
            break;
        case SMTC_MODEM_EVENT_RELAY_TX_DYNAMIC:  //!< Relay TX dynamic mode has enable or disable the WOR protocol
            SMTC_HAL_TRACE_INFO( "Event received: RELAY_TX_DYNAMIC\n" );
            break;
        case SMTC_MODEM_EVENT_RELAY_TX_MODE:  //!< Relay TX activation has been updated
            SMTC_HAL_TRACE_INFO( "Event received: RELAY_TX_MODE\n" );
            break;
        case SMTC_MODEM_EVENT_RELAY_TX_SYNC:  //!< Relay TX synchronisation has changed
            SMTC_HAL_TRACE_INFO( "Event received: RELAY_TX_SYNC\n" );
            break;
        default:
            SMTC_HAL_TRACE_ERROR( "Unknown event %u\n", current_event.event_type );
            break;
        }
    } while( event_pending_count > 0 );
}

static void user_button_callback( void* context )
{
    SMTC_HAL_TRACE_INFO( "Button pushed\n" );

    ( void ) context;  // Not used in the example - avoid warning

    static uint32_t last_press_timestamp_ms = 0;

    // Debounce the button press, avoid multiple triggers
    if( ( int32_t ) ( smtc_modem_hal_get_time_in_ms( ) - last_press_timestamp_ms ) > 2000 )
    {
        last_press_timestamp_ms = smtc_modem_hal_get_time_in_ms( );
        user_button_is_press    = true;
    }
}

static bool is_joined( void )
{
    smtc_modem_status_mask_t status_mask = 0;
    smtc_modem_get_status( STACK_ID, &status_mask );
    return ( ( status_mask & SMTC_MODEM_STATUS_JOINED ) == SMTC_MODEM_STATUS_JOINED );
}

static bool modem_join( void )
{
    if (!is_joined())
    {
        ASSERT_SMTC_MODEM_RC( smtc_modem_debug_connect_with_abp( STACK_ID, abp_dev_addr, abp_nwk_key, abp_app_key) ); 
		wor_debug_set_root_skey( abp_dev_addr, abp_wor_key );
        return true;
    }
    return false;
}

static void send_message( uint8_t port, uint8_t* data )
{
    static uint8_t uplink_counter = 0;

    if (!is_joined())
    {
        modem_join();
    }
    
    if (modem_busy)
    {
        SMTC_HAL_TRACE_WARNING( "Modem is busy ... dropping message\n" );    
    }
    else
    {
        uint32_t timestamp=hal_rtc_get_time_s()+rtc_offset;
        uint8_t msg[40];
        
        msg[0]=port;
        msg[1]=uplink_counter;
        msg[2]=(uint8_t)(timestamp>>24);
        msg[3]=(uint8_t)(timestamp>>16);
        msg[4]=(uint8_t)(timestamp>>8);
        msg[5]=(uint8_t)(timestamp);
        msg[6]=PRE_PROVISIONED?1:2;
        
        uint8_t len=strlen((char*)data);
        if (len>30) len=30;
        memcpy(msg+7,data, len);
        ASSERT_SMTC_MODEM_RC( smtc_modem_request_uplink( STACK_ID, port, false, msg, len + 7 ) );

        SMTC_HAL_TRACE_WARNING( "Modem is busy now ... ignoring any cli input until message has been sent\n" );    
        
        uplink_counter++;
        modem_busy=true;
    }
}

static bool cli_parse_hex_value( uint8_t* buffer, uint8_t* hex, uint8_t n ) 
{
    uint8_t i,j,idx,value;

    if (strlen((const char*)buffer)!=(n*2)) return false; 
    
    for (i=0;i<n;i++)
    {
        value=0;
       
        for (j=0;j<2;j++)
        {
            value<<=4;
            idx=(i*2)+j;
            
            if ((buffer[idx]>=48)&&(buffer[idx]<=57)) 
            { 
                value|=buffer[idx]-48; // 0-9
            }
            else 
            {
                if ((buffer[idx]>=65)&&(buffer[idx]<=70)) 
                {
                    value|=buffer[idx]-55; // A-F
                }
                else 
                {
                    if ((buffer[idx]>=97)&&(buffer[idx]<=102)) 
                    {
                        value|=buffer[idx]-87; // a-f
                    }
                    else 
                    {
                        return false;
                    }
                }
            }

        }
        hex[i]=value;
    }
    return true;
}

static uint32_t cli_parse_date(uint8_t* date)
{
    struct tm t;
    time_t t_of_day;

    if (strlen((char*)date)!=20) return 0;
    if ((date[4]!='-')||(date[7]!='-')||(date[10]!='T')||(date[13]!=':')||(date[16]!=':')||(date[19]!='Z')) return 0;

    date[4]=0;date[7]=0;date[10]=0;date[13]=0;date[16]=0;date[19]=0;  

    t.tm_year = atoi((char*)date)-1900;  // Year - 1900
    t.tm_mon = atoi((char*)date+5);           // Month, where 0 = jan
    t.tm_mday = atoi((char*)date+8);          // Day of the month
    t.tm_hour = atoi((char*)date+11);
    t.tm_min = atoi((char*)date+14);
    t.tm_sec = atoi((char*)date+17);
    t.tm_isdst = 0;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
    t_of_day = mktime(&t);
    return (uint32_t) t_of_day;
}

static void cli_print_time( void )
{
    struct tm ts_s;
    time_t ts;
    char time_str[25];
    uint32_t uptime=hal_rtc_get_time_s();
            
    if (rtc_offset)
    {
        ts=(time_t)(uptime+rtc_offset);
        ts_s=*gmtime(&ts);
        strftime(time_str, sizeof(time_str), "%Y-%m-%dT%H:%M:%SZ", &ts_s);
        SMTC_HAL_TRACE_INFO( "Current timestamp: %s\n", time_str );
    }

    ts=(time_t)uptime;
    ts_s=*gmtime(&ts), 
    strftime(time_str, sizeof(time_str), "%H:%M:%S", &ts_s);
    SMTC_HAL_TRACE_INFO( "Device uptime: %s\n", time_str );
}

static void cli_handle_uart_errors( void )
{
	static uint32_t flags_old, flags_new; 
    flags_new=trace_uart_get_error_flags();

    if (flags_new)
	{
        if (flags_new!=flags_old)
        {
            if (flags_new&((uint32_t)(0x01)))
    			SMTC_HAL_TRACE_WARNING( "UART parity error\n" );
    	    if (flags_new&((uint32_t)(0x02)))
	    		SMTC_HAL_TRACE_WARNING( "UART frame error\n" );
	        if (flags_new&((uint32_t)(0x04)))
			    SMTC_HAL_TRACE_WARNING( "UART noise error\n" );
    	    if (flags_new&((uint32_t)(0x08)))
                SMTC_HAL_TRACE_WARNING( "UART overrun error\n" );
            flags_old=flags_new;
        }
        trace_uart_clear_error_flags();
        cli_idx=0;
    }
}       	

static void cli_process_input( uint8_t c )
{
    if ((c=='\n')||(c=='\r'))
    {
        // process line
        uint8_t hex_buffer[16];
        cli_buffer[cli_idx]=0;
        cli_idx=0;
        if (!strncmp("set_address",(const char*)cli_buffer,11))
        {
            if (is_joined())
            {
                SMTC_HAL_TRACE_ERROR( "Can't change address after the device has joined the network\n" );       
            }        
            else
            {
                if (cli_parse_hex_value(cli_buffer+12,hex_buffer,4))
                {
                    abp_dev_addr=(((uint32_t)hex_buffer[0])<<24)|(((uint32_t)hex_buffer[1])<<16)|(((uint32_t)hex_buffer[2])<<8)|((uint32_t)hex_buffer[3]);
                    SMTC_HAL_TRACE_INFO( "Address OK: %08x\n",abp_dev_addr);
                }
                else
                {
                    SMTC_HAL_TRACE_ERROR( "Invalid address\n" );
                }
            }       
        }
        else if (!strncmp("set_nwk_key",(const char*)cli_buffer,11))
        {
            if (is_joined())
            {
                SMTC_HAL_TRACE_ERROR( "Can't change network key after the device has joined the network\n" );       
            }        
            else
            {
                if (cli_parse_hex_value(cli_buffer+12,hex_buffer,16))
                {
                    SMTC_HAL_TRACE_INFO( "Network key OK: %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",hex_buffer[0],hex_buffer[1],hex_buffer[2],hex_buffer[3],hex_buffer[4],hex_buffer[5],hex_buffer[6],hex_buffer[7],hex_buffer[8],hex_buffer[9],hex_buffer[10],hex_buffer[11],hex_buffer[12],hex_buffer[13],hex_buffer[14],hex_buffer[15]);
                    memcpy(abp_nwk_key,hex_buffer,16);
                }    
                else
                {
                    SMTC_HAL_TRACE_ERROR( "Invalid key format\n" );
                }
            }
        }
        else if (!strncmp("set_app_key",(const char*)cli_buffer,11))
        {
            if (is_joined())
            {
                SMTC_HAL_TRACE_ERROR( "Can't change application key after the device has joined the network\n" );       
            }        
            else
            {
                if (cli_parse_hex_value(cli_buffer+12,hex_buffer,16))
                {
                    SMTC_HAL_TRACE_INFO( "Application key OK: %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",hex_buffer[0],hex_buffer[1],hex_buffer[2],hex_buffer[3],hex_buffer[4],hex_buffer[5],hex_buffer[6],hex_buffer[7],hex_buffer[8],hex_buffer[9],hex_buffer[10],hex_buffer[11],hex_buffer[12],hex_buffer[13],hex_buffer[14],hex_buffer[15]);
                    memcpy(abp_app_key,hex_buffer,16);
                }    
                else
                {
                    SMTC_HAL_TRACE_ERROR( "Invalid key format\n" );
                }
            }
        }
        else if (!strncmp("set_wor_key",(const char*)cli_buffer,11))
        {
            if (is_joined())
            {
                SMTC_HAL_TRACE_ERROR( "Can't change wor key after the device has joined the network\n" );       
            }        
            else
            {
                if (cli_parse_hex_value(cli_buffer+12,hex_buffer,16))
                {
                    SMTC_HAL_TRACE_INFO( "Wor key OK: %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",hex_buffer[0],hex_buffer[1],hex_buffer[2],hex_buffer[3],hex_buffer[4],hex_buffer[5],hex_buffer[6],hex_buffer[7],hex_buffer[8],hex_buffer[9],hex_buffer[10],hex_buffer[11],hex_buffer[12],hex_buffer[13],hex_buffer[14],hex_buffer[15]);
                    memcpy(abp_wor_key,hex_buffer,16);
                }    
                else
                {
                    SMTC_HAL_TRACE_ERROR( "Invalid key format\n" );
                }
            }
        }
        else if (!strncmp("join",(const char*)cli_buffer,4))
        {
            if (is_joined())
            {
                SMTC_HAL_TRACE_ERROR( "The device has already joined the network\n" );       
            }        
            else
            {
                SMTC_HAL_TRACE_INFO( "Joining the network\n" );       
                modem_join();
            }
        }
        else if (!strncmp("get_status",(const char*)cli_buffer,10))
        {
            SMTC_HAL_TRACE_INFO( "Address: %08x\n",abp_dev_addr);
            SMTC_HAL_TRACE_INFO( "Nwk key: %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",abp_nwk_key[0],abp_nwk_key[1],abp_nwk_key[2],abp_nwk_key[3],abp_nwk_key[4],abp_nwk_key[5],abp_nwk_key[6],abp_nwk_key[7],abp_nwk_key[8],abp_nwk_key[9],abp_nwk_key[10],abp_nwk_key[11],abp_nwk_key[12],abp_nwk_key[13],abp_nwk_key[14],abp_nwk_key[15]);
            SMTC_HAL_TRACE_INFO( "App key: %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",abp_app_key[0],abp_app_key[1],abp_app_key[2],abp_app_key[3],abp_app_key[4],abp_app_key[5],abp_app_key[6],abp_app_key[7],abp_app_key[8],abp_app_key[9],abp_app_key[10],abp_app_key[11],abp_app_key[12],abp_app_key[13],abp_app_key[14],abp_app_key[15]);
            SMTC_HAL_TRACE_INFO( "Wor key: %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",abp_wor_key[0],abp_wor_key[1],abp_wor_key[2],abp_wor_key[3],abp_wor_key[4],abp_wor_key[5],abp_wor_key[6],abp_wor_key[7],abp_wor_key[8],abp_wor_key[9],abp_wor_key[10],abp_wor_key[11],abp_wor_key[12],abp_wor_key[13],abp_wor_key[14],abp_wor_key[15]);
            if (is_joined())
            {
                SMTC_HAL_TRACE_INFO( "Join status: JOINED\n" );
            }    
            else    
            {
                SMTC_HAL_TRACE_INFO( "Join status: NOT JOINED\n" );
            }
        }
        else if (!strncmp("get_time",(const char*)cli_buffer,8))
        {
            cli_print_time();
        }
        else if (!strncmp("set_time",(const char*)cli_buffer,8))
        {
            uint32_t epoch = cli_parse_date(cli_buffer+9),
                    uptime=hal_rtc_get_time_s();
            if ((!epoch)||(epoch<=uptime))
            {
                SMTC_HAL_TRACE_ERROR("Invalid timestamp. Timestamp should be in the form: YYYY-MM-DDTHH:MM:SSZ\n");    
            }
            else
            {
                SMTC_HAL_TRACE_INFO("RTC time set\n");    
                rtc_offset=epoch-uptime;
                cli_print_time();
            }
        }
        else if (!strncmp("send_message",(const char*)cli_buffer,12))
        {
            if (strlen((const char*)(cli_buffer))==12) // no argument given, terminate string correctly
                cli_buffer[13]=0;

            size_t message_size = strlen((const char*)(cli_buffer+13));
            if (message_size<=30)
            {
                send_message( 101, cli_buffer+13 );
            }
            else
            {
                SMTC_HAL_TRACE_ERROR( "Message too long, maximum message size is 30\n" );
            }    
        }
        else if (!strncmp("help",(const char*)cli_buffer,4))
        {
            SMTC_HAL_TRACE_INFO( "Commands available:\n");
            SMTC_HAL_TRACE_INFO( "get_status\n");
            SMTC_HAL_TRACE_INFO( "set_address\n");
            SMTC_HAL_TRACE_INFO( "set_nwk_key\n");
            SMTC_HAL_TRACE_INFO( "set_app_key\n");
            SMTC_HAL_TRACE_INFO( "set_wor_key\n");
            SMTC_HAL_TRACE_INFO( "join\n");
            SMTC_HAL_TRACE_INFO( "get_time\n");
            SMTC_HAL_TRACE_INFO( "set_time\n");
            SMTC_HAL_TRACE_INFO( "send_message\n");
        }
        else
        {
            if (strlen((const char*)cli_buffer)>=1)
            {
                SMTC_HAL_TRACE_ERROR( "Unknown command: %s\n", cli_buffer );
            }    
        }
    }
    else
    {
        cli_buffer[ cli_idx++ ] = c;
        if ( cli_idx >= 100 ) // CLI buffer overflow
            cli_idx=0;
    }   
}
 


/* --- EOF ------------------------------------------------------------------ */
