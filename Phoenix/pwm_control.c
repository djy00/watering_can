/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the license.txt file.
 */

/** @file
 * @brief    UART over BLE application using the app_uart library (event driven).
 *
 * This UART example is configured with flow control enabled which is necessary when softdevice
 * is enabled, in order to prevent data loss. To connect the development kit with your PC via 
 * UART, connect the configured RXD, TXD, RTS and CTS pins to the RXD, TXD, RTS and CTS pins 
 * on header P15 on the motherboard. Then connect the RS232 port on the nRFgo motherboard to
 * your PC. Configuration for UART pins is defined in the uart_conf.h header file.
 *
 * This file contains source code for a sample application that uses the Nordic UART service.
 * Connect to the UART example via Master Control Panel and the PCA10000 USB dongle, or via 
 * nRF UART 2.0 app for Android, or nRF UART app for IOS, available on 
 * https://www.nordicsemi.com/Products/nRFready-Demo-APPS.
 *
 * This example should be operated in the same way as the UART example for the evaluation board
 * in the SDK. Follow the same guide for this example, given on:
 * https://devzone.nordicsemi.com/documentation/nrf51/6.0.0/s110/html/a00066.html#project_uart_nus_eval_test
 *
 * This example uses FIFO RX and FIFO TX buffer to operate with the UART. You can set the size
 * for the FIFO buffers by modifying the RX_BUFFER_SIZE and TX_BUFFER_SIZE constants.
 *
 * Documentation for the app_uart library is given in UART driver documentation in the SDK at:
 * https://devzone.nordicsemi.com/documentation/nrf51/6.1.0/s110/html/a00008.html
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
//#include "simple_uart.h"
#include "boards.h"
#include "ble_error_log.h"
#include "ble_debug_assert_handler.h"
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "boards.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/


#define WAKEUP_BUTTON_PIN               BUTTON_0                                    /**< Button used to wake up the application. */

#define ADVERTISING_LED_PIN_NO          LED_0                                       /**< LED to indicate advertising state. */
#define CONNECTED_LED_PIN_NO            LED_1                                       /**< LED to indicate connected state. */

#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             4                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            2                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               16                                          /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               60                                          /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< slave latency. */
#define CONN_SUP_TIMEOUT                400                                         /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */
#define ON															"ON"
#define OFF															"OFF"
#define START_STRING                    "Start...\r\n"                                /**< The string that will be sent over the UART when the application starts. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
/**< Pin number for PWM output. */
#define TIMER_PRESCALER       (4)     /**< Prescaler setting for timers. */
#define LED_INTENSITY_HIGH    (255U)  /**< High intensity. */
#define LED_INTENSITY_LOW     (32U)   /**< Low intensity. */
#define LED_OFF               (0U)    /**< Led off. */
#define LED_INTENSITY_HALF    (128U)  /**< Half intensity. Used to calculate timer parameters. */
#define IN1 1
#define IN2 2
#define IN3 3	
#define IN4	4
#define IN5	5
#define IN6	6
#define IN7	7
#define IN8	8


//#define APP_GPIOTE_MAX_USERS            1

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */

//static bool ble_buffer_available = true;
//static bool tx_complete = false;

static void pwm_set(uint8_t new_setting)
{
    NRF_TIMER1->CC[3] = new_setting;

}
static void gpiote_init(void)
{
    NRF_GPIO->OUT    = 0x00000000UL;
    NRF_GPIO->DIRSET = 0x0000FF00UL;
    NRF_GPIO->DIRCLR = 0x000000FFUL;

    /* Configuring Button 0 as input */
    nrf_gpio_cfg_input(BUTTON_0, BUTTON_PULL);

    /* Configuring Button 1 as input. */
    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    nrf_gpio_cfg_input(BUTTON_1, BUTTON_PULL);

    /* Configuring Pin PWM_OUTPUT_PIN_NUMBER as output to be used for the PWM waveform. */
    nrf_gpio_cfg_output(IN1);
		nrf_gpio_cfg_output(IN2);
		nrf_gpio_cfg_output(IN3);
		nrf_gpio_cfg_output(IN4);
		nrf_gpio_cfg_output(IN5);
		nrf_gpio_cfg_output(IN6);
		nrf_gpio_cfg_output(IN7);
		nrf_gpio_cfg_output(IN8);
		nrf_gpio_cfg_output(29);
		nrf_gpio_cfg_output(1);
  
    /* Configure GPIOTE channel 0 to toggle the PWM pin state. */
    /* Note that we can only connect one GPIOTE task to one output pin. */
    
																	

																	

					//go
		
		
			nrf_gpiote_task_config(0,IN1, NRF_GPIOTE_POLARITY_TOGGLE, \
														

NRF_GPIOTE_INITIAL_VALUE_HIGH);
		
			nrf_gpiote_task_config(0,IN3 ,NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);
			
			
			nrf_gpiote_task_config(0,IN5, NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);
			nrf_gpiote_task_config(0,IN7 ,NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);
	
	
		
		
		
																	

																	

								//b
		/*nrf_gpiote_task_config(0,IN2, NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);
		
		nrf_gpiote_task_config(0,IN4, NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);
		
		nrf_gpiote_task_config(0,IN6, NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);
		
		nrf_gpiote_task_config(0,IN8, NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);
		*/
		
																	

																	

										//L
	/*
		nrf_gpiote_task_config(0,IN3, NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);
		nrf_gpiote_task_config(0,IN7, NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);
		nrf_gpiote_task_config(0,IN2, NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);
		nrf_gpiote_task_config(0,IN6, NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);*/
	
																	

																



		/*															

																	

									//R
		nrf_gpiote_task_config(0,IN4, NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);
		nrf_gpiote_task_config(0,IN8, NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);
		nrf_gpiote_task_config(0,IN1, NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);
		nrf_gpiote_task_config(0,IN5, NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);
		
		*/
}
void on_LED(){
	
	/*nrf_gpiote_task_config(0,IN1, NRF_GPIOTE_POLARITY_TOGGLE, \
													NRF_GPIOTE_INITIAL_VALUE_HIGH);
		
	nrf_gpiote_task_config(0,IN3 ,NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);
			
			
	nrf_gpiote_task_config(0,IN5, NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);
	nrf_gpiote_task_config(0,IN7 ,NRF_GPIOTE_POLARITY_TOGGLE, \
                           NRF_GPIOTE_INITIAL_VALUE_HIGH);*/
	nrf_gpio_pin_set(LED_3);
}

void off_LED(){
	nrf_gpio_pin_clear(LED_3);
}

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    //ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}


/**@brief       Assert macro callback function.
 *
 * @details     This function will be called in case of an assert in the SoftDevice.
 *
 * @warning     This handler is an example only and does not fit a final product. You need to
 *              analyze how your product is supposed to react in case of Assert.
 * @warning     On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief   Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    nrf_gpio_cfg_output(ADVERTISING_LED_PIN_NO);
    nrf_gpio_cfg_output(CONNECTED_LED_PIN_NO);
}

/**@brief   Function for Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
		   /* Start 16 MHz crystal oscillator */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;
  
    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)       
    {
        // Do nothing.
    }
  
    NRF_TIMER1->MODE      = TIMER_MODE_MODE_Timer;
    NRF_TIMER1->PRESCALER = 4;

    /* Load initial values to Timer 2 CC registers */
    /* Set initial CC0 value to anything > 1 */
    NRF_TIMER1->CC[0] = LED_INTENSITY_LOW;
    NRF_TIMER1->CC[1] = (LED_INTENSITY_HALF*2);

    /* Set up interrupt for CC2 */
    /* This interrupt is to force the change of CC0 to happen when it is safe */
    /* A safe time is after the highest possible CC0 value, but before the lowest one. */
    NRF_TIMER1->CC[2]    = LED_INTENSITY_HIGH;
    NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE2_Enabled << TIMER_INTENSET_COMPARE2_Pos;

    /* Create an Event-Task shortcut to clear TIMER2 on COMPARE[1] event. */
    NRF_TIMER1->SHORTS = (TIMER_SHORTS_COMPARE1_CLEAR_Enabled << TIMER_SHORTS_COMPARE1_CLEAR_Pos);
}


/**@brief   Function for the GAP initialization.
 *
 * @details This function will setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
																		
																				
}	
static void ppi_init(void)
{	/* Configure PPI channel 0 to toggle PWM_OUTPUT_PIN on every Timer 2 COMPARE[0] match. */
    NRF_PPI->CH[0].EEP  = (uint32_t)&NRF_TIMER1->EVENTS_COMPARE[0];
    NRF_PPI->CH[0].TEP  = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];

    /* Configure PPI channel 1 to toggle PWM_OUTPUT_PIN on every Timer 2 COMPARE[1] match. */
    NRF_PPI->CH[1].EEP  = (uint32_t)&NRF_TIMER1->EVENTS_COMPARE[1];
    NRF_PPI->CH[1].TEP  = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];
    /* Enable only PPI channels 0 and 1. */
    NRF_PPI->CHEN       = (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos) |
                          (PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos);
}

/**@brief   Function for the Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    
    ble_uuid_t adv_uuids[] = {{BLE_UUID_NUS_SERVICE, m_nus.uuid_type}};

    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;
    
    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}


/**@brief    Function for handling the data from the Nordic UART Service.
 *
 * @details  This function will process the data received from the Nordic UART BLE Service and send
 *           it to the UART module.
 */
/**@snippet [Handling the data received over BLE] */
void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{		
		if(strstr((char*)p_data, ON)) on_LED();
	  if(strstr((char*)p_data, OFF)) off_LED();
	
   /* for (int i = 0; i < length; i++)
    {
        simple_uart_put(p_data[i]);
    }
    simple_uart_putstring("\r\n"); */
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t         err_code;
    ble_nus_init_t   nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;  
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief       Function for handling an event from the Connection Parameters Module.
 *
 * @details     This function will be called for all events in the Connection Parameters Module
 *              which are passed to the application.
 *
 * @note        All this function does is to disconnect. This could have been done by simply setting
 *              the disconnect_on_fail config parameter, but instead we use the event handler
 *              mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief       Function for handling errors from the Connection Parameters module.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;
    
    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));
    
    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}


/**@brief       Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_evt_auth_status_t m_auth_status;
    ble_gap_enc_info_t *             p_enc_info;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
            nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            advertising_start();

            break;
            
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS, 
                                                   &m_sec_params);
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;
            
        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            p_enc_info = &m_auth_status.periph_keys.enc_info;
            if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div)
            {
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                // No keys found for this device
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            { 
                nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);

                // Configure buttons with sense level low as wakeup source.
                nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN,
                                         BUTTON_PULL,
                                         NRF_GPIO_PIN_SENSE_LOW);
                
                // Go to system-off mode (this function will not return; wakeup will cause a reset)
                err_code = sd_power_system_off();    
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief       Function for dispatching a S110 SoftDevice event to all modules with a S110
 *              SoftDevice event handler.
 *
 * @details     This function is called from the S110 SoftDevice event interrupt handler after a
 *              S110 SoftDevice event has been received.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief   Function for the S110 SoftDevice initialization.
 *
 * @details This function initializes the S110 SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief  Function for configuring the buttons.
 */
static void buttons_init(void)
{
    nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN,
                             BUTTON_PULL, 
                             NRF_GPIO_PIN_SENSE_LOW);    
}


/**@brief  Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}








int main(void)
{
      
    // Initialize
    leds_init();
    timers_init();
		ppi_init();
    buttons_init();
//    uart_init();
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    sec_params_init();
    nrf_gpio_cfg_output(LED_3);
		gpiote_init();
	//	NVIC_EnableIRQ(TIMER1_IRQn);

    advertising_start();
		
    for (;;)
    { 
				power_manage();
    }
}

/** 
 * @}
 */
