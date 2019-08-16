#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "app_util.h"
#include "app_uart.h"
#include "app_pwm.h"
#include "app_scheduler.h"

#include "boards.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_peripherals.h"
#include "nrf_drv_clock.h"
#ifdef ADC_PRESENT
#include "nrf_drv_adc.h"
#else
#include "nrf_drv_saadc.h"
#endif //ADC_PRESENT
#include "nrf_delay.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"

#include "ble.h"
#include "ble_hci.h"
#include "ble_nus.h"
#include "ble_conn_state.h"
#include "ble_db_discovery.h"
#include "ble_conn_params.h"
#include "ble_nus.h"
#include "ble_tps.h"
#include "ble_ias.h"
#include "ble_lls.h"
#include "ble_bas.h"
#include "ble_ias_c.h"
#include "ble_advertising.h"

#include "softdevice_handler.h"
#include "peer_manager.h"
#include "fstorage.h"
#include "fds.h"

#include "sensorsim.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"


#define APP_FEATURE_NOT_SUPPORTED         BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2         /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT				0												/**<number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT			1												/**<number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME								"Nordic_test"						/**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL								40								/**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS			180								/**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER							0									/**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE					4									/**< Size of timer operation queues. */

#define ADVERTISEMENT_LED_TIMER_INTERVAL								APP_TIMER_TICKS(100, APP_TIMER_PRESCALER) /**< LED blinking  interval (ticks). This value corresponds to 0.1 seconds. */
#define MOTOR_STOP_TIMER_INTERVAL			APP_TIMER_TICKS(100, APP_TIMER_PRESCALER) /**< Motor stop timing(ticks). This value corresponds to 0.1 seconds. */
#define BATTERY_LEVEL_MEAS_INTERVAL			APP_TIMER_TICKS(120000, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). This value corresponds to 120 seconds. */
#define BUZZER_STOP_TIMER_INTERVAL			APP_TIMER_TICKS(100, APP_TIMER_PRESCALER) /**< Buzzer stop  timing (ticks). This value corresponds to 0.1 seconds. */

/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 *
 * @retval     Result converted to millivolts.
 */
#ifdef ADC_PRESENT
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VBG_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_INPUT_PRESCALER)
#else // SAADC_PRESENT
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)
#endif // ADC_PRESENT

#define ADC_REF_VBG_VOLTAGE_IN_MILLIVOLTS 1200                                         /**< Value in millivolts for voltage used as reference in ADC conversion on NRF51. */
#define ADC_INPUT_PRESCALER               3                                            /**< Input prescaler for ADC convestion on NRF51. */
#define ADC_RES_10BIT                     1024                                         /**< Maximum digital value for 10-bit ADC conversion. */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS     600                                          /**< Reference voltage (in milli volts) used by ADC whiledoing conversion. */
#define ADC_PRE_SCALING_COMPENSATION      6                                            /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS    270                                          /**< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com. */

#define NUS_SERVICE_UUID_TYPE		BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */
#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define SEC_PARAM_BOND						1												/**< Perform bonding. */
#define SEC_PARAM_MITM						0												/**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC						0												/**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS				0												/**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES	BLE_GAP_IO_CAPS_NONE		/**< No I/O capabilities. */
#define SEC_PARAM_OOB						0												/**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE		7												/**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE		16											/**< Maximum encryption key size. */

#define MIN_CONN_INTERVAL					MSEC_TO_UNITS(500, UNIT_1_25_MS)				/**< Minimum acceptable connection interval (0.5 seconds).  */
#define MAX_CONN_INTERVAL					MSEC_TO_UNITS(1000, UNIT_1_25_MS)			/**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY							0																		/**< Slave latency. */
#define CONN_SUP_TIMEOUT					MSEC_TO_UNITS(4000, UNIT_10_MS)				/**< Connection supervisory timeout (4 seconds). */

#define TX_POWER_LEVEL						(-8)																	/**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */
#define INITIAL_LLS_ALERT_LEVEL			BLE_CHAR_ALERT_LEVEL_NO_ALERT				/**< Initial value for the Alert Level characteristic in the Link Loss service. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY     APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT      3														/**< Number of attempts before giving up the connection parameter negotiation. */


enum notes{
	C1 = 30581L,	C2	= 15290L,	C3	= 7645L,	C4	= 3822L,	C5	= 1911L,	C6	= 955L, C7	= 477L, C8	= 238L,
	D1 = 27247L,	D2	= 13623L,	D3	= 6811L,	D4	= 3404L,	D5	= 1702L,	D6	= 851L, D7	= 425L, D8	= 212L,
	E1 = 24271L,	E2	= 12135L,	E3	= 6067L,	E4	= 3033L,	E5	= 1516L,	E6	= 758L, E7	= 379L, E8	= 189L,
	F1 = 22883L,	F2	= 11454L,	F3	= 5727L,	F4	= 2863L,	F5	= 1431L,	F6	= 715L, F7	= 357L, F8	= 178L,
	G1 = 20408L,	G2	= 10204L,	G3	= 5102L,	G4	= 2551L,	G5	= 1275L,	G6	= 637L, G7	= 318L, G8	= 159L,
	A1 = 18181L,	A2	= 9090L,	A3	= 4545L,	A4	= 2272L,	A5	= 1136L,	A6	= 568L, A7	= 284L, A8	= 142L,
	B1 = 16207L,	B2	= 8097L,	B3	= 4050L,	B4	= 2024L,	B5	= 1012L,	B6	= 506L, B7	= 253L, B8	= 126L
};

static uint16_t								m_conn_handle = BLE_CONN_HANDLE_INVALID;			/**< Handle of the current connection. */

static ble_nus_t							m_nus;								/**< Structure to identify the Nordic UART Service. */
static ble_db_discovery_t				m_ble_db_discovery;			/**< Structure used to identify the DB Discovery module. */
static ble_tps_t								m_tps;									/**< Structure used to identify the TX Power service. */
static ble_ias_t								m_ias;									/**< Structure used to identify the Immediate Alert service. */
static ble_lls_t								m_lls;									/**< Structure used to identify the Link Loss service. */
static ble_bas_t							m_bas;								/**< Structure used to identify the battery service. */
static ble_ias_c_t							m_ias_c;								/**< Structure used to identify the client to the Immediate Alert Service at peer. */

static volatile bool					m_is_door_locked = true;
static volatile bool					m_is_motor_working = false;
static volatile bool					pwm_ready = false;
static volatile bool					m_is_high_alert_signalled;               /**< Variable to indicate whether a high alert has been signalled to the peer. */
static volatile bool					m_is_ias_present = false;                /**< Variable to indicate whether the immediate alert service has been discovered at the connected peer. */

static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_IMMEDIATE_ALERT_SERVICE, BLE_UUID_TYPE_BLE},
														{BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
														{BLE_UUID_TX_POWER_SERVICE, BLE_UUID_TYPE_BLE},
														{BLE_UUID_LINK_LOSS_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */
static ble_uuid_t m_scan_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

#if LEDS_NUMBER > 0
static const uint8_t m_board_led_list[LEDS_NUMBER] = LEDS_LIST;
#endif


APP_TIMER_DEF(m_advertisement_led_timer_id);												/**< LED timer. */
APP_TIMER_DEF(m_motor_stop_timer_id);									/**< Motor stop timer. */
APP_TIMER_DEF(m_battery_timer_id);											/**< Battery measurement timer. */
APP_TIMER_DEF(m_buzzer_timer_id);												/**< Buzzer timer. */
APP_TIMER_DEF(m_door_test_timer_id);									/**< Doorlock test timer. */

APP_PWM_INSTANCE(PWM1, 2);                   // Create the instance "PWM1" using TIMER2.

#ifdef ADC_PRESENT
static nrf_adc_value_t adc_buf[1];
#else
static nrf_saadc_value_t adc_buf[2];
#endif //ADC_PRESENT


static void pwm_start(const enum notes note);
static void advertising_start(void);


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in] nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}


static void led_on(uint32_t led_idx)
{
	//nrf_gpio_pin_write(m_board_led_list[led_idx], LEDS_ACTIVE_STATE ? 1 : 0);
	nrf_gpio_pin_set(m_board_led_list[led_idx]);
}


static void led_off(uint32_t led_idx)
{
	//nrf_gpio_pin_write(m_board_led_list[led_idx], LEDS_ACTIVE_STATE ? 0 : 1);
	nrf_gpio_pin_clear(m_board_led_list[led_idx]);
}


static void leds_on(void)
{
	uint32_t i;
	for(i = 0; i < LEDS_NUMBER; i++)
	{
		led_on(i);
	}
}


static void leds_off(void)
{
	uint32_t i;
	for(i = 0; i < LEDS_NUMBER; i++)
	{
		led_off(i);
	}
}


static void led_invert(uint32_t led_idx)
{
	ASSERT(led_idx < LEDS_NUMBER);
	nrf_gpio_pin_toggle(m_board_led_list[led_idx]);
}


/**@brief Function for initializing the LED.
 */
static void leds_init(void)
{
	uint32_t i;

	// direction set(output)
	for(i = 0; i < LEDS_NUMBER; ++i)
	{
		nrf_gpio_cfg_output(m_board_led_list[i]);
	}
	// board led off
	for(i = 0; i < LEDS_NUMBER; ++i)
	{
		ASSERT(i < LEDS_NUMBER);
		leds_off();
	}
}

static void switch_init(void)
{
	nrf_gpio_cfg_input(SWITCH1, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(SWITCH2, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(SWITCH3, NRF_GPIO_PIN_PULLUP);
}

static void door_lock(void)
{
	uint32_t err_code;

	if(! m_is_door_locked){
		if(! m_is_motor_working)
		{
			NRF_LOG_INFO("lock.\r\n");
			pwm_start(G5);

			m_is_motor_working = true;
			
			nrf_gpio_pin_write(MOTOR_F, 1);
			nrf_gpio_pin_write(MOTOR_R, 0);
			err_code = app_timer_start(m_motor_stop_timer_id, MOTOR_STOP_TIMER_INTERVAL, NULL);
			APP_ERROR_CHECK(err_code);
		}
	}
}

static void door_unlock(void)
{
	uint32_t err_code;
	
	
	if(m_is_door_locked){
		if(! m_is_motor_working)
		{
			NRF_LOG_INFO("unlock.\r\n");
			pwm_start(C5);

			m_is_motor_working = true;

			nrf_gpio_pin_write(MOTOR_F, 0);
			nrf_gpio_pin_write(MOTOR_R, 1);
			err_code = app_timer_start(m_motor_stop_timer_id, MOTOR_STOP_TIMER_INTERVAL, NULL);
			APP_ERROR_CHECK(err_code);
		}
	}
}

static void door_invert(void)
{
	if(m_is_door_locked)
		door_unlock();
	else
		door_lock();
}

static void door_test(void)
{
	//test var set
}

/**@brief Function for initializing the Motor.
 */
static void motors_init(void)
{
    nrf_gpio_cfg_output(MOTOR_F);
    nrf_gpio_cfg_output(MOTOR_R);
}

static void GPIO_init(void)
{
	leds_init();
	switch_init();
	motors_init();
}

/**
 * @brief Handler for timer events.
 */
static void timer_advertisement_led_event_handler(void* p_context)
{
	UNUSED_PARAMETER(p_context);
	static uint32_t timerCnt = 0;

	timerCnt = (++timerCnt) % 10;
	led_off(1);
	if(timerCnt == 0)
		led_on(1);
}

static void timer_motor_stop_event_handler(void* p_context)
{
	UNUSED_PARAMETER(p_context);

	app_timer_stop(m_motor_stop_timer_id);

    nrf_gpio_pin_write(MOTOR_F, 0);
    nrf_gpio_pin_write(MOTOR_R, 0);

	m_is_door_locked = !m_is_door_locked;
	m_is_motor_working = false;

	
	if(m_is_door_locked)
		ble_nus_string_send(&m_nus, "00", sizeof("00")/sizeof(char));
	else
		ble_nus_string_send(&m_nus, "01", sizeof("01")/sizeof(char));		
}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *          This function will start the ADC.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
	#ifdef ADC_PRESENT
    nrf_drv_adc_sample();
    #else // SAADC_PRESENT
    uint32_t err_code;
    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
    #endif // ADC_PRESENT
}

static void timer_buzzer_stop_event_handler(void* p_context)
{
	UNUSED_PARAMETER(p_context);
#ifdef BUZZER
	uint32_t err_code;
	err_code = app_timer_stop(m_buzzer_timer_id);
	APP_ERROR_CHECK(err_code);

	app_pwm_uninit(&PWM1);

	
	NRF_LOG_INFO("PWM STOP\r\n");
	
	
#endif // BUZZER
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

	err_code = app_timer_create(&m_advertisement_led_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_advertisement_led_event_handler);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_create(&m_motor_stop_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_motor_stop_event_handler);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_create(&m_buzzer_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_buzzer_stop_event_handler);
	APP_ERROR_CHECK(err_code);
}

#ifdef ADC_PRESENT
/**@brief Function for handling the ADC driver eevent.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_ADC_EVT_DONE)
    {
        nrf_adc_value_t adc_result;
        uint16_t		batt_lvl_in_milli_volts;
        uint8_t			percentage_batt_lvl;
        uint32_t		err_code;

        adc_result = p_event->data.done.p_buffer[0];

        err_code = nrf_drv_adc_buffer_convert(p_event->data.done.p_buffer, 1);
        APP_ERROR_CHECK(err_code);

        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;
        percentage_batt_lvl = battery_level_in_percent(batt_lvl_in_milli_volts);

        err_code = ble_bas_battery_level_update(&m_bas, percentage_batt_lvl);
        if (
            (err_code != NRF_SUCCESS)
            &&
            (err_code != NRF_ERROR_INVALID_STATE)
            &&
            (err_code != BLE_ERROR_NO_TX_PACKETS)
            &&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
           )
        {
            APP_ERROR_HANDLER(err_code);
        }
    }
}

#else // SAADC_PRESENT
/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;
        uint16_t          batt_lvl_in_milli_volts;
        uint8_t           percentage_batt_lvl;
        uint32_t          err_code;

        adc_result = p_event->data.done.p_buffer[0];

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
        APP_ERROR_CHECK(err_code);

        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;
        percentage_batt_lvl = battery_level_in_percent(batt_lvl_in_milli_volts);

        err_code = ble_bas_battery_level_update(&m_bas, percentage_batt_lvl);
        if (
            (err_code != NRF_SUCCESS)
            &&
            (err_code != NRF_ERROR_INVALID_STATE)
            &&
            (err_code != BLE_ERROR_NO_TX_PACKETS)
            &&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
           )
        {
            APP_ERROR_HANDLER(err_code);
        }
    }
}
#endif // ADC_PRESENT

/**@brief Function for configuring ADC to do battery level conversion.
 */
static void ADC_init(void)
{
    #ifdef ADC_PRESENT

    ret_code_t err_code = nrf_drv_adc_init(NULL, adc_event_handler);
    APP_ERROR_CHECK(err_code);

    static nrf_drv_adc_channel_t channel =
        NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_2);
    // channel.config.config.input = NRF_ADC_CONFIG_SCALING_SUPPLY_ONE_THIRD;
    channel.config.config.input = (uint32_t)NRF_ADC_CONFIG_SCALING_SUPPLY_ONE_THIRD;
    nrf_drv_adc_channel_enable(&channel);

    err_code = nrf_drv_adc_buffer_convert(&adc_buf[0], 1);
    APP_ERROR_CHECK(err_code);
    #else //  SAADC_PRESENT
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
    err_code = nrf_drv_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], 1);
    APP_ERROR_CHECK(err_code);
    #endif //ADC_PRESENT
}

void connect(void)
{
	NRF_LOG_INFO("%s\r\n", (uint32_t)__FUNCTION__);
	ble_nus_string_send(&m_nus, "08", sizeof("08")/sizeof(char));
}

void disconnect(void)
{
	uint32_t err_code;
	NRF_LOG_INFO("%s\r\n", (uint32_t)__FUNCTION__);
	err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	if(err_code != NRF_ERROR_INVALID_STATE)
	{
		APP_ERROR_CHECK(err_code);
	}
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' i.e '\r\n' (hex 0x0D) or if the string has reached a length of
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
static void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||(data_array[index - 1] == '\r') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
				if(data_array[0] == '0')
					switch(data_array[1])
					{
						case '0':
							door_lock();
							break;
						case '1':
							door_unlock();
							break;
						case '2':
							door_invert();
							break;
						case '3':
							door_test();
							break;
						case '8':
							connect();
							break;
						case '9':
							disconnect();
							break;
					}
				
                err_code = ble_nus_string_send(&m_nus, data_array, index);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }

                index = 0;
            }
            break;
        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void UART_init(void)
{
    uint32_t                     err_code;

    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */

static void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    pwm_ready = true;
}

static void pwm_start(const enum notes note)
{
	NRF_LOG_INFO("%s\r\n", (uint32_t)__FUNCTION__);
	uint32_t err_code;
	
    app_pwm_config_t pwm_cfg = APP_PWM_DEFAULT_CONFIG_1CH(note, BUZZER);
    err_code = app_pwm_init(&PWM1, &pwm_cfg, pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);

	NRF_LOG_INFO("PWM INIT\r\n");
	
	
	pwm_ready = false;
	while(app_pwm_channel_duty_set(&PWM1, 0, 50) == NRF_ERROR_BUSY);
	
	
	NRF_LOG_INFO("PWM STARTED\r\n");
	
	
	err_code = app_timer_start(m_buzzer_timer_id, BUZZER_STOP_TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connected.\r\n");
			err_code = app_timer_stop(m_advertisement_led_timer_id);
            APP_ERROR_CHECK(err_code);
			pwm_start(C6);
			led_off(1);
			led_on(0);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = ble_db_discovery_start(&m_ble_db_discovery,
                                              p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
        } break; // BLE_GAP_EVT_CONNECTED
		
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.\r\n");
			pwm_start(C4);
			led_off(0);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_ias_on_ble_evt(&m_ias, p_ble_evt);
    ble_lls_on_ble_evt(&m_lls, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_ias_c_on_ble_evt(&m_ias_c, p_ble_evt);
    ble_tps_on_ble_evt(&m_tps, p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);

    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

	NRF_LOG_INFO("%s\r\n", (uint32_t)__FUNCTION__);

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.\r\n");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
			NRF_LOG_INFO("%u\r\n", p_evt->peer_id);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start();
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}

/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

	NRF_LOG_INFO("%s\r\n", (uint32_t)__FUNCTION__);

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

	NRF_LOG_INFO("%s\r\n", (uint32_t)__FUNCTION__);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);


	NRF_LOG_INFO("sd_ble_gap_device_name_set\r\n");
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

	NRF_LOG_INFO("sd_ble_gap_appearance_set\r\n");
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_KEYRING);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

	NRF_LOG_INFO("sd_ble_gap_ppcp_set\r\n");
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

	NRF_LOG_INFO("sd_ble_gap_tx_power_set\r\n");
    err_code = sd_ble_gap_tx_power_set(TX_POWER_LEVEL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
	uint32_t err_code;
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
			err_code = app_timer_start(m_advertisement_led_timer_id, ADVERTISEMENT_LED_TIMER_INTERVAL, NULL);
			APP_ERROR_CHECK(err_code);
            break; // BLE_ADV_EVT_FAST

        case BLE_ADV_EVT_IDLE:
			advertising_start();

            // sleep_mode_enter();
            break; // BLE_ADV_EVT_IDLE

        default:
            break;
    }
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
	uint32_t               err_code;
	ble_advdata_t          advdata;
	ble_advdata_t          scanrsp;
	ble_adv_modes_config_t options;

	// Build and set advertising data.
	memset(&advdata, 0, sizeof(advdata));
	advdata.name_type               = BLE_ADVDATA_FULL_NAME;
	advdata.include_appearance      = true;
	advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;;
	advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	advdata.uuids_complete.p_uuids  = m_adv_uuids;

	memset(&scanrsp, 0, sizeof(scanrsp));
	scanrsp.uuids_complete.uuid_cnt = sizeof(m_scan_uuids) / sizeof(m_scan_uuids[0]);
	scanrsp.uuids_complete.p_uuids  = m_scan_uuids;

	memset(&options, 0, sizeof(options));
	options.ble_adv_fast_enabled  = true;
	options.ble_adv_fast_interval = APP_ADV_INTERVAL;
	options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

	err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_ias_c_on_db_disc_evt(&m_ias_c, p_evt);
}

/** @brief Database discovery module initialization.
 */
static void db_discovery_init(void)
{
	NRF_LOG_INFO("%s\r\n", (uint32_t)__FUNCTION__);

    uint32_t err_code = ble_db_discovery_init(db_disc_handler);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the TX Power Service.
 */
static void tps_init(void)
{
    uint32_t       err_code;
    ble_tps_init_t tps_init_obj;

    memset(&tps_init_obj, 0, sizeof(tps_init_obj));
    tps_init_obj.initial_tx_power_level = TX_POWER_LEVEL;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&tps_init_obj.tps_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&tps_init_obj.tps_attr_md.write_perm);

    err_code = ble_tps_init(&m_tps, &tps_init_obj);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Signals alert event from Immediate Alert or Link Loss services.
 *
 * @param[in] alert_level  Requested alert level.
 */
static void alert_signal(uint8_t alert_level)
{
	NRF_LOG_INFO("alert_level : %u\r\n", alert_level);
    switch (alert_level)
    {
        case BLE_CHAR_ALERT_LEVEL_NO_ALERT:
            NRF_LOG_INFO("No Alert.\r\n");
            break; // BLE_CHAR_ALERT_LEVEL_NO_ALERT

        case BLE_CHAR_ALERT_LEVEL_MILD_ALERT:
            NRF_LOG_INFO("MILD_ALERT.\r\n");
            break; // BLE_CHAR_ALERT_LEVEL_MILD_ALERT

        case BLE_CHAR_ALERT_LEVEL_HIGH_ALERT:
            NRF_LOG_INFO("HIGH_ALERT.\r\n");
            break; // BLE_CHAR_ALERT_LEVEL_HIGH_ALERT

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling Immediate Alert events.
 *
 * @details This function will be called for all Immediate Alert events which are passed to the
 *          application.
 *
 * @param[in] p_ias  Immediate Alert structure.
 * @param[in] p_evt  Event received from the Immediate Alert service.
 */
static void on_ias_evt(ble_ias_t * p_ias, ble_ias_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_IAS_EVT_ALERT_LEVEL_UPDATED:
            alert_signal(p_evt->params.alert_level);
            break; // BLE_IAS_EVT_ALERT_LEVEL_UPDATED

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initializing the Immediate Alert Service.
 */
static void ias_init(void)
{
    uint32_t       err_code;
    ble_ias_init_t ias_init_obj;

    memset(&ias_init_obj, 0, sizeof(ias_init_obj));
    ias_init_obj.evt_handler = on_ias_evt;

    err_code = ble_ias_init(&m_ias, &ias_init_obj);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Link Loss events.
 *
 * @details This function will be called for all Link Loss events which are passed to the
 *          application.
 *
 * @param[in] p_lls  Link Loss structure.
 * @param[in] p_evt  Event received from the Link Loss service.
 */
static void on_lls_evt(ble_lls_t * p_lls, ble_lls_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_LLS_EVT_LINK_LOSS_ALERT:
            alert_signal(p_evt->params.alert_level);
            break; // BLE_LLS_EVT_LINK_LOSS_ALERT

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initializing the Link Loss Service.
 */
static void lls_init(void)
{
    uint32_t       err_code;
    ble_lls_init_t lls_init_obj;

    // Initialize Link Loss Service
    memset(&lls_init_obj, 0, sizeof(lls_init_obj));

    lls_init_obj.evt_handler         = on_lls_evt;
    lls_init_obj.error_handler       = service_error_handler;
    lls_init_obj.initial_alert_level = INITIAL_LLS_ALERT_LEVEL;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&lls_init_obj.lls_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&lls_init_obj.lls_attr_md.write_perm);

    err_code = ble_lls_init(&m_lls, &lls_init_obj);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Battery Service events.
 *
 * @details This function will be called for all Battery Service events which are passed to the
 |          application.
 *
 * @param[in] p_bas  Battery Service structure.
 * @param[in] p_evt  Event received from the Battery Service.
 */
static void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t * p_evt)
{
	uint32_t err_code;
    switch (p_evt->evt_type)
    {
        case BLE_BAS_EVT_NOTIFICATION_ENABLED:
            // Start battery timer
            err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_BAS_EVT_NOTIFICATION_ENABLED

        case BLE_BAS_EVT_NOTIFICATION_DISABLED:
            err_code = app_timer_stop(m_battery_timer_id);
            APP_ERROR_CHECK(err_code);
            break; // BLE_BAS_EVT_NOTIFICATION_DISABLED

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initializing the Battery Service.
 */
static void bas_init(void)
{
    uint32_t       err_code;
    ble_bas_init_t bas_init_obj;

    memset(&bas_init_obj, 0, sizeof(bas_init_obj));

    bas_init_obj.evt_handler          = on_bas_evt;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
    bas_init_obj.initial_batt_level   = 100;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init_obj.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init_obj.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init_obj.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init_obj.battery_level_report_read_perm);

    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while(app_uart_put('\r') != NRF_SUCCESS);
    while(app_uart_put('\n') != NRF_SUCCESS);

	switch(p_data[0])
	{
		// 0 -> lock
		case 0:
		case '0':
			door_lock();
			break;

		// 1 -> unlock
		case 1:
		case '1':
			door_unlock();
			break;

		// 2 -> invert
		case 2:
		case '2':
			door_invert();
			break;

		// 3 -> test
		case 3:
		case'3':
			door_test();
			break;

		// 8 -> disconnect
		case 8:
		case'8':
			connect();
			break;

		// 9 -> disconnect
		case 9:
		case'9':
			disconnect();
			break;

		default:
			break;
	}
}
/**@snippet [Handling the data received over BLE] */

/**@brief Function for initializing the Nordic Uart Service.
 *
 * @details
 */
static void nus_init(void)
{
    uint32_t         err_code;
    ble_nus_init_t nus_init_obj;

    memset(&nus_init_obj, 0, sizeof(nus_init_obj));

    nus_init_obj.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init_obj);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling IAS Client events.
 *
 * @details This function will be called for all IAS Client events which are passed to the
 *          application.
 *
 * @param[in] p_ias_c  IAS Client structure.
 * @param[in] p_evt    Event received.
 */
static void on_ias_c_evt(ble_ias_c_t * p_ias_c, ble_ias_c_evt_t * p_evt)
{
    uint32_t err_code;

    switch (p_evt->evt_type)
    {
        case BLE_IAS_C_EVT_DISCOVERY_COMPLETE:
            // IAS is found on peer. The Find Me Locator functionality of this app will work.
            err_code = ble_ias_c_handles_assign(&m_ias_c,
                                                p_evt->conn_handle,
                                                p_evt->alert_level.handle_value);
            APP_ERROR_CHECK(err_code);

            m_is_ias_present = true;
            break; // BLE_IAS_C_EVT_DISCOVERY_COMPLETE

        case BLE_IAS_C_EVT_DISCOVERY_FAILED:
            // IAS is not found on peer. The Find Me Locator functionality of this app will NOT work.
            break; // BLE_IAS_C_EVT_DISCOVERY_FAILED

        case BLE_IAS_C_EVT_DISCONN_COMPLETE:
            // Disable alert buttons
            m_is_ias_present = false;
            break; // BLE_IAS_C_EVT_DISCONN_COMPLETE

        default:
            break;
    }
}

/**@brief Function for initializing the immediate alert service client.
 *
 * @details This will initialize the client side functionality of the Find Me profile.
 */
static void ias_client_init(void)
{
    uint32_t         err_code;
    ble_ias_c_init_t ias_c_init_obj;

    memset(&ias_c_init_obj, 0, sizeof(ias_c_init_obj));

    m_is_high_alert_signalled = false;

    ias_c_init_obj.evt_handler   = on_ias_c_evt;
    ias_c_init_obj.error_handler = service_error_handler;

    err_code = ble_ias_c_init(&m_ias_c, &ias_c_init_obj);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the services that will be used by the application.
 */
static void services_init(void)
{
	NRF_LOG_INFO("%s\r\n", (uint32_t)__FUNCTION__);

	tps_init();
	ias_init();
	lls_init();
	bas_init();
	nus_init();
	ias_client_init();
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code = NRF_SUCCESS;
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
    uint32_t err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

/** @brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

int main(void)
{
	uint32_t err_code;

	err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

	GPIO_init();

	led_on(0);

	timers_init();
	ADC_init();
	UART_init();

	ble_stack_init();
	// params : bonds_erase select
	peer_manager_init(false);
	gap_params_init();
	db_discovery_init();
	services_init();
	conn_params_init();
	advertising_init();

//	door_lock();
	led_off(0);
	NRF_LOG_INFO("Initialized\r\n");

	// Start execution.
    advertising_start();

    // Enter main loop.
	while(true)
	{
		power_manage();
	}

}
/**
 *@}
 **/
