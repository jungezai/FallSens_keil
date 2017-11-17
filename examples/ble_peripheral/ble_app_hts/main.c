/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_hts_main main.c
 * @{
 * @ingroup ble_sdk_app_hts
 * @brief Health Thermometer Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Health Thermometer service
 * It also includes the sample code for Battery and Device Information services.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hts.h"
#include "ble_dis.h"
#ifdef BLE_DFU_APP_SUPPORT
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#endif // BLE_DFU_APP_SUPPORT
#include "ble_conn_params.h"
#include "boards.h"
#include "sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_delay.h"

#include "mpu6050.h"
#include "twi_master.h"





//lint -e553
#ifdef SVCALL_AS_NORMAL_FUNCTION
#include "ser_phy_debug_app.h"
#endif




#define CFX_BOARD
//#define DIAPER
#define FALLSENS

#define IS_SRVC_CHANGED_CHARACT_PRESENT 1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define SEND_MEAS_BUTTON_ID             0                                          /**< Button used for sending a measurement. */
#ifndef CFX_BOARD
#define DEVICE_NAME                     "Nordic_HTS"                               /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                       "NS-HTS-EXAMPLE"                            /**< Model number. Will be passed to Device Information Service. */
#else
#ifdef DIAPER
#define DEVICE_NAME                     "CFX_DIAPER"                               /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "CFX TECHNOLOGY"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                       "CFX-HTS-EXAMPLE"                            /**< Model number. Will be passed to Device Information Service. */
#endif

#ifdef FALLSENS
#define DEVICE_NAME                     "CFX_FALLSENS"                               /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "CFX TECHNOLOGY"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                       "CFX-HTS-EXAMPLE"                            /**< Model number. Will be passed to Device Information Service. */
#endif

#endif
#define MANUFACTURER_ID                 0x1122334455                               /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   0x667788                                   /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_ADV_INTERVAL                40                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                        /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER             0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                          /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL               81                                         /**< Minimum battery level as returned by the simulated measurement function. */
#define MAX_BATTERY_LEVEL               100                                        /**< Maximum battery level as returned by the simulated measurement function. */
#define BATTERY_LEVEL_INCREMENT         1                                          /**< Value by which the battery level is incremented/decremented for each call to the simulated measurement function. */

#define HEART_RATE_MEAS_INTERVAL_SEND        APP_TIMER_TICKS(100, APP_TIMER_PRESCALER) /**< Heart rate measurement interval (ticks). */


#define HEART_RATE_MEAS_INTERVAL        APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Heart rate measurement interval (ticks). */
#define TEMP_TYPE_AS_CHARACTERISTIC     0                                          /**< Determines if temperature type is given as characteristic (1) or as a field of measurement (0). */

#define MIN_CELCIUS_DEGREES             3688                                       /**< Minimum temperature in celcius for use in the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */
#define MAX_CELCIUS_DEGRESS             3972                                       /**< Maximum temperature in celcius for use in the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */
#define CELCIUS_DEGREES_INCREMENT       36                                         /**< Value by which temperature is incremented/decremented for each call to the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(25, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.5 seconds) */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(50, UNIT_1_25_MS)          /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   5                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(13000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of indication) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


/**for HTD21D **/
#ifdef CFX_BOARD

#ifdef DIAPER
#define I2C_SCL 18
#define I2C_SDA 17
#define RED 5
#define GREEN 6
#define ON_OFF 15
#define POWER_IN 16
#endif

#ifdef FALLSENS
#define I2C_SCL 0
#define I2C_SDA 1
#define RED 22
#define GREEN 24
#define ON_OFF 8
#define POWER_IN 7 
int16_t tem1[3];
//float temp1[3];


#define SAMPLE_SIZE 1
	int16_t tem1[3];
	float temp1[SAMPLE_SIZE][3];



#endif



#else
#define I2C_SCL 0
#define I2C_SDA 1
#endif
#define COMMAND_SOFT_RESET 0XFE
#define COMMAND_QUERY_TEMPURATURE 0XE3
#define COMMAND_QUERY_HUMIDITY 0XE5
#define HTU21D_ADDR 0X80
#define ERR_NO_ACK 200
void I2C_Pin_Configuration(char send);
void I2C_Start();
void I2C_Stop();
void I2C_Tx(unsigned char txbyte);
unsigned char I2C_Rx();
void I2C_SendAck(char ack);
void I2C_Release_Bus();
void HTU21D_Init();
float HTU21D_Read(char command);
 unsigned short num;
 float voltage,battery;
void LED_On(char led_num);
void LED_Off(char led_num);
#ifdef BLE_DFU_APP_SUPPORT
#define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                    0x01                                       /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
#define APP_SERVICE_HANDLE_START         0x000C                                     /**< Handle of first application specific service when when service changed characteristic is present. */
#define BLE_HANDLE_MAX                   0xFFFF                                     /**< Max handle value in BLE. */

STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                                     /** When having DFU Service support in application the Service Changed Characteristic should always be present. */
#endif // BLE_DFU_APP_SUPPORT





static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
static ble_bas_t                        m_bas;                                     /**< Structure used to identify the battery service. */
static ble_hts_t                        m_hts;                                     /**< Structure used to identify the health thermometer service. */

static sensorsim_cfg_t                  m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t                m_battery_sim_state;                       /**< Battery Level sensor simulator state. */
static bool                             m_hts_meas_ind_conf_pending = false;       /**< Flag to keep track of when an indication confirmation is pending. */

static sensorsim_cfg_t                  m_temp_celcius_sim_cfg;                    /**< Temperature simulator configuration. */
static sensorsim_state_t                m_temp_celcius_sim_state;                  /**< Temperature simulator state. */

APP_TIMER_DEF(m_battery_timer_id);                                                 /**< Battery timer. */
APP_TIMER_DEF(m_heart_rate_timer_id);                                               /**< Heart rate measurement timer. */
static dm_application_instance_t        m_app_handle;                              /**< Application identifier allocated by device manager */

static    ble_uuid_t m_adv_uuids[] = {{BLE_UUID_HEALTH_THERMOMETER_SERVICE, BLE_UUID_TYPE_BLE},
                                      {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
                                      {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */
#ifdef BLE_DFU_APP_SUPPORT    
static ble_dfu_t                         m_dfus;                                    /**< Structure used to identify the DFU service. */
#endif // BLE_DFU_APP_SUPPORT    


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


#ifdef CFX_BOARD
#ifdef DIAPER
 unsigned short nrf_adc_read(void)  //读取数据
{
  unsigned short adc_data;
  NRF_ADC->TASKS_START = 1;
  while(NRF_ADC->EVENTS_END == 0);
  NRF_ADC->EVENTS_END = 0;
  adc_data = NRF_ADC->RESULT;
  return adc_data;
}
#endif
#endif

/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
static void battery_level_update(float a)
{
    uint32_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);
	#ifdef CFX_BOARD
	#ifdef DIAPER
	num=nrf_adc_read();
	voltage = (num*3*2)*1.2/1024;
	battery_level = (voltage-2.7)/0.6*100;
	#endif
	#endif
	battery_level = (char)(a*10/2);
	if(a < 0)
	{
		battery_level = (battery_level)|(0x80);
	}
    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
//关机函数
static void battery_level_meas_timeout_handler(void * p_context)
{
		static char i = 0;
		UNUSED_PARAMETER(p_context);
		battery_level_update((float)(1.0));
		//LED_On(6);


#ifdef CFX_BOARD

	
		if( nrf_gpio_pin_read(ON_OFF)== 0 )
		{
			i++;
			LED_On(RED);
			if(i == 2) {
				nrf_gpio_pin_clear(POWER_IN);
				LED_Off(RED);
				i = 0;
			}

		}	
		else {
					i = 0;
					LED_Off(RED);
			
	}
		
#endif // CFX_BOARD




	
	
	

}

    uint32_t celciusX100;
/**@brief Function for populating simulated health thermometer measurement.
 */
static void hts_sim_measurement(ble_hts_meas_t * p_meas,float a)
{
    static ble_date_time_t time_stamp = { 2012, 12, 5, 11, 50, 0 };
#if 1
		static char is_temp = 0;

		if(!is_temp) {
			p_meas->temp_in_fahr_units = false;
			is_temp++;
		} else {
			p_meas->temp_in_fahr_units = true;
			is_temp--;
		}
#else
		p_meas->temp_in_fahr_units = false;
#endif

		p_meas->time_stamp_present = true;
    p_meas->temp_type_present  = (TEMP_TYPE_AS_CHARACTERISTIC ? false : true);

    celciusX100 = sensorsim_measure(&m_temp_celcius_sim_state, &m_temp_celcius_sim_cfg);
		//celciusX100 = (unsigned int)(HTU21D_Read(COMMAND_QUERY_TEMPURATURE)*100);


#if 0
		static uint32_t change_num = 0;
		celciusX100 = change_num*100;
		change_num++;
		if(change_num == 9)
			change_num = 0;
#endif
    p_meas->temp_in_celcius.exponent = -2;
    //p_meas->temp_in_celcius.mantissa = celciusX100;
		p_meas->temp_in_celcius.mantissa = a;
    p_meas->temp_in_fahr.exponent    = -2;
    p_meas->temp_in_fahr.mantissa    = (32 * 100) + ((celciusX100 * 9) / 5);
		p_meas->temp_in_fahr.mantissa    = a;
    p_meas->time_stamp               = time_stamp;
    p_meas->temp_type                = BLE_HTS_TEMP_TYPE_FINGER;

    // update simulated time stamp
    time_stamp.seconds += 27;
    if (time_stamp.seconds > 59)
    {
        time_stamp.seconds -= 60;
        time_stamp.minutes++;
        if (time_stamp.minutes > 59)
        {
            time_stamp.minutes = 0;
        }
    }
}

	 int current_index = 0;
/**@brief Function for simulating and sending one Temperature Measurement.
 */
static void temperature_measurement_send(void)
{
    ble_hts_meas_t simulated_meas;
    uint32_t       err_code;
		bool     is_indication_enabled;
	static bool is_collect_finish = false;
    //err_code = ble_hts_is_indication_enabled(&m_hts, &is_indication_enabled);
    //APP_ERROR_CHECK(err_code);
		is_indication_enabled = true;
		m_hts_meas_ind_conf_pending = false;

	
	
	if((current_index == 0)&&(is_collect_finish == false)){
    if (is_indication_enabled&&!m_hts_meas_ind_conf_pending)
    {
			
			#ifdef CFX_BOARD
		
#ifdef FALLSENS

			LED_On(GREEN);
			
			
	#if 1
			
			
			
				err_code = app_timer_stop(m_heart_rate_timer_id);
				
			for(int m = 0;m < SAMPLE_SIZE;m++)
		{
		MPU6050_ReadAcc( &tem1[0], &tem1[1] , &tem1[2] );
		//MPU6050_ReadGyro(&tem2[0] , &tem2[1] , &tem2[2] );
		//MPU6050_ReadTmp( &tmp);
		//printf("ACC:\r");
		for(int i = 0;i < 3;i++)
		{
			temp1[m][i] = (float)2*9.8*tem1[i]/32768;
			//printf("  %.2f  \r",temp1[i]);
		}

		//printf("\r\r GYRO:\r");

		//nrf_delay_us(50000);
		
	  }
		
		
	#endif	
		
		
		
		LED_Off(GREEN);
		//current_index = 0;
		is_collect_finish = true;
		err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL_SEND, NULL);
		}
	}
		
		
		
		
		
		
		
		#endif
		#endif
		LED_On(RED);
			//hts_sim_measurement(&simulated_meas,123,123);
		//err_code = ble_hts_measurement_send(&m_hts, &simulated_meas);
		//nrf_delay_us(1000);
			float *xx = (float *)temp1;
			//for(int i = 0; i < 3 * SAMPLE_SIZE ; i++){
				//for(int i = 0; i < 1 ; i++){
				if(current_index < SAMPLE_SIZE ) {
       // hts_sim_measurement(&simulated_meas,xx[current_index]*100);
			
				
			
			
       err_code = ble_hts_measurement_send(&m_hts, &simulated_meas);
						current_index = current_index++;
					//if(current_index == 1){
					//err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL_SEND, NULL);
					//}
				} else {
					current_index = 0;
					is_collect_finish = false;
				}
		//nrf_delay_us(1000000);
			
			//}
			LED_Off(RED);
		
# if 0
        switch (err_code)
        {
            case NRF_SUCCESS:
                // Measurement was successfully sent, wait for confirmation.
                m_hts_meas_ind_conf_pending = true;
                break;

            case NRF_ERROR_INVALID_STATE:
                // Ignore error.
                break;

            default:
                APP_ERROR_HANDLER(err_code);
                break;
        }
#endif
    
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
	#if 1
    err_code = app_timer_create(&m_heart_rate_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                temperature_measurement_send);
	#endif
    APP_ERROR_CHECK(err_code);
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

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_THERMOMETER);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

#ifdef BLE_DFU_APP_SUPPORT
/**@brief Function for stopping advertising.
 */
static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for loading application-specific context after establishing a secure connection.
 *
 * @details This function will load the application context and check if the ATT table is marked as 
 *          changed. If the ATT table is marked as changed, a Service Changed Indication
 *          is sent to the peer if the Service Changed CCCD is set to indicate.
 *
 * @param[in] p_handle The Device Manager handle that identifies the connection for which the context 
 *                     should be loaded.
 */
static void app_context_load(dm_handle_t const * p_handle)
{
    uint32_t                 err_code;
    static uint32_t          context_data;
    dm_application_context_t context;

    context.len    = sizeof(context_data);
    context.p_data = (uint8_t *)&context_data;

    err_code = dm_application_context_get(p_handle, &context);
    if (err_code == NRF_SUCCESS)
    {
        // Send Service Changed Indication if ATT table has changed.
        if ((context_data & (DFU_APP_ATT_TABLE_CHANGED << DFU_APP_ATT_TABLE_POS)) != 0)
        {
            err_code = sd_ble_gatts_service_changed(m_conn_handle, APP_SERVICE_HANDLE_START, BLE_HANDLE_MAX);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
            {
                APP_ERROR_HANDLER(err_code);
            }
        }

        err_code = dm_application_context_delete(p_handle);
        APP_ERROR_CHECK(err_code);
    }
    else if (err_code == DM_NO_APP_CONTEXT)
    {
        // No context available. Ignore.
    }
    else
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/** @snippet [DFU BLE Reset prepare] */
/**@brief Function for preparing for system reset.
 *
 * @details This function implements @ref dfu_app_reset_prepare_t. It will be called by 
 *          @ref dfu_app_handler.c before entering the bootloader/DFU.
 *          This allows the current running application to shut down gracefully.
 */
static void reset_prepare(void)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);

    nrf_delay_ms(500);
}
/** @snippet [DFU BLE Reset prepare] */
#endif // BLE_DFU_APP_SUPPORT

#if 0
/**@brief Function for simulating and sending one Temperature Measurement.
 */
static void temperature_measurement_send(void)
{
    ble_hts_meas_t simulated_meas;
    uint32_t       err_code;
		bool     is_indication_enabled;
    err_code = ble_hts_is_indication_enabled(&m_hts, &is_indication_enabled);
    APP_ERROR_CHECK(err_code);

    if (is_indication_enabled&&!m_hts_meas_ind_conf_pending)
    {
        hts_sim_measurement(&simulated_meas);

        err_code = ble_hts_measurement_send(&m_hts, &simulated_meas);

        switch (err_code)
        {
            case NRF_SUCCESS:
                // Measurement was successfully sent, wait for confirmation.
                m_hts_meas_ind_conf_pending = true;
                break;

            case NRF_ERROR_INVALID_STATE:
                // Ignore error.
                break;

            default:
                APP_ERROR_HANDLER(err_code);
                break;
        }
    }
}
#endif

/**@brief Function for handling the Health Thermometer Service events.
 *
 * @details This function will be called for all Health Thermometer Service events which are passed
 *          to the application.
 *
 * @param[in] p_hts  Health Thermometer Service structure.
 * @param[in] p_evt  Event received from the Health Thermometer Service.
 */
#if 0
static void on_hts_evt(ble_hts_t * p_hts, ble_hts_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HTS_EVT_INDICATION_ENABLED:
            // Indication has been enabled, send a single temperature measurement
            temperature_measurement_send();
            break;

        case BLE_HTS_EVT_INDICATION_CONFIRMED:
            m_hts_meas_ind_conf_pending = false;
            break;

        default:
            // No implementation needed.
            break;
    }
}
#endif

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Health Thermometer, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t         err_code;
    ble_hts_init_t   hts_init;
    ble_bas_init_t   bas_init;
    ble_dis_init_t   dis_init;
    ble_dis_sys_id_t sys_id;

    // Initialize Health Thermometer Service
    memset(&hts_init, 0, sizeof(hts_init));

    //hts_init.evt_handler                 = on_hts_evt;
	    hts_init.evt_handler                 = NULL;
    hts_init.temp_type_as_characteristic = TEMP_TYPE_AS_CHARACTERISTIC;
    hts_init.temp_type                   = BLE_HTS_TEMP_TYPE_BODY;

    // Here the sec level for the Health Thermometer Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.hts_meas_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hts_init.hts_meas_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hts_init.hts_meas_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.hts_temp_type_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hts_init.hts_temp_type_attr_md.write_perm);

    err_code = ble_hts_init(&m_hts, &hts_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str,     MODEL_NUM);

    sys_id.manufacturer_id            = MANUFACTURER_ID;
    sys_id.organizationally_unique_id = ORG_UNIQUE_ID;
    dis_init.p_sys_id                 = &sys_id;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [DFU BLE Service initialization] */
    ble_dfu_init_t   dfus_init;

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.error_handler = NULL;
    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.revision      = DFU_REVISION;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);

    dfu_app_reset_prepare_set(reset_prepare);
    dfu_app_dm_appl_instance_set(m_app_handle);
    /** @snippet [DFU BLE Service initialization] */
#endif // BLE_DFU_APP_SUPPORT

}


/**@brief Function for initializing the sensor simulators.
 */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);

    // Temperature is in celcius (it is multiplied by 100 to avoid floating point arithmetic).
    m_temp_celcius_sim_cfg.min          = MIN_CELCIUS_DEGREES;
    m_temp_celcius_sim_cfg.max          = MAX_CELCIUS_DEGRESS;
    m_temp_celcius_sim_cfg.incr         = CELCIUS_DEGREES_INCREMENT;
    m_temp_celcius_sim_cfg.start_at_max = false;

    sensorsim_init(&m_temp_celcius_sim_state, &m_temp_celcius_sim_cfg);
}



/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
	#if 1
    err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
	#endif
    APP_ERROR_CHECK(err_code);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
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


/**@brief Function for handling a Connection Parameters error.
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
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_hts.meas_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
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
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
#if 0
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle               = BLE_CONN_HANDLE_INVALID;
            m_hts_meas_ind_conf_pending = false;
            break;

        case BLE_GATTS_EVT_TIMEOUT:

            if (p_ble_evt->evt.gatts_evt.params.timeout.src == BLE_GATT_TIMEOUT_SRC_PROTOCOL)
            {
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}
#else
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
            {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        default:
            // No implementation needed.
            break;
    }
}
#endif

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_hts_on_ble_evt(&m_hts, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    dm_ble_evt_handler(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [Propagating BLE Stack events to DFU Service] */
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    /** @snippet [Propagating BLE Stack events to DFU Service] */
#endif // BLE_DFU_APP_SUPPORT
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
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);
	//NRF_CLOCK_LFCLKSRC_XTAL_20_PPM
	//NRF_CLOCK_LFCLKSRC_RC_250_PPM_250MS_CALIBRATION
    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#if (defined(S130) || defined(S132))
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_KEY_0:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                temperature_measurement_send();
            }
            break;

        default:
            break;
    }
}
/*******timer init******/
#define LED_0          19
#define LED_1          20  

void led_init(void)  //led-pin初始化
{
  // Configure LED-pins as outputs
  nrf_gpio_cfg_output(LED_0);
  nrf_gpio_cfg_output(LED_1);
}
void LED1_Toggle(void)
{
  nrf_gpio_pin_toggle(LED_0);  //电平翻转
}
void time1_init(void)  //定时器初始化,定时1s
{

    NRF_TIMER2->MODE        = TIMER_MODE_MODE_Timer;
    NRF_TIMER2->PRESCALER   = 9;  //Ftimer  = 31250 Hz   =32us
 
    NRF_TIMER2->CC[2]       = (10000U);  //=32us*31250=1s
    NRF_TIMER2->INTENSET    = TIMER_INTENSET_COMPARE2_Enabled << TIMER_INTENSET_COMPARE2_Pos;

 
    NRF_TIMER2->SHORTS      = (TIMER_SHORTS_COMPARE1_CLEAR_Enabled << TIMER_SHORTS_COMPARE1_CLEAR_Pos);
		
		NVIC_ClearPendingIRQ(TIMER2_IRQn);
    NVIC_SetPriority(TIMER2_IRQn,3);
		NVIC_EnableIRQ(TIMER2_IRQn);  
	  NRF_TIMER2->TASKS_START = 1; //开启定时器
}


void TIMER2_IRQHandler(void)  //定时器中断模式服务函数
{

    if ((NRF_TIMER2->EVENTS_COMPARE[2] == 1) && (NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE2_Msk))
    {
      NRF_TIMER2->EVENTS_COMPARE[2] = 0;
   
      LED1_Toggle();//电平翻转void LED2_Toggle(void)
	    temperature_measurement_send();

		 	NRF_TIMER2->TASKS_CLEAR = 1; //清楚计算
    }
 
}
/*******timer init******/



/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    uint32_t err_code;
    bool     is_indication_enabled;

    switch(p_event->event_id)
    {
        case DM_EVT_LINK_SECURED:
            // Send a single temperature measurement if indication is enabled.
            // NOTE: For this to work, make sure ble_hts_on_ble_evt() is called before
            //       dm_ble_evt_handler() in ble_evt_dispatch().
            err_code = ble_hts_is_indication_enabled(&m_hts, &is_indication_enabled);
            APP_ERROR_CHECK(err_code);

            if (is_indication_enabled)
            {
								//led_init();  //初始化led
                temperature_measurement_send();
								//time1_init(); //初始化time；
            }
			#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }
#endif // BLE_DFU_APP_SUPPORT
			
			
			
            break;

        default:
            break;
    }

    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));
    
    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}
#ifdef CFX_BOARD
void OnOff_KEY_Init(void)
{
	
  nrf_gpio_cfg_input(ON_OFF,NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_output(POWER_IN);
  //nrf_gpio_pin_clear(16);			//add by ljc 20161108

}

uint8_t OnOff_KEY_Detect(void)
{
	char i;
	for(i = 0;i < 3;i++)
	{
		if( nrf_gpio_pin_read(ON_OFF)== 0 )
		{	
			nrf_delay_ms(500);
			LED_On(GREEN);
			nrf_delay_ms(100);
			LED_Off(GREEN);
		}
		else
			nrf_gpio_pin_clear(POWER_IN);
	}
	
	nrf_gpio_pin_set(POWER_IN);

}




void LED_Init()
{
  nrf_gpio_cfg_output(RED);
  nrf_gpio_cfg_output(GREEN);
}

void LED_On(char led_num)
{
#ifdef DIAPER	
	
	nrf_gpio_pin_set(led_num);
	#endif

#ifdef FALLSENS
		nrf_gpio_pin_clear(led_num);
		#endif
	
}
void LED_Off(char led_num)
{
	#ifdef DIAPER	
	
	nrf_gpio_pin_clear(led_num);
		#endif

#ifdef FALLSENS
	
	nrf_gpio_pin_set(led_num);
		#endif
}
#endif
//float temp;
/**@brief Function for application main entry.
 */
void adc_init(unsigned char ADC_res,unsigned char ADC_input_selection,unsigned char ADC_interrupt_enabled)
{
 	
   if(ADC_input_selection <= 7)
     {                     //多小位                         //BBB=2
       NRF_ADC->CONFIG = ADC_res << ADC_CONFIG_RES_Pos |  ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos |
                          0 << ADC_CONFIG_REFSEL_Pos | (1 << ADC_input_selection) << ADC_CONFIG_PSEL_Pos;
                          //cc=3
			 NRF_ADC->ENABLE = 1;  
       
    }
 }



int main(void)
{
    uint32_t err_code;
	uint16_t i;
    bool erase_bonds;
	#ifdef CFX_BOARD
		LED_Off(GREEN);
	LED_Off(RED);
		//NRF_POWER->DCDCEN = (POWER_DCDCEN_DCDCEN_Enabled & POWER_DCDCEN_DCDCEN_Msk) << POWER_DCDCEN_DCDCEN_Pos;
		/***onoff key init***/
		OnOff_KEY_Init();
		LED_Init();
		nrf_delay_ms(500);
		nrf_delay_ms(500);
		/***whether key down for 3s or not***/
		OnOff_KEY_Detect();

		LED_On(GREEN);
		for(i = 0;i < 4;i++)
		{
			nrf_delay_ms(500);
		}
		LED_Off(GREEN);
		//LED_On(5);
		//LED_On(6);
		
		
		#ifdef DIAPER
		
		adc_init(2,2,0);
		#endif
		HTU21D_Init();
		//#define DIAPER
	#endif
		//HTU21D_Init();
		

    // Initialize.
		#ifdef FALLSENS
		uint8_t id;
		twi_master_init();
		
		
			if(mpu6050_init(0x68) == false)
	{
		//printf("mpu6050 init fail\r\n");
	}else{
		//printf("mpu6050 init succeeded\r\n");
	}
	nrf_delay_us(100);
	mpu6050_register_read(0x75U, &id, 1);	
	//printf("mpu6050 id is 0x%x \r\n",id);
		
		
		
		
		
				MPU6050_ReadAcc( &tem1[0], &tem1[1] , &tem1[2]);
		celciusX100 =  tem1[0];
		
		
		
		
		
		
		
		
		
		
		#endif
		
	#if 1
		
    app_trace_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    device_manager_init(erase_bonds);
	gap_params_init();
	advertising_init();
	services_init();
    sensor_simulator_init();
    conn_params_init();

    // Start execution.
    application_timers_start();
		
	#if 1
		err_code =  sd_ble_gap_tx_power_set(0);  //
		APP_ERROR_CHECK(err_code);
	#endif
	
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    // Enter main loop.
    for (;; )
    {
        power_manage();
				
    }
	#endif
		#if 0
		while(1) {
		temp = HTU21D_Read_Temp();
		nrf_delay_ms(1000);
		}
		#endif
}


/**
 * @}
 */

void I2C_Pin_Configuration(char send)
{
	if(send){
	nrf_gpio_cfg_output(I2C_SCL);
  nrf_gpio_cfg_output(I2C_SDA);
	} else {
	nrf_gpio_cfg_output(I2C_SCL);
  	nrf_gpio_cfg_input(I2C_SDA, NRF_GPIO_PIN_PULLUP);
	}
}

void I2C_Start()
{
	I2C_Pin_Configuration(1);
	nrf_gpio_pin_clear(I2C_SCL);
	nrf_gpio_pin_set(I2C_SDA);
	nrf_delay_us(1);
	nrf_gpio_pin_set(I2C_SCL);
	/*start time over 5us*/
	nrf_delay_us(5);
	nrf_gpio_pin_clear(I2C_SDA);
	/*start time over 5us*/
	nrf_delay_us(4);
	/*clamp the scl*/
	nrf_gpio_pin_clear(I2C_SCL);
	nrf_delay_us(2);
}

void I2C_Stop()
{
	I2C_Pin_Configuration(1);
	nrf_gpio_pin_clear(I2C_SCL);
	nrf_gpio_pin_clear(I2C_SDA);
	nrf_delay_us(1);	
	nrf_gpio_pin_set(I2C_SCL);
	nrf_delay_us(5);
	nrf_gpio_pin_set(I2C_SDA);
	nrf_delay_us(4);
}

void I2C_Release_Bus()
{
	nrf_gpio_pin_clear(I2C_SCL);
	nrf_gpio_pin_set(I2C_SDA);
}
/*check ack from slave, set sda as input*/ 
/*and write it back*/
unsigned char I2C_CheckAck()
{
	unsigned char errtime = 255;
	nrf_gpio_pin_set(I2C_SDA);
	nrf_delay_us(3);
	nrf_gpio_pin_set(I2C_SCL);
	nrf_delay_us(5);
	I2C_Pin_Configuration(0);
	while(nrf_gpio_pin_read(I2C_SDA))
	{
		errtime--;
		if(errtime == 0)
		{
			I2C_Pin_Configuration(1);
			I2C_Stop();
			return 0;
		}
	}
	nrf_gpio_pin_clear(I2C_SCL);
	I2C_Pin_Configuration(1);
	nrf_delay_us(1);
	return 1;
}

void I2C_SendAck(char ack)
{
	if(ack)
		nrf_gpio_pin_set(I2C_SDA);
	else
		nrf_gpio_pin_clear(I2C_SDA);
	nrf_delay_us(3);
	nrf_gpio_pin_set(I2C_SCL);
	nrf_delay_us(5);
	nrf_gpio_pin_clear(I2C_SCL);	
	nrf_delay_us(2);
}

void I2C_Tx(unsigned char txbyte)
{
	signed char i;
	I2C_Pin_Configuration(1);
	for(i = 7;i >= 0;i--)
	{
		nrf_gpio_pin_clear(I2C_SCL);
		(((txbyte >> i) & 0x1) > 0) ? nrf_gpio_pin_set(I2C_SDA)\
		:nrf_gpio_pin_clear(I2C_SDA);
		nrf_delay_us(2);
		nrf_gpio_pin_set(I2C_SCL);
		nrf_delay_us(5);
		nrf_gpio_pin_clear(I2C_SCL);	
	}
	nrf_delay_us(2);
}

unsigned char I2C_Rx()
{
	unsigned char byte = 0;
	signed char i;
	I2C_Pin_Configuration(1);
	I2C_Release_Bus();
	I2C_Pin_Configuration(0);
	for(i = 7;i >= 0;i--)
	{
		nrf_delay_us(1);
		nrf_gpio_pin_clear(I2C_SCL);
		nrf_delay_us(5);
		nrf_gpio_pin_set(I2C_SCL);
		nrf_delay_us(2);
		byte = byte | (nrf_gpio_pin_read(I2C_SDA) << i);
		nrf_delay_us(2);
	}
	nrf_gpio_pin_clear(I2C_SCL);
	nrf_delay_us(2);
	return byte;
}

void HTU21D_Init()
{
	I2C_Start();
	I2C_Tx(0XFE);
	I2C_Stop();
	nrf_delay_ms(15);
}

float HTU21D_Read(char command)
{
	char ack;
	char errtime = 0;
	unsigned short result = 0;
	float temp,humi;
	I2C_Start();
	I2C_Tx(HTU21D_ADDR);
	ack = I2C_CheckAck();
	if(ack) {
		//printf("ack address\n");
	}
	else {
		I2C_Stop();
		return ERR_NO_ACK;
	}
	I2C_Tx(command);
	ack = I2C_CheckAck();
	if(ack) {
		//printf("ack cmd\n");
	}
	else {
		I2C_Stop();
		return ERR_NO_ACK;
	}
	I2C_Start();
	I2C_Tx(HTU21D_ADDR|0x01);
	ack = I2C_CheckAck();
	if(ack) {
		//printf("ack address read\n");
	}
	else {
		I2C_Stop();
		return ERR_NO_ACK;
	}
	nrf_delay_ms(50);
	result|= (I2C_Rx() << 8);
	
	I2C_SendAck(1);
	result|= I2C_Rx();
	//printf("result = %d\r\n", result);
	I2C_SendAck(0);
	I2C_Stop();
	if(command == COMMAND_QUERY_TEMPURATURE) {
		result &= 0xFFFC;
		temp = -46.85 + 175.72*result/(1 << 16);
	return temp;
	} else if (command == COMMAND_QUERY_HUMIDITY){
		result &= 0xFFF0;
	humi = -6 + 125.0*result/(1 << 16);
	return humi;
	} else {
	while(1);
	}
	
}
