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
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */
#include "function.h"
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
//========== include spi and clock ========================
#include "nrf_drv_clock.h"
#include "nrf_drv_spi.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "boards.h"
#include "spi_example.h"
#include "spi_master_v2.h"
#include "sdk_config.h"
//========== include sdcard =======================================================================
#include "nrf.h"
#include "bsp.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"

#define NRF_LOG_MODULE_NAME "APP"


#define FILE_NAME   "Test.TXT"
#define TEST_STRING "SD card example.\r\n"

#define SDC_SCK_PIN     8  ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN    10  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN    9  ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN      12  ///< SDC chip select (CS) pin.


NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);
//================================================================================================
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "Nordic_UART_2"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)  
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(400, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                1024                                        /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                1024                                         /**< UART RX buffer size. */

#define one               13                              /**< Is on when device is advertising. */
#define two               14                              /**< Is on when device has connected. */
#define three             16   

//========================== Lis3dh ==========================================
#define  LIS3DH_CTRL_REG1_DATARATE_100HZ      0x50
#define  LIS3DH_CTRL_REG1_XYZEN               0x07
#define  LIS3DH_REGISTER_CTRL_REG1            0x20
// ======================== timer ===========================================
#define APP_TIMER_PRESCALER             15    // Value of the RTC1 PRESCALER register.
#define APP_TIMER_OP_QUEUE_SIZE         3     // Size of timer operation queues.

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

APP_TIMER_DEF(m_led_a_timer_id);

static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING)+1];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */
static int index=0,sdaccindex=0;
//==================== ble declare ==============================================
static uint8_t ACC_read_address[6][2]={{0xA9,0xFF},{0xA8,0xFF},{0xAB,0xFF},{0xAA,0xFF},{0xAD,0xFF},{0xAC,0xFF}};
bool b_ble_tx_complette=false;

uint8_t tx_data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};  // Transmit buffer
uint8_t rx_data[4]; // Receive buffer
uint8_t accdata[20]={0},sdaccdata[36],sdaccdata1[600],sdaccdata2[600];
uint32_t* spi_address;

uint8_t CSselect=1;
//==================== SD Card Declare ============================================

char filecountpath[]={"AAA.TXT"};
char accpath[]={"xxx.TXT"};
char readText[]={"000"};

//=================== Program flag ====================================================
bool 	writeflag=false,writeflag1=false,writeflag2=false,sdwriteto1=true,sdwriteto2=false;
bool	closeflag=false;


//=========================================================================================
/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
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
void SDC_init()
{

   
    DSTATUS disk_state = STA_NOINIT;
		
		static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));
		
    NRF_LOG_INFO("Initializing disk 0 (SDC)...\r\n");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
		
    if (disk_state)
    {		
				
        return;
    }
}
void Filename_Process(bool iswrite)
{
		if( Getfile(filecountpath, readText))
		{
				nrf_drv_gpiote_out_toggle(one);
				int filecount=(readText[0]-0x30)*100 + (readText[1]-0x30)*10 + (readText[2]-0x30);
				if(iswrite)
					filecount++;
				accpath[0]=(filecount/100)+0x30;
				readText[0]=accpath[0];
				filecount=filecount%100;
				accpath[1]=(filecount/10)+0x30;
				readText[1]=accpath[1];
				filecount=filecount%10;
				accpath[2]=filecount+0x30;
				readText[2]=accpath[2];
				if(iswrite)
				{
					Writefile(filecountpath,readText,3);
					nrf_drv_gpiote_out_set(two);

				}
				
		}
		else
			nrf_gpio_pin_set(three);
}
void Read_LIS3DH_WHOAMI_Register(void)
{
	uint8_t temp[2]={0x8F,0x8F},send[2];
	spi_master_tx_rx(spi_address, 2, (const uint8_t *)temp, send,CSselect);
			send[0]=send[1] >> 4;
		send[1]=send[1] & 0x0F;	
	if( send[0] >0x09)
			send[0] = send[0] + 0x37;
		else
			send[0] = send[0] + 0x30;

		if(send[1] >0x09)
			send[1] = send[1] + 0x37;
		else
			send[1] = send[1] + 0x30;		
	ble_nus_string_send(&m_nus,send,2);

}
void Read_SPI_two()
{
	uint8_t tx_temp[6][2]={{0xA9,0xFF},{0xA8,0xFF},{0xAB,0xFF},{0xAA,0xFF},{0xAD,0xFF},{0xAC,0xFF}},rx_temp[6];
	uint8_t xx[2],send[3];
	uint8_t rxx[2];
	//nrf_delay_ms(10);
	//ble_nus_string_send(&m_nus, rx_temp, 3);
	
	for(int i=0;i<6;i++)
	{
		
		spi_master_tx_rx(spi_address, 2, (const uint8_t *)tx_temp[i], rxx,CSselect);
		send[0]=rxx[1] >> 4;
		send[1]=rxx[1] & 0x0F;	
		if( send[0] >0x09)
			send[0] = send[0] + 0x37;
		else
			send[0] = send[0] + 0x30;

		if(send[1] >0x09)
			send[1] = send[1] + 0x37;
		else
			send[1] = send[1] + 0x30;		
		ble_nus_string_send(&m_nus,send,2);
	}
	
	if(rx_temp[1]== 0x33)
	{
		xx[0]='o';
	}
	if(rx_temp[0]==0x33)
	{
		xx[0]='O';
		//ble_nus_string_send(&m_nus,xx, 1);
	}
	//send[1]=temp[0] >> 4;
	//ble_nus_string_send(&m_nus,xx,1);
}
bool turnon=true;
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
	static uint8_t xx[BLE_NUS_MAX_DATA_LEN];
	xx[0]='1';
	
	//ble_nus_string_send(p_nus, p_data, length);
//	nrf_gpio_pin_clear(two);
	//Enable_LIS3DH_Sensor();
	//Read_LIS3DH_WHOAMI_Register();
	//Read_SPI_two();
	//tx_data[0]=LIS3DH_REGISTER_CTRL_REG1;
	//Read_LIS3DH_WHOAMI_Register();
	rx_data[3]=97;rx_data[2]=97;
	
	/*spi_master_tx_rx(spi_address, 2, (const uint8_t *)tx_data, rx_data);
	nrf_delay_ms(10);
	tx_data[0]=(LIS3DH_CTRL_REG1_DATARATE_100HZ | LIS3DH_CTRL_REG1_XYZEN);
	spi_master_tx_rx(spi_address, 2, (const uint8_t *)tx_data, rx_data);	
	ble_nus_string_send(p_nus, rx_data, 4);
	nrf_delay_ms(10);
	tx_data[0]=0x0F;
//	tx_data[0]=(LIS3DH_CTRL_REG1_DATARATE_100HZ | LIS3DH_CTRL_REG1_XYZEN);
	spi_master_tx_rx(spi_address, 2, (const uint8_t *)tx_data, rx_data);
	ble_nus_string_send(p_nus, rx_data, 4);*/
	//Read_SPI_two();
	if(p_data[0]=='1')
	{				
		sdaccindex=-1;
		index=-1;
			//SDC_init();	
		nrf_drv_gpiote_out_toggle(one);
			//testSDC();
		/*	f_mount(&accfs, "", 0);
			f_open(&accfile, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND | FA_OPEN_ALWAYS);
			f_write(&accfile, TEST_STRING, sizeof(TEST_STRING) - 1, (UINT *) &accbytes_written);*/
			app_timer_start(m_led_a_timer_id, APP_TIMER_TICKS(10, APP_TIMER_PRESCALER), NULL);				
			turnon=false;		
			writeflag=false;		
			//Read_LIS3DH_WHOAMI_Register();
	}
	else if(p_data[0]=='2')
	{
			
			
			app_timer_stop(m_led_a_timer_id);
			
			turnon=true;
			closeflag=true;
	}		
						//nrf_gpio_pin_clear(two);
		
   /* for (uint32_t i = 0; i < length; i++)
    {
			
        while(app_uart_put(xx[0]) != NRF_SUCCESS);
    }*/
	
    //while(app_uart_put('\n') != NRF_SUCCESS);
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
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
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
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



/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t  err_code;
    uint8_t xx[1];
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;				
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
				case BLE_EVT_TX_COMPLETE: 
						b_ble_tx_complette = true;
						xx[0]='c';
						//xx[0]=index+0x30;
						//ble_nus_string_send(&m_nus,xx,1);
						break;
        default:
            // No implementation needed.
            break;
    }
		
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice 
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a 
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
        
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
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

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */

void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;

    uint32_t       err_code;
	
	
    switch (p_event->evt_type)
    {
			    
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                err_code = ble_nus_string_send(&m_nus, data_array, 1);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
                
                index = 0;
            }
            break;

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
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_ENABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;
		
    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;
		
    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
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


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**@brief Application main function.
 */
static void lfclk_request(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}
static void timer_a_handler(void * p_context)
{

	index++;
	sdaccindex++;
	if(index>=3)
	{
		index=0; 
		ble_nus_string_send(&m_nus,accdata, 18);	
		//writeflag=true;
		//nrf_drv_gpiote_out_set(one);
	}
	if(sdaccindex>=50)
	{
		sdaccindex=0;
		if(sdwriteto1)
		{
			writeflag1=true;
			
			sdwriteto1=false;
			sdwriteto2=true;
		}
		else if(sdwriteto2)
		{
			writeflag2=true;
			
			sdwriteto1=true;
			sdwriteto2=false;			
		}		
		
		nrf_drv_gpiote_out_toggle(three);
		
	}
	uint8_t rx_temp[6];
	uint8_t send[3];
	uint8_t rxx[2];
	
	//ble_nus_string_send(&m_nus,xx,1);
	
	for(int i=0;i<6;i++)
	{
		
		spi_master_tx_rx(spi_address, 2, (const uint8_t *)ACC_read_address[i], rxx,CSselect);
		
		send[0]=rxx[1] >> 4;
		send[1]=rxx[1] & 0x0F;	
		if( send[0] >0x09)
			send[0] = send[0] + 0x37;
		else
			send[0] = send[0] + 0x30;

		if(send[1] >0x09)
			send[1] = send[1] + 0x37;
		else
			send[1] = send[1] + 0x30;		
		
		accdata[6*index+i]=rxx[1];
		if(sdwriteto1)
		{
			sdaccdata1[12*sdaccindex+i*2]=send[0];
			sdaccdata1[12*sdaccindex+i*2+1]=send[1];
		}
		else if(sdwriteto2)
		{
			sdaccdata2[12*sdaccindex+i*2]=send[0];
			sdaccdata2[12*sdaccindex+i*2+1]=send[1];			
		}
	  	//accdata[6*index+i]=rxx[1];
		//accdata[6*index+i]=i+0x30;
		/*
		sdaccdata[12*sdaccindex+i*2]=send[0];
		sdaccdata[12*sdaccindex+i*2+1]=send[1];*/
	//	accdata[i*2+1]=i*2+1+0x30;
	}
	//accdata[6*index+5]=index+0x30;
	
	//ble_nus_string_send(&m_nus,accdata, 11);
	//ble_nus_string_send(&m_nus,xx, 1);
}
static void create_timers()
{   
    uint32_t err_code;

    // Create timers
    err_code = app_timer_create(&m_led_a_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_a_handler);
    APP_ERROR_CHECK(err_code);
}
static void fatfs_example()
{
    static FATFS fs;
    static DIR dir;
    static FILINFO fno;
    static FIL file;

    uint32_t bytes_written;
    FRESULT ff_result;
    DSTATUS disk_state = STA_NOINIT;
		
    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));
		
    NRF_LOG_INFO("Initializing disk 0 (SDC)...\r\n");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
		
    if (disk_state)
    {
							
        NRF_LOG_INFO("Disk initialization failed.\r\n");
								
        return;
    }
			
    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    NRF_LOG_INFO("Capacity: %d MB\r\n", capacity);

    NRF_LOG_INFO("Mounting volume...\r\n");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
        NRF_LOG_INFO("Mount failed.\r\n");
				
        return;
					
		
    }

    NRF_LOG_INFO("\r\n Listing directory: /\r\n");
    ff_result = f_opendir(&dir, "/");
    if (ff_result)
    {
        NRF_LOG_INFO("Directory listing failed!\r\n");
			
        return;
    }
    
    do
    {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            NRF_LOG_INFO("Directory read failed.");
				
            return;
        }
        
        if (fno.fname[0])
        {
            if (fno.fattrib & AM_DIR)
            {
                NRF_LOG_RAW_INFO("   <DIR>   %s\r\n",(uint32_t)fno.fname);
            }
            else
            {
                NRF_LOG_RAW_INFO("%9lu  %s\r\n", fno.fsize, (uint32_t)fno.fname);
            }
        }
    }
    while (fno.fname[0]);
    NRF_LOG_RAW_INFO("\r\n");
    
    NRF_LOG_INFO("Writing to file " FILE_NAME "...\r\n");
    ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
			
        NRF_LOG_INFO("Unable to open or create file: " FILE_NAME ".\r\n");
        return;
    }

    ff_result = f_write(&file, TEST_STRING, sizeof(TEST_STRING) - 1, (UINT *) &bytes_written);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Write failed\r\n.");
    }
    else
    {
        NRF_LOG_INFO("%d bytes written.\r\n", bytes_written);
    }

    (void) f_close(&file);
		
    return;
}
int main(void)
{

    uint32_t err_code;
    bool erase_bonds;
		uint8_t success_string[5];
		success_string[0]=65;
		nrf_gpio_cfg_output(one);
		nrf_gpio_cfg_output(two);
		nrf_gpio_cfg_output(three);
	
	/*	nrf_gpio_pin_clear(one);
		nrf_gpio_pin_clear(two);
		nrf_gpio_pin_clear(three);*/
		nrf_gpio_pin_set(one);
		nrf_gpio_pin_set(two);
		nrf_gpio_pin_set(three);
	
		lfclk_request();
			//================ timer Init =======================================	
		APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);	
		//fatfs_example();
		
		//testSDC();

		//writeSDC_test();	
		

    // Initialize.
		SDC_init();
    //uart_init();		
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();				
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
			


		//================ SPI Init =======================================
	  spi_address=spi_master_init(SPI1,0,false);
		nrf_gpio_cfg_output(0); //cs G
		nrf_gpio_cfg_output(1); //cs X
		nrf_gpio_pin_set(0);
		nrf_gpio_pin_set(1);
		uint8_t tx_temp[2]={0xA8,0xA8},rx_temp[2]={0x00,0x00};
		uint8_t xx[2],send[3];	
		uint8_t CTRL_REG1_DATA[2]={0x20,0x67};//0b0110 0111 0x67
		/*spi_master_tx_rx(spi_address, 2, (const uint8_t *)CTRL_REG1_DATA, rx_temp,CSselect);

		CTRL_REG1_DATA[0]=0x23; //
		CTRL_REG1_DATA[1]=0x38; //0b0011 1000 0x38
		spi_master_tx_rx(spi_address, 2, (const uint8_t *)CTRL_REG1_DATA, rx_temp,CSselect);		*/  //LIS3DH Accelerometer
    // Enter main loop.
		
		CTRL_REG1_DATA[0]=0x20; //
		CTRL_REG1_DATA[1]=0x77; //0b0011 1000 0x38
		spi_master_tx_rx(spi_address, 2, (const uint8_t *)CTRL_REG1_DATA, rx_temp,CSselect);		
		CTRL_REG1_DATA[0]=0x26; //
		CTRL_REG1_DATA[1]=0x00; //0b0011 1000 0x38
		spi_master_tx_rx(spi_address, 2, (const uint8_t *)CTRL_REG1_DATA, rx_temp,CSselect);		
		CTRL_REG1_DATA[0]=0x24; //
		CTRL_REG1_DATA[1]=0xF0; //0b0011 1000 0x38
		spi_master_tx_rx(spi_address, 2, (const uint8_t *)CTRL_REG1_DATA, rx_temp,CSselect);		//LIS9DH Accelerometer */
		
		
	  /*
		CTRL_REG1_DATA[0]=0x20;
		CTRL_REG1_DATA[1]=0x0F; //0b0011 1000 0x38
		spi_master_tx_rx(spi_address, 2, (const uint8_t *)CTRL_REG1_DATA, rx_temp,CSselect);		
		CTRL_REG1_DATA[0]=0x23; //
		CTRL_REG1_DATA[1]=0x00; //0b0011 1000 0x38
		spi_master_tx_rx(spi_address, 2, (const uint8_t *)CTRL_REG1_DATA, rx_temp,CSselect);	//*/
		
		/*CTRL_REG1_DATA[2]={0x21,0xA0}; 
		spi_master_tx_rx(spi_address, 2, (const uint8_t *)CTRL_REG1_DATA, rx_temp,CSselect);*/		
		SDCtest();
		//testSDC();
		create_timers();

		nrf_gpio_pin_clear(one);
		nrf_gpio_pin_clear(two);
		nrf_gpio_pin_clear(three);
		Filename_Process(false);
		
		FATFS accfs;
		FIL accfile;
		uint32_t accbytes_written;
		FRESULT accff_result;

		
    for (;;)
    {
        power_manage();
				if(!turnon)
				{
					//testSDC();

					turnon=true;
				}
/*			if(writeflag)
				{
					writeflag=false;
					//f_write(&accfile, TEST_STRING, 36, (UINT *) &accbytes_written);
					accff_result = f_mount(&accfs, "", 0);
					accff_result = f_open(&accfile, accpath, FA_READ | FA_WRITE | FA_OPEN_ALWAYS | FA_OPEN_APPEND);
					//accff_result = f_write(&accfile, TEST_STRING, sizeof(TEST_STRING) - 1, (UINT *) &accbytes_written);
					f_write(&accfile, sdaccdata, 36,  &accbytes_written);
					(void) f_close(&accfile);						
					//nrf_drv_gpiote_out_toggle(three);
					
				}//	*/		
				if(writeflag1)
				{
					writeflag1=false;
					//f_write(&accfile, TEST_STRING, 36, (UINT *) &accbytes_written);
					accff_result = f_mount(&accfs, "", 0);
					accff_result = f_open(&accfile, accpath, FA_READ | FA_WRITE | FA_OPEN_ALWAYS | FA_OPEN_APPEND);
					//accff_result = f_write(&accfile, TEST_STRING, sizeof(TEST_STRING) - 1, (UINT *) &accbytes_written);
					f_write(&accfile, sdaccdata1, 600,  &accbytes_written);
					(void) f_close(&accfile);		
					nrf_drv_gpiote_out_set(one);					
					nrf_drv_gpiote_out_clear(two);
				}
				if(writeflag2)
				{
					writeflag2=false;
					//f_write(&accfile, TEST_STRING, 36, (UINT *) &accbytes_written);
					accff_result = f_mount(&accfs, "", 0);
					accff_result = f_open(&accfile, accpath, FA_READ | FA_WRITE | FA_OPEN_ALWAYS | FA_OPEN_APPEND);
					//accff_result = f_write(&accfile, TEST_STRING, sizeof(TEST_STRING) - 1, (UINT *) &accbytes_written);
					f_write(&accfile, sdaccdata2, 600,  &accbytes_written);
					(void) f_close(&accfile);			
					nrf_drv_gpiote_out_set(two);					
					nrf_drv_gpiote_out_clear(one);					
				}
				if(closeflag)
				{					
					closeflag=false;
					Filename_Process(true);					
				}
    }
}


/** 
 * @}
 */
