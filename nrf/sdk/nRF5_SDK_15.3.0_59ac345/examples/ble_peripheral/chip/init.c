/** 
 * \file init.c
 * \brief Hardware Initialization and Helper functions for nRF module
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */
 
#include "init.h"
#include "nordic_common.h"
#include "nrf.h"
#include <string.h>
#include "sdk_errors.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_error.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "app_timer.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "common_nrf.h"
#include "helpers.h"
#include "nrf_ble_qwr.h"
#include "nrf_ble_gatt.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "nrf_delay.h"
#include "ble_advertising.h"
#include "ble_advdata.h"
#include "ble_hci.h"
#include "nrf_sdh_soc.h"
#include "ble_conn_params.h"
#include "nrf_uart.h"
#include "nrf_drv_gpiote.h"
#include <assert.h>
#include "nrfx_spim.h"
#include "nrfx_spis.h"

uint8_t order = 1;

volatile bool bAdiWokeFromDeepSleep = false;
volatile bool bAdiWokeFromNapSleep = false;
volatile bool bNapSleepTimedOutFlag = false;
volatile bool bAdvTimedOutFlag = false;
volatile bool bIpaired = false;
volatile bool bSpiMasterTxRxDone = false;
volatile bool bSpiSlaveTxRxDone = false;
nrfx_spim_t const hSpiMasterDev0 = NRFX_SPIM_INSTANCE(SPI_DEV_ID_0);
nrfx_spis_t const hSpiSlaveDev0 = NRFX_SPIS_INSTANCE(SPI_DEV_ID_1);

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
APP_TIMER_DEF(m_adv_timer_id);
APP_TIMER_DEF(m_nap_timer_id);

static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

// HACK. TBU
int i;
int sampling_period_ms = 5000;
bool useridAvail = false;
bool waitingForUserId = false;
bool promptAgain = true;

const int freq_seed = 63;
const int temp_seed = 98;
const int rh_seed = 49;

char userId[11];
//const char* timeId = "2019-10-05:14:20:01";
int ch = 1;
const char* deviceId = "dev0001";
int freq = 63;
const char* presetCode = "pc";
int rh = 49;
int temp = 97;
int timeStamp = 5;
float zmod = 1000000.5;
float zphase = 0.6;

/*******************************HELPER FUNCTIONS*******************************/
void HWATCH_NRF_BLE_AdvStart()
{
  ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
  APP_ERROR_CHECK(err_code);
  NRF_LOG_INFO("[%d]init.c::advertising_start", order++);
}

void HWATCH_NRF_BLE_AdvStop()
{
  ret_code_t err_code = sd_ble_gap_adv_stop( m_advertising.adv_handle );
  APP_ERROR_CHECK(err_code);
}

void HWATCH_NRF_BLE_StartAdvTimerMsec(uint32_t time_msec)
{
  ret_code_t err_code = app_timer_start(m_adv_timer_id,
                                        APP_TIMER_TICKS(time_msec),
                                        NULL);
  APP_ERROR_CHECK(err_code);
}

void HWATCH_NRF_BLE_StopAdvTimer()
{
  ret_code_t err_code;

  err_code = app_timer_stop(m_adv_timer_id);
  APP_ERROR_CHECK(err_code);
}

void HWATCH_NRF_BLE_StartNapTimerMsec(uint32_t time_msec)
{
  ret_code_t err_code = app_timer_start(m_nap_timer_id,
                                        APP_TIMER_TICKS(time_msec),
                                        NULL);
  APP_ERROR_CHECK(err_code);
}

void HWATCH_NRF_BLE_StopNapTimer()
{
  ret_code_t err_code;

  err_code = app_timer_stop(m_nap_timer_id);
  APP_ERROR_CHECK(err_code);
}

/***************************EVENT HANDLERS*************************************/
/**@brief Function for handling wakeup signal events from ADI.
 *
 * @param[in]   pin     pin on which wakeup signal is received
 * @param[in]   action  polarity detected on the pin
 */
void Isr_Ble_Wakeup_Handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  switch(action)
  {
    case NRF_GPIOTE_POLARITY_HITOLO:
      if( false == nrf_drv_gpiote_in_is_set(pin) ) bAdiWokeFromDeepSleep = true;
      break;

    case NRF_GPIOTE_POLARITY_LOTOHI:
    case NRF_GPIOTE_POLARITY_TOGGLE:
    default:
      /* Flag error. TBU */
      break;
  }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            /* Stop advertising timer */
            HWATCH_NRF_BLE_StopAdvTimer();

            /* Send ACK to ADI (HI to LO) */
            bIpaired = true;
            HWATCH_NRF_BLE_AssertAckToAdi();
            HWATCH_NRF_BLE_DeAssertAckToAdi();

            NRF_LOG_INFO("[%d]init.c::ble_evt_handler: evt_type=BLE_GAP_EVT_CONNECTED", order++);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("[%d]init.c::ble_evt_handler: evt_type=BLE_GAP_EVT_DISCONNECTED", order++);
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            bIpaired = false;
            break;

      case BLE_GAP_EVT_TIMEOUT:
            NRF_LOG_INFO("[%d]init.c::ble_evt_handler: evt_type=BLE_GAP_EVT_TIMEOUT", order++);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_INFO("[%d]init.c::ble_evt_handler: evt_type=BLE_GAP_EVT_PHY_UPDATE_REQUEST", order++);
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
          NRF_LOG_INFO("[%d]init.c::ble_evt_handler: evt_type=BLE_GAP_EVT_SEC_PARAMS_REQUEST", order++);
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
          NRF_LOG_INFO("[%d]init.c::ble_evt_handler: evt_type=BLE_GATTS_EVT_SYS_ATTR_MISSING", order++);
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_INFO("[%d]init.c::ble_evt_handler: evt_type=BLE_GATTC_EVT_TIMEOUT", order++);
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_INFO("[%d]init.c::ble_evt_handler: evt_type=BLE_GATTS_EVT_TIMEOUT", order++);
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling events from the GATT library. */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
//    uint8_t rx_byte;
    NRF_LOG_INFO("[%d]init.c::nus_data_handler", order++);
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        NRF_LOG_INFO("[%d]init.c::nus_data_handler: evt=BLE_NUS_EVT_RX_DATA", order++);
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);

//            nrf_delay_ms(10);
//
//            err_code = app_uart_get(&rx_byte);
//            if( ( rx_byte != p_evt->params.rx_data.p_data[i] ) && err_code != NRF_SUCCESS )
//            {
//              NRF_LOG_ERROR("Failed to Loopback character 0x%x. Error: 0x%x",
//                            p_evt->params.rx_data.p_data[i],
//                            err_code );
//            }
        }

        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);

//            nrf_delay_ms(10);
//
//            err_code = app_uart_get(&rx_byte);
//
//            if( ( rx_byte != p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] ) && err_code != NRF_SUCCESS )
//            {
//              NRF_LOG_ERROR("Failed to Loopback character 0x%x. Error: 0x%x",
//                            p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1],
//                            err_code );
//            }
        }
    }

    NRF_LOG_DEBUG("Received string %s", p_evt->params.rx_data.p_data);
    NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
    uint32_t err_code;
    char data_buffer[BLE_NUS_MAX_DATA_LEN];

    if( true == waitingForUserId )
    {
      // If Data Received has prefix "userid" of format useridxxxx, then start sending data in a loop
      char const * expected_prefix_str = "userid";
      if( p_evt->params.rx_data.length > 0)
      {
        if( 0 == strncmp( (char*)(p_evt->params.rx_data.p_data), expected_prefix_str, strlen(expected_prefix_str) ) )
        {
          useridAvail = true;
          waitingForUserId  = false;
          promptAgain = false;
          strncpy(userId, (char*)(p_evt->params.rx_data.p_data), strlen("useridxxxx"));
        }
        else
        {
          // please re-enter user id
          useridAvail = false;
          waitingForUserId  = true;
          promptAgain = true;
        }
      }
    }

    if( true == promptAgain )
    {
      if( p_evt->params.rx_data.length > 0)
      {
        promptAgain = false;
        sprintf(data_buffer, "\nEnter user id in format useridxxxx: " );
        uint16_t length = strlen(data_buffer);
        do
        {
          err_code = ble_nus_data_send(&m_nus, (uint8_t*)data_buffer, &length, m_conn_handle);
          if ((err_code != NRF_ERROR_INVALID_STATE) &&
              (err_code != NRF_ERROR_RESOURCES) &&
              (err_code != NRF_ERROR_NOT_FOUND))
          {
              APP_ERROR_CHECK(err_code);
          }
        } while (err_code == NRF_ERROR_RESOURCES);
      }
    }

    if( false == useridAvail && (false == waitingForUserId) )
    {
      sprintf(data_buffer, "\nEnter user id in format useridxxxx: " );
      uint16_t length = strlen(data_buffer);
      do
      {
        err_code = ble_nus_data_send(&m_nus, (uint8_t*)data_buffer, &length, m_conn_handle);
        if ((err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }
      } while (err_code == NRF_ERROR_RESOURCES);

      waitingForUserId = true;
    }

    if(true == useridAvail && false == waitingForUserId)
    {
      NRF_LOG_DEBUG("Ready to send sensor data over BLE NUS");

     //char * data = "24, 0.000, 0.000, 1, D1\n";

      // srand (time(NULL));
      // float x;
      if( i < 100000)
      {
        i++;
       // x = ((float)rand()/(float)(RAND_MAX)) * 0.9999;
       // sprintf(data_buffer, "24,%.2f,%d,D1\n",x, i+1);

        if( freq < (freq_seed-7) || freq > (freq_seed+6)){ freq = freq_seed; }
        else { freq = (i%3)?(int)(freq - freq%(freq-3)):(int)(freq + freq%(freq-3)); }

        if( temp < (temp_seed-1) || temp > (temp_seed+1) ){ temp = temp_seed; }
        else { temp = (i%3)?(temp - (int)temp%(int)(temp-1)):(temp + (int)temp%(int)(temp-1)); }

        if( rh < (rh_seed-5) || rh > (rh_seed+4) ){ rh = rh_seed; }
        else { rh = (i%3)?(int)(rh - rh%(rh-2)):(int)(rh + rh%(rh-2)); }

        sprintf(data_buffer, "\n%s,%d,%s,%d,%s,%d,%d,%d,%f,%f",
               userId,
               ch,
               deviceId,
               freq,
               presetCode,
               rh,
               temp,
               timeStamp,
               zmod,
               zphase );
        uint16_t length = strlen(data_buffer);
        NRF_LOG_DEBUG("Sending over BLE. Data length: %d, Data: %s", length, data_buffer);
        do
        {
          err_code = ble_nus_data_send(&m_nus, (uint8_t*)data_buffer, &length, m_conn_handle);
          if ((err_code != NRF_ERROR_INVALID_STATE) &&
              (err_code != NRF_ERROR_RESOURCES) &&
              (err_code != NRF_ERROR_NOT_FOUND))
          {
              APP_ERROR_CHECK(err_code);
          }
        } while (err_code == NRF_ERROR_RESOURCES);
        nrf_delay_ms(sampling_period_ms);
      }
    }
//    else
//    {
//      NRF_LOG_DEBUG("Not received required string!");
//    }
}
/**@snippet [Handling the data received over BLE] */

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    // uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
          // err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
          // APP_ERROR_CHECK(err_code);
          NRF_LOG_INFO("[%d]init.c::on_adv_evt: evt_type=BLE_ADV_EVT_FAST", order++);
          break;

        case BLE_ADV_EVT_SLOW:
          NRF_LOG_INFO("[%d]init.c::on_adv_evt: evt_type=BLE_ADV_EVT_SLOW", order++);
          break;

        case BLE_ADV_EVT_IDLE:
        /* Note: we can only nap here but must not deep sleep here
                 as it causes reset. Instead we want to nap, periodically
                 wakeup and advertise, and if not connected go back to nap
        */
          NRF_LOG_WARNING("[%d]on_adv_evt: evt_type=BLE_ADV_EVT_IDLE", order++);
          NRF_LOG_WARNING("[%d]on_adv_evt: Advertisement timed out as no client attempted to respond to advertisement from BLE.", order++);
//          bNapSleepTimedOutFlag = false;
//          HWATCH_NRF_BLE_EnterNapMode(&bNapSleepTimedOutFlag);
          break;

        default:
          break;
    }
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
      NRF_LOG_INFO("[%d]init.c::on_conn_params_evt: evt_type=BLE_CONN_PARAMS_EVT_FAILED", order++);
      err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
      APP_ERROR_CHECK(err_code);
    }
    else
    {
      NRF_LOG_INFO("[%d]init.c::on_conn_params_evt: evt_type=BLE_CONN_PARAMS_EVT_SUCCEEDED", order++);
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

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
static void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            NRF_LOG_INFO("[%d]init.c::main::uart_event_handle: evt_type=APP_UART_DATA_READY, data_array[%d]:%c ", order++, index, data_array[index]);
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            NRF_LOG_INFO("[%d]init.c::main::uart_event_handle: evt_type=APP_UART_COMMUNICATION_ERROR", order++);
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            NRF_LOG_INFO("[%d]init.c::main::uart_event_handle: evt_type=APP_UART_FIFO_ERROR", order++);
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        case APP_UART_TX_EMPTY:
          NRF_LOG_INFO("[%d]init.c::main::uart_event_handle: evt_type=APP_UART_TX_EMPTY", order++ );
          break;

        case APP_UART_DATA:
          NRF_LOG_INFO("[%d]init.c::main::uart_event_handle: evt_type=APP_UART_DATA", order++ );
          break;

        default:
            break;
    }
}

/**@brief   Function for handling advertisement timeout event.
 *
 */
/**@snippet [Handling the advertisement time out event */

static void Isr_Adv_Timeout_Handler( void * p_context )
{
  UNUSED_PARAMETER(p_context);

  assert( false == bAdvTimedOutFlag);
  bAdvTimedOutFlag = true;
}

/**@brief   Function for handling nap timeout event.
 *
 */
/**@snippet [Handling the nap time out event */
static void Isr_Nap_Timeout_Handler( void * p_context )
{
  UNUSED_PARAMETER(p_context);

  assert( false == bNapSleepTimedOutFlag);

  bNapSleepTimedOutFlag = true;
}

/**
 * @brief SPI master event handler.
 * @param event
 */
static void spi_master_event_handler(nrfx_spim_evt_t const * p_event,
                                     void *                  p_context)
{
  if( p_event->type ==  NRFX_SPIM_EVENT_DONE)
  {    
    bSpiMasterTxRxDone = true;
    NRF_LOG_INFO("Transfer completed.");
//    if (m_rx_buf[0] != 0)
//    {
//        NRF_LOG_INFO(" Received:");
//        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
//    }
  }
}

/**
 * @brief SPI Slave event handler.
 *
 * @param event
 */
static void spi_slave_event_handler(nrfx_spis_evt_t const * p_event,
                                    void *                  p_context)
{
    if (p_event->evt_type == NRFX_SPIS_XFER_DONE)
    {
        bSpiSlaveTxRxDone = true;
//        NRF_LOG_INFO(" Transfer completed. Received: %s",(uint32_t)m_rx_buf);
    }
}

/*********************************INIT FUNCTIONS******************************/
void gpio_init(void)
{
  ret_code_t err_code;

  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);

  /* Configure input interrupt on GPIO SPI MOSI pin (12/P0.13)
     that wakes up the BLE from Deep sleep(System OFF mode)
     when wakeup signal(active low) is received from ADI
  */
//  nrf_drv_gpiote_in_config_t gpio_wakeup_in_pin_config = {.sense = NRF_GPIOTE_POLARITY_HITOLO,
//                                                          .pull = NRF_GPIO_PIN_PULLUP,
//                                                          .is_watcher = false,
//                                                          .hi_accuracy = false };
//
//  err_code = nrf_drv_gpiote_in_init(HWATCH_NRF_BLE_WAKEUP_PIN,
//                                    &gpio_wakeup_in_pin_config,
//                                    Isr_Ble_Wakeup_Handler);
//  APP_ERROR_CHECK(err_code);

  /* Configure input interrupt on GPIO SPI MOSI pin (12/P0.13)
     that wakes up the BLE from Deep sleep(System OFF mode)
     to sense DETECT signal(active low wake up signal) from ADI */
  nrf_gpio_cfg_sense_input( HWATCH_NRF_BLE_WAKEUP_PIN,
                            NRF_GPIO_PIN_PULLUP,
                            NRF_GPIO_PIN_SENSE_LOW);

  /* Configure GPIO SPI MISO pin (14/P0.14) as output to send ACK signal (active low)
     to ADI to confirm that if I have woken up
  */
  nrf_drv_gpiote_out_config_t gpio_ack_out_pin_config = { .init_state = NRF_GPIOTE_INITIAL_VALUE_HIGH,
                                                          .task_pin = false };
  err_code = nrf_drv_gpiote_out_init(HWATCH_NRF_BLE_ACK_PIN,
                                     &gpio_ack_out_pin_config);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_INFO("[%d]init.c::gpio_init", order++);
}

void clks_init(void)
{
  // LFCLK XTAL clock (32 KHz) is required to operate rtc (24-bit resolution, ~30 us)
  // LFCLK must be requested explicitly whenever softdevice is absent or inactive
  HWATCH_NRF_BLE_Req_LFClk();
  NRF_LOG_INFO("[%d]init.c::clks_init", order++);
}

/**@snippet [UART Initialization] */
void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = HWATCH_NRF_BLE_UART_RX_PIN,
        .tx_pin_no    = HWATCH_NRF_BLE_UART_TX_PIN,
        .rts_pin_no   = HWATCH_NRF_BLE_UART_RTS_PIN,
        .cts_pin_no   = HWATCH_NRF_BLE_UART_CTS_PIN,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       HWATCH_NRF_BLE_UART_RX_BUF_SIZE,
                       HWATCH_NRF_BLE_UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("[%d]init.c::uart_init", order++);
}
/**@snippet [UART Initialization] */

/**@brief Function for initializing the nrf log module.
 */
void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("[%d]init.c::log_init", order++);
}

/**@brief Function for initializing power management.
 */
void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("[%d]init.c::power_management_init", order++);
}

/**@brief Function for initializing the timer module.
 */
void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    /* Create advertising timer */
    err_code = app_timer_create(&m_adv_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                Isr_Adv_Timeout_Handler);
    APP_ERROR_CHECK(err_code);

    /* Create nap(low power sleep i.e.  SYSTEM ON but idle) timer */
    err_code = app_timer_create(&m_nap_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                Isr_Nap_Timeout_Handler);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("[%d]init.c::timers_init", order++);
}

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

    NRF_LOG_INFO("[%d]init.c::ble_stack_init", order++);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
void gap_params_init(void)
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

    NRF_LOG_INFO("[%d]init.c::gap_params_init", order++);
}

/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("[%d]init.c::gatt_init", order++);
}

/**@brief Function for initializing services that will be used by the application.
 */
void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("[%d]init.c::services_init", order++);
}

/**@brief Function for initializing the Advertising functionality.
 */
void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

    NRF_LOG_INFO("[%d]init.c::advertising_init", order++);
}

/**@brief Function for initializing the Connection Parameters module.
 */
void conn_params_init(void)
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

    NRF_LOG_INFO("[%d]init.c::conn_params_init", order++);
}

/**@brief Function for initializing SPI controller in master/slave mode
 */
void spi_init(void const * pHSpiDev,
              HWATCH_SPI_MODE_TYPE const spi_mode,
              bool const blockingMode )
{
    ret_code_t err_code;
    
    if(HWATCH_SPI_MODE_MASTER == spi_mode)
    {
      assert( pHSpiDev != NULL );
      nrfx_spim_config_t spi_master_config = NRFX_SPIM_DEFAULT_CONFIG;
      spi_master_config.sck_pin  = HWATCH_NRF_BLE_SPI_SCK_PIN;
      spi_master_config.mosi_pin = HWATCH_NRF_BLE_SPI_MOSI_PIN;
      spi_master_config.miso_pin = HWATCH_NRF_BLE_SPI_MISO_PIN;
      spi_master_config.ss_pin   = HWATCH_NRF_BLE_SPI_CS_PIN;
      spi_master_config.ss_active_high = HWATCH_NRF_BLE_SPI_CS_ACTIVE_LOW;
      spi_master_config.orc = HWATCH_READER_MSG_DUMMY;      
      spi_master_config.frequency = HWATCH_SPI_BIT_RATE;
      spi_master_config.bit_order = HWATCH_SPI_MASTER_BIT_ORDER;
      spi_master_config.mode =  HWATCH_NRF_BLE_SPI_MASTER_MODE;

      err_code = nrfx_spim_init((nrfx_spim_t const*)pHSpiDev,
                                &spi_master_config,
                                ( blockingMode ? (NULL):(spi_master_event_handler)),
                                NULL);
      APP_ERROR_CHECK(err_code);
    }
    else if(HWATCH_SPI_MODE_SLAVE == spi_mode)
    {
      assert( pHSpiDev != NULL );
      nrfx_spis_config_t spi_slave_config = NRFX_SPIS_DEFAULT_CONFIG;
      spi_slave_config.miso_pin = HWATCH_NRF_BLE_SPI_MISO_PIN;
      spi_slave_config.mosi_pin = HWATCH_NRF_BLE_SPI_MOSI_PIN;
      spi_slave_config.sck_pin  = HWATCH_NRF_BLE_SPI_SCK_PIN;
      spi_slave_config.csn_pin   = HWATCH_NRF_BLE_SPI_CS_PIN;
      spi_slave_config.mode =  HWATCH_NRF_BLE_SPI_SLAVE_MODE;
      spi_slave_config.bit_order = HWATCH_SPI_SLAVE_BIT_ORDER;
      spi_slave_config.def = HWATCH_READER_MSG_DUMMY;
      spi_slave_config.orc = HWATCH_READER_MSG_DUMMY;

      err_code = nrfx_spis_init((nrfx_spis_t const*)pHSpiDev,
                                &spi_slave_config,
                                ( blockingMode ? (NULL):(spi_slave_event_handler)),
                                NULL);
      APP_ERROR_CHECK(err_code);
    }
    else
    {
      // Default SPI on BLE is slave
    }

    NRF_LOG_INFO("[%d]init.c::spi_init", order++);
}