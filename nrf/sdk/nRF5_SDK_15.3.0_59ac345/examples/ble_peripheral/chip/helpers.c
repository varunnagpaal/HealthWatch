/** 
 * \file helpers.c
 * \brief Helper Functions for nRF Module
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */
 
#include "helpers.h"
#include "ble_types.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "common_nrf.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"

uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */

/*******************************HELPER FUNCTIONS******************************/

/**@brief Function for putting the chip into deep sleep (System OFF) mode.
 *
 * @note This function will not return.
 */
void HWATCH_NRF_BLE_EnterDeepSleepMode(void)
{
  /* Blocking function call */
  if (false == NRF_LOG_PROCESS()) // required to process buffer if NRF_LOG_DEFERRED is set to 1
  {
    // Goto deep sleep (SYSTEM OFF mode)
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
  }
}

/**@brief Function for putting the chip into nap (Low Power) mode.
 *
 * @note This function will not return.
 */
void HWATCH_NRF_BLE_EnterNapMode(bool volatile *pbWakeupFlag)
{
  while(false == *pbWakeupFlag)
  {
    /* This function for nap sleep is non-blocking
       unlike nrf_pwr_mgmt_shutdown function call
       which is a blocking function call that puts
       nrf into deep sleep mode */
    nrf_pwr_mgmt_run();
  }
}

/**@brief Function for asserting ACK to ADI.
 *
 * @note This function will not return.
 */
void HWATCH_NRF_BLE_AssertAckToAdi(void)
{
  /* ACK is an active low signal */
  nrf_drv_gpiote_out_clear( HWATCH_NRF_BLE_ACK_PIN );
}

/**@brief Function for de-asserting ACK to ADI.
 *
 * @note This function will not return.
 */
void HWATCH_NRF_BLE_DeAssertAckToAdi(void)
{
  /* ACK is an active low signal */
  nrf_drv_gpiote_out_set( HWATCH_NRF_BLE_ACK_PIN );
}

/**@brief Function for enabling Interrupt on a GPIO pin
 *
 * @note This function will not return.
 */
void HWATCH_NRF_BLE_EnableGPIOInt( const nrfx_gpiote_pin_t pin )
{
  nrf_drv_gpiote_in_event_enable(pin,true);
}

/**@brief Function for disabling Interrupt on a GPIO pin
 *
 * @note This function will not return.
 */
void HWATCH_NRF_BLE_DisableGPIOInt( const nrfx_gpiote_pin_t pin )
{
  nrf_drv_gpiote_in_event_enable(pin,false);
}

/** @brief Function for starting the internal 32 KHz LFCLK XTAL oscillator.
           that is required for driving RTC.
 */
void HWATCH_NRF_BLE_Req_LFClk(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

char const* HWATCH_State_To_Str(HWATCH_NRF_STATE_TYPE const state)
{
  switch(state)
  {
    case HWATCH_NRF_STATE_POWER_ON_RST:           return "HWATCH_NRF_STATE_POWER_ON_RST";
    case HWATCH_NRF_STATE_DEEP_SLEEP:             return "HWATCH_NRF_STATE_DEEP_SLEEP";
    case HWATCH_NRF_STATE_HW_INIT:                return "HWATCH_NRF_STATE_HW_INIT";
    case HWATCH_NRF_STATE_NAP_SLEEP:              return "HWATCH_NRF_STATE_NAP_SLEEP";
    case HWATCH_NRF_STATE_CONFIRM_WAKEUP_TO_ADI:  return "HWATCH_NRF_STATE_CONFIRM_WAKEUP_TO_ADI";
    case HWATCH_NRF_STATE_PAIR:                   return "HWATCH_NRF_STATE_PAIR";
    case HWATCH_NRF_STATE_WAIT_FOR_USER_DETAILS:  return "HWATCH_NRF_STATE_WAIT_FOR_USER_DETAILS";
    case HWATCH_NRF_STATE_WAIT:                   return "HWATCH_NRF_STATE_WAIT";
    case HWATCH_NRF_STATE_STORING_SENSOR_DATA:    return "HWATCH_NRF_STATE_STORING_SENSOR_DATA";
    case HWATCH_NRF_STATE_STREAM_SENSOR_DATA:     return "HWATCH_NRF_STATE_STREAM_SENSOR_DATA";
    default:                                    return "HWATCH_NRF_STATE_UNKNOWN";
  }
}

/**@brief Function for delay execution in ms.
 *
 * @note This function will not return.
 */
void HWATCH_NRF_Delay(const uint32_t time_delay_ms)
{
  nrf_delay_ms(time_delay_ms);
}
