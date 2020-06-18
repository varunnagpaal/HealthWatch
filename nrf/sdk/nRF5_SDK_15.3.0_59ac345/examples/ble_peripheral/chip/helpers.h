/** 
 * \file helpers.h
 * \brief Helper functions for nRF Module
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */
 
#ifndef HWATCH_NRF_HELPERS_H
#define HWATCH_NRF_HELPERS_H

#include <stdint.h>
#include "nrf_drv_gpiote.h"
#include "common_nrf.h"

extern uint16_t   m_conn_handle;                  /**< Handle of the current connection. */

extern void HWATCH_NRF_BLE_EnterDeepSleepMode(void);
extern void HWATCH_NRF_BLE_EnterNapMode(bool volatile *pbWakeupFlag);
extern void HWATCH_NRF_BLE_AssertAckToAdi(void);
extern void HWATCH_NRF_BLE_DeAssertAckToAdi(void);
extern void HWATCH_NRF_BLE_EnableGPIOInt( const nrfx_gpiote_pin_t pin );
extern void HWATCH_NRF_BLE_DisableGPIOInt( const nrfx_gpiote_pin_t pin );
extern void HWATCH_NRF_BLE_Req_LFClk(void);
extern char const* HWATCH_State_To_Str(HWATCH_NRF_STATE_TYPE const state);
extern void HWATCH_NRF_Delay(const uint32_t time_delay_ms);

#endif /* HWATCH_NRF_HELPERS_H */
