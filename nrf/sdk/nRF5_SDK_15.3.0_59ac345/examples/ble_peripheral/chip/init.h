/** 
 * \file init.h
 * \brief Functions for initializing nRF hardware and helper functions
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */
 
#ifndef HWATCH_NRF_INIT_H
#define HWATCH_NRF_INIT_H

#include <stdint.h>
#include <stdbool.h>
#include "common.h"

extern void HWATCH_NRF_BLE_AdvStart();
extern void HWATCH_NRF_BLE_AdvStop();
extern void HWATCH_NRF_BLE_StartAdvTimerMsec(uint32_t time_msec);
extern void HWATCH_NRF_BLE_StopAdvTimer();
extern void HWATCH_NRF_BLE_StartNapTimerMsec(uint32_t time_msec);
extern void HWATCH_NRF_BLE_StopNapTimer();

extern void gpio_init(void);
extern void clks_init(void);
extern void uart_init(void);
extern void log_init(void);
extern void power_management_init(void);
extern void timers_init(void);
extern void ble_stack_init(void);
extern void gap_params_init(void);
extern void gatt_init(void);
extern void services_init(void);
extern void advertising_init(void);
extern void conn_params_init(void);
extern void spi_init(void const * pHSpiDev,
                     HWATCH_SPI_MODE_TYPE const spi_mode,
                     bool const blockingMode );

#endif /* HWATCH_NRF_INIT_H */
