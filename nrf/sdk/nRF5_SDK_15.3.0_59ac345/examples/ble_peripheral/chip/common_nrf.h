/** 
 * \file common_nrf.h
 * \brief Common macros, typedefs, enums for nRF module
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */
 
#ifndef HWATCH_NRF_COMMON_H
#define HWATCH_NRF_COMMON_H

#include <stdint.h>
#include "nrfx_gpiote.h"

extern uint8_t order;

/* Set this to 1 to debug */
#define HWATCH_NRF_DEBUG 0

/* Set this to 1 to run prototype code */
#define HWATCH_NRF_PROTO

#define HWATCH_NRF_DEVICE_NAME            "EnLiSense"

#define APP_BLE_CONN_CFG_TAG            1
#define APP_BLE_OBSERVER_PRIO           3
#define DEVICE_NAME                     HWATCH_NRF_DEVICE_NAME                        /**< Name of device. Will be included in the advertising data. */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define APP_ADV_INTERVAL                640                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 400 ms). */
#define APP_ADV_DURATION                5000                                        /**< The advertising duration (50 seconds) in units of 10 milliseconds. */

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define HWATCH_NRF_BLE_ADV_NAP_TIMEOUT    60000                                       /** Total advertise + nap time(sec) **/
#define HWATCH_NRF_BLE_ADV_TIMEOUT        15000                                       /** advertise time (sec) */
#define HWATCH_NRF_BLE_NAP_TIMEOUT        5000                                        /** nap time (sec) upon advertisement timeout */

#define HWATCH_NRF_BLE_AD_WAIT_TIMEOUT    1000                                        /** Wait for 100 ms before sending the ACK signal to ADI */
#define HWATCH_NRF_BLE_AD_WAIT_POR        3000                                        /** Wait for 3000 msec for ADI to go to sleep on POR **/

#define HWATCH_NRF_BLE_UART_RX_PIN  8
#define HWATCH_NRF_BLE_UART_TX_PIN  6
#define HWATCH_NRF_BLE_UART_CTS_PIN 7
#define HWATCH_NRF_BLE_UART_RTS_PIN 5

#define HWATCH_NRF_BLE_UART_TX_BUF_SIZE                512                                         /**< UART TX buffer size. */
#define HWATCH_NRF_BLE_UART_RX_BUF_SIZE                512                                         /**< UART RX buffer size. */

/* GPIO pin for SPI MOSI(12/P0.13) */
#define HWATCH_NRF_BLE_SPI_MOSI_PIN ((nrfx_gpiote_pin_t)13)

/* GPIO pin for SPI MISO(14/P0.14) */
#define HWATCH_NRF_BLE_SPI_MISO_PIN ((nrfx_gpiote_pin_t)14)

/* GPIO pin for SPI SCK(13/P0.12)*/
#define HWATCH_NRF_BLE_SPI_SCK_PIN ((nrfx_gpiote_pin_t)12)

/* GPIO pin for SPI CS(11/P0.15)*/
#define HWATCH_NRF_BLE_SPI_CS_PIN ((nrfx_gpiote_pin_t)15)

/* SPI device bit rates */
#define HWATCH_SPI_FREQ_125K        HWATCH_SPI_MASTER_FREQ_125K
#define HWATCH_SPI_FREQ_250K        HWATCH_SPI_MASTER_FREQ_250K
#define HWATCH_SPI_MASTER_FREQ_125K NRF_SPIM_FREQ_125K
#define HWATCH_SPI_MASTER_FREQ_250K NRF_SPIM_FREQ_250K

/* SPI TX-RX Bit order */
#define HWATCH_SPI_MASTER_MSB_FIRST NRF_SPIM_BIT_ORDER_MSB_FIRST
#define HWATCH_SPI_MASTER_LSB_FIRST NRF_SPIM_BIT_ORDER_LSB_FIRST

#define HWATCH_SPI_SLAVE_MSB_FIRST NRF_SPIS_BIT_ORDER_MSB_FIRST
#define HWATCH_SPI_SLAVE_LSB_FIRST NRF_SPIS_BIT_ORDER_MSB_FIRST

/* SPI master. polarity of CS during TX-RX */
#define HWATCH_NRF_BLE_SPI_CS_ACTIVE_LOW  false
#define HWATCH_NRF_BLE_SPI_CS_ACTIVE_HIGH true

/* GPIO pin for Wakeup input signal */
#define HWATCH_NRF_BLE_WAKEUP_PIN HWATCH_NRF_BLE_SPI_MOSI_PIN

/* GPIO pin for ACK output signal */
#define HWATCH_NRF_BLE_ACK_PIN HWATCH_NRF_BLE_SPI_MISO_PIN

/* SPI Mode (clock phase and polarity)
   Mode 0(CPOL=0, CPHASE=0),
   Mode 1(CPOL=0, CPHASE=1),
   Mode 2(CPOL=1, CPHASE=0),
   Mode 3(CPOL=1, CPHASE=1)
*/
#if (HWATCH_SPI_CPOL == HWATCH_SPI_CPOL_0) && (HWATCH_SPI_CPHASE == HWATCH_SPI_CPHASE_0)

#define HWATCH_NRF_BLE_SPI_MASTER_MODE  NRF_SPIM_MODE_0
#define HWATCH_NRF_BLE_SPI_SLAVE_MODE   NRF_SPIS_MODE_0

#elif (HWATCH_SPI_CPOL == HWATCH_SPI_CPOL_0) && (HWATCH_SPI_CPHASE == HWATCH_SPI_CPHASE_1)

#define HWATCH_NRF_BLE_SPI_MASTER_MODE  NRF_SPIM_MODE_1
#define HWATCH_NRF_BLE_SPI_SLAVE_MODE   NRF_SPIS_MODE_1

#elif (HWATCH_SPI_CPOL == HWATCH_SPI_CPOL_1) && (HWATCH_SPI_CPHASE == HWATCH_SPI_CPHASE_0)

#define HWATCH_NRF_BLE_SPI_MASTER_MODE  NRF_SPIM_MODE_2
#define HWATCH_NRF_BLE_SPI_SLAVE_MODE   NRF_SPIS_MODE_2

#elif (HWATCH_SPI_CPOL == HWATCH_SPI_CPOL_1) && (HWATCH_SPI_CPHASE == HWATCH_SPI_CPHASE_1)

#define HWATCH_NRF_BLE_SPI_MASTER_MODE  NRF_SPIM_MODE_3
#define HWATCH_NRF_BLE_SPI_SLAVE_MODE   NRF_SPIS_MODE_3

#else

#define HWATCH_NRF_BLE_SPI_MASTER_MODE  NRF_SPIM_MODE_0
#define HWATCH_NRF_BLE_SPI_SLAVE_MODE   NRF_SPIS_MODE_0

#endif

/* SPI device instance */
#define SPI_DEV_ID_0 0
#define SPI_DEV_ID_1 1

/* State of the NRF system */
typedef enum
{
  HWATCH_NRF_STATE_POWER_ON_RST,
  HWATCH_NRF_STATE_DEEP_SLEEP,
  HWATCH_NRF_STATE_HW_INIT,
  HWATCH_NRF_STATE_NAP_SLEEP,
  HWATCH_NRF_STATE_CONFIRM_WAKEUP_TO_ADI,
  HWATCH_NRF_STATE_PAIR,
  HWATCH_NRF_STATE_WAIT_FOR_USER_DETAILS,
  HWATCH_NRF_STATE_WAIT_FOR_USER_TO_WEAR,
  HWATCH_NRF_STATE_WAIT,
  HWATCH_NRF_STATE_STORING_SENSOR_DATA,
  HWATCH_NRF_STATE_STREAM_SENSOR_DATA,

  HWATCH_NRF_STATE_HW_CONFIG,
  HWATCH_NRF_STATE_ISRS_INIT,
  HWATCH_NRF_STATE_IRQ_ENABLE,
  HWATCH_NRF_STATE_SLEEP_CORE,
  HWATCH_NRF_STATE_SLEEP_SYS,
  HWATCH_NRF_STATE_SLEEP_HIB,
  HWATCH_NRF_STATE_AWAKEN,
  HWATCH_NRF_STATE_SHUT_DOWN,
  HWATCH_NRF_STATE_BLE_SPI_COM,
  HWATCH_NRF_STATE_RX_RH_SENSOR_DATA,
  HWATCH_NRF_STATE_RX_TEMP_SENSOR_DATA,
  HWATCH_NRF_STATE_RX_PXMTY_SENSOR_DATA
} HWATCH_NRF_STATE_TYPE;
#endif /* HWATCH_NRF_COMMON_H */
