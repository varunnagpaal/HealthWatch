/** 
 * \file common.h
 * \brief Macros, typedefs, enum common for AD and nRF module
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */
#ifndef HWATCH_COMMON_H
#define HWATCH_COMMON_H

#include <stdint.h>

/* Macros for bit manipulation */
#define HWATCH_SET_BIT_MSK(var,bitmask)    ( ( var ) |=  ( bitmask ) )
#define HWATCH_CLEAR_BIT_MSK(var,bitmask)  ( ( var ) &= ~( bitmask ) )
#define HWATCH_TOGGLE_BIT_MSK(var,bitmask) ( ( var ) ^=  ( bitmask ) )
#define HWATCH_CHECK_BIT_MSK(var,bitmask)  ( ( var )  &  ( bitmask ) )

#define HWATCH_SET_BIT_IDX(var,idx)    ( ( var ) |=  ( 1 << ( idx ) ) )
#define HWATCH_CLEAR_BIT_IDX(var,idx)  ( ( var ) &= ~( 1 << ( idx ) ) )
#define HWATCH_TOGGLE_BIT_IDX(var,idx) ( ( var ) ^=  ( 1 << ( idx ) ) )
#define HWATCH_CHECK_BIT_IDX(var,idx)  ( ( var )  &  ( 1 << ( idx ) ) )

/* SPI Device bit rate set to 125K as it works for both ADI and BLE */
#define HWATCH_SPI_BIT_RATE     HWATCH_SPI_FREQ_125K

/* We use default SPI mode 0 (CPOL=0, CPHASE=0) */
#define HWATCH_SPI_CPOL         HWATCH_SPI_CPOL_0
#define HWATCH_SPI_CPHASE       HWATCH_SPI_CPHASE_0

/* SPI TX-RX Bit order is set as MSB first */
#define HWATCH_SPI_BIT_ORDER        HWATCH_SPI_MSB_FIRST
#define HWATCH_SPI_MASTER_BIT_ORDER HWATCH_SPI_MASTER_MSB_FIRST
#define HWATCH_SPI_SLAVE_BIT_ORDER  HWATCH_SPI_SLAVE_MSB_FIRST

/* Buffer sizes for SPI TX/RX */
#define HWATCH_DATA_SYNC_PROLOGUE_BUFFER_SIZE 2u
#define HWATCH_DATA_SYNC_PAYLOAD_SAMPLE_CNT 10u
#define HWATCH_DATA_SYNC_EPILOGUE_BUFFER_SIZE 2u

/* Unions to convert double/float to bytes  */
typedef union
{
  float baseFloatValue;
  uint8_t bytes[sizeof(float)];
}HWATCH_FLOAT_TO_BYTES_TYPE;

typedef union
{
  double baseFloatValue;
  uint8_t bytes[sizeof(float)];
}HWATCH_DOUBLE_TO_BYTES_TYPE;

#define HWATCH_USER_ID_DATA_TYPE uint16_t
#define HWATCH_DEV_ID_DATA_TYPE uint16_t
#define HWATCH_TIMESTAMP_DATA_TYPE uint16_t
#define HWATCH_FREQ_DATA_TYPE uint16_t
#define HWATCH_TEMP_DATA_TYPE uint16_t
#define HWATCH_RH_DATA_TYPE uint16_t
#define HWATCH_ZMOD_DATA_TYPE HWATCH_FLOAT_TO_BYTES_TYPE
#define HWATCH_ZPHASE_DATA_YPE HWATCH_FLOAT_TO_BYTES_TYPE
#define HWATCH_CH_DATA_TYPE uint16_t

/* For current user id, each record in flash has following format:
    userid(uint16_t, primary key),
    devid(uint16_t),
    timestamp(uint16_t, primary key),
    frequency(uint16_t),
    temperature(uint16_t),
    rh(uint16_t),
    zmod(float),
    zphase(float),
    channel(uint16_t)
*/
typedef struct
{
  HWATCH_USER_ID_DATA_TYPE    userId; /* NOTE: this may be removed as phone already has this. TBU */
  HWATCH_DEV_ID_DATA_TYPE     devId;
  HWATCH_TIMESTAMP_DATA_TYPE  timestamp;
  HWATCH_FREQ_DATA_TYPE       frequency;
  HWATCH_TEMP_DATA_TYPE       temperature;
  HWATCH_RH_DATA_TYPE         rh;
  HWATCH_ZMOD_DATA_TYPE       zmod;
  HWATCH_ZPHASE_DATA_YPE      zphase;
  HWATCH_CH_DATA_TYPE         channel;
}HWATCH_FLASH_MEM_RECORD_TYPE;

/* SPI Modes */
typedef enum
{
  HWATCH_SPI_MODE_MASTER,
  HWATCH_SPI_MODE_SLAVE
}HWATCH_SPI_MODE_TYPE;

/* Common messages between AD and nRF BLE */
#define HWATCH_READER_MSG_READY             ((uint8_t)0x01)
#define HWATCH_READER_MSG_WAIT              ((uint8_t)0x02)
#define HWATCH_READER_MSG_ACK               ((uint8_t)0x03)
#define HWATCH_READER_MSG_RETRY             ((uint8_t)0x04)
#define HWATCH_READER_MSG_SLEEP             ((uint8_t)0x05)
#define HWATCH_READER_MSG_WAKEUP            ((uint8_t)0x06)
#define HWATCH_READER_MSG_PAIR              ((uint8_t)0x07)
#define HWATCH_READER_MSG_UNPAIR            ((uint8_t)0x08)
#define HWATCH_READER_MSG_WORN              ((uint8_t)0x09)
#define HWATCH_READER_MSG_UNWORN            ((uint8_t)0x0A)
#define HWATCH_READER_MSG_USER_DETAILS      ((uint8_t)0x0B)
#define HWATCH_READER_MSG_NEW_USER          ((uint8_t)0x0C)
#define HWATCH_READER_MSG_MEASUREMENT_DONE  ((uint8_t)0x0D)
#define HWATCH_READER_MSG_START_DATA_SYNC   ((uint8_t)0x0E)
#define HWATCH_READER_MSG_CONT_DATA_SYNC    ((uint8_t)0x0F)
#define HWATCH_READER_MSG_STOP_DATA_SYNC    ((uint8_t)0x10)
#define HWATCH_READER_MSG_ERROR             ((uint8_t)0xEE)
#define HWATCH_READER_MSG_DUMMY             ((uint8_t)0xFF)

#define TRY_SET_ERR( func,\
                     func_expected_success_return_value,\
                     return_result_variable,\
                     on_fail_return_value,\
                     cleanup_label )\
        if( (func_expected_success_return_value) != (func) )\
        {\
          return_result_variable = (on_fail_return_value);\
          goto cleanup_label;\
        }

#define TRY_CHK_ERR( func,\
                     func_expected_success_return_value,\
                     func_return_result_variable,\
                     cleanup_label )\
        if( (func_expected_success_return_value)\
            != ( func_return_result_variable = (func) ) )\
            {\
              goto cleanup_label;\
            }

#define TRY_FAIL( func,\
                  func_expected_success_return_value,\
                  error_msg )\
        if( (func_expected_success_return_value) != (func) )\
            {\
              FAIL( error_msg );\
            }

#endif /* HWATCH_COMMON_H */
