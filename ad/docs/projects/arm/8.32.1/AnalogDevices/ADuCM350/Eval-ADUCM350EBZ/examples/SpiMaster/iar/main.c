/** 
 * \file main.c
 * \brief top level main file for AD module
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "test_common.h"
#include "fsm.h"
#include "log.h"

/* __argc, the number of arguments in __argv. */
__no_init int __argc;
/* __argv, an array of pointers to strings that holds the
arguments; must be large enough to fit the number of
parameters.*/
__no_init const char * __argv[10];
/* __argvbuf, a storage area for __argv; must be large enough to
hold all command line parameters. */
__no_init __root char __argvbuf[255];

unsigned int ad_log_level = HWATCH_LOG_LEVEL_OFF;
unsigned int ad_log_id = 0;

int main(void)
{
  unsigned int cmdLineLogLevel = ad_log_level;
  if( __argc == 1) 
  { 
    cmdLineLogLevel = strtol(__argv[__argc-1], NULL, 10);
    if( cmdLineLogLevel <= HWATCH_LOG_LEVEL_DEBUG )
    {
      ad_log_level = cmdLineLogLevel;
    }
  }
  
  HWATCH_LOG_INFO("Setting log level to: %s(%d)", HWATCH_Log_Level_To_Str(ad_log_level), ad_log_level);

  HWATCH_AD_STATE_TYPE nextState = HWATCH_AD_STATE_HW_INIT;
  do {
    switch (nextState) {
      case HWATCH_AD_STATE_HW_INIT:
        nextState = AD_State_HardwareInit();
        break;

      case HWATCH_AD_STATE_SLEEP:
        nextState = AD_State_Sleep();
        break;

      case HWATCH_AD_STATE_WAIT_FOR_BLE_TO_WAKEUP:
        nextState = AD_State_WaitForBleToWakeup();
        break;

      case HWATCH_AD_STATE_WAIT_FOR_BLE_TO_PAIR:
        nextState = AD_State_WaitForBleToPair();
        break;

      case HWATCH_AD_STATE_WAIT_FOR_USER_DETAILS:
        nextState = AD_State_WaitForUserDetails();
        break;

      case HWATCH_AD_STATE_WAIT_FOR_USER_TO_WEAR:
        nextState = AD_State_WaitForUserToWear();
        break;

      case HWATCH_AD_STATE_STORE_SENSOR_DATA:
        nextState = AD_State_StoreSensorData();
        break;

      case HWATCH_AD_STATE_STREAM_SENSOR_DATA:
        nextState = AD_State_StreamSensorData();
        break;

      default:
        break;
    }
  } while(1);
}
