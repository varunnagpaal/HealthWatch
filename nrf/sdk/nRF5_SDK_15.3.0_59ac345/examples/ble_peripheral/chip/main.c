/** 
 * \file main.c
 * \brief top level main file for nRF module
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */
 
#include "fsm.h"

int main(void)
{
  HWATCH_NRF_STATE_TYPE nextState = HWATCH_NRF_STATE_POWER_ON_RST;
  do
  {
    switch (nextState)
    {
      case HWATCH_NRF_STATE_POWER_ON_RST:
        nextState = NRF_State_PowerOnReset();
        break;

      case HWATCH_NRF_STATE_DEEP_SLEEP:
        nextState = NRF_State_DeepSleep();
        break;

      case HWATCH_NRF_STATE_CONFIRM_WAKEUP_TO_ADI:
        nextState = NRF_State_ConfirmWakeupToAdi();
        break;

      case HWATCH_NRF_STATE_HW_INIT:
        nextState = NRF_State_HardwareInit();
        break;

      case HWATCH_NRF_STATE_PAIR:
        nextState = NRF_State_Pair();
        break;

      case HWATCH_NRF_STATE_NAP_SLEEP:
        nextState = NRF_State_Nap();
        break;

      case HWATCH_NRF_STATE_WAIT_FOR_USER_DETAILS:
        nextState = NRF_State_WaitForUserDetails();// 1 minute
        break;

      case HWATCH_NRF_STATE_WAIT_FOR_USER_TO_WEAR:
        nextState = NRF_State_WaitForUserToWear();
        break;

      case HWATCH_NRF_STATE_STORING_SENSOR_DATA:
        nextState = NRF_State_StoringSensorData();
        break;

      case HWATCH_NRF_STATE_STREAM_SENSOR_DATA:
        nextState = NRF_State_StreamSensorData();
        break;

      default:
        break;
    }
  } while(1);
}
