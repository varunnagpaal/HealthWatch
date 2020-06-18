/** 
 * \file fsm.c
 * \brief Finite State Machine functions for nRF module
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */
 
#include "fsm.h"
#include "init.h"
#include "helpers.h"
#include "nrf_log.h"
#include "nrfx_spim.h"
#include "nrfx_spis.h"

HWATCH_NRF_STATE_TYPE lastState;
HWATCH_NRF_STATE_TYPE currentState;
HWATCH_NRF_STATE_TYPE nextState;

extern volatile bool bAdiWokeFromDeepSleep;
extern volatile bool bAdiWokeFromNapSleep;
extern volatile bool bNapSleepTimedOutFlag;
extern volatile bool bAdvTimedOutFlag;
extern volatile bool bIpaired;
extern nrfx_spim_t const hSpiMasterDev0;
extern nrfx_spis_t const hSpiSlaveDev0;

HWATCH_NRF_STATE_TYPE NRF_State_PowerOnReset( void )
{
  currentState = nextState;
  nextState = HWATCH_NRF_STATE_DEEP_SLEEP;

  /* Initialize minimum hardware:
    1. Logging
    2. GPIOS:
       a. Wake up input signal (from deep sleep) interrupt on GPIO(SPI MOSI pin) from ADI
       b. ACK output signal from BLE to ADI on GPIO(SPI MISO pin)
    3. Power management
    4. Clocks
  */
  log_init();
  gpio_init();
  power_management_init();
  clks_init();

  lastState = currentState;
  return nextState;
}
HWATCH_NRF_STATE_TYPE NRF_State_DeepSleep( void )
{
  currentState = nextState;
  NRF_LOG_INFO("[%d]%s -> %s", order++, HWATCH_State_To_Str(lastState), HWATCH_State_To_Str(currentState));

  switch(lastState)
  {
    case HWATCH_NRF_STATE_DEEP_SLEEP:
    case HWATCH_NRF_STATE_HW_INIT:
    case HWATCH_NRF_STATE_NAP_SLEEP:
    case HWATCH_NRF_STATE_CONFIRM_WAKEUP_TO_ADI:
    case HWATCH_NRF_STATE_PAIR:
    case HWATCH_NRF_STATE_WAIT_FOR_USER_DETAILS:
    case HWATCH_NRF_STATE_WAIT_FOR_USER_TO_WEAR:
    default:
          HWATCH_NRF_BLE_EnterDeepSleepMode();
          break;

    case HWATCH_NRF_STATE_POWER_ON_RST:
      /* Before going to deep sleep, enable interrupt on wake up pin */
//      HWATCH_NRF_BLE_EnableGPIOInt( HWATCH_NRF_BLE_WAKEUP_PIN );

      /* Before going to sleep, check if we are here for the first time
         or second time due to a reset caused when I was woken up from
         deep sleep(System OFF) through a wakeup signal from AD */
      bool bPowerOnReset = nrf_drv_gpiote_in_is_set(HWATCH_NRF_BLE_WAKEUP_PIN);
      if( true == bPowerOnReset )
      {
        /* Wait for ADI to initialize first before going to sleep.
           This is done so that on POR, just in case BLE races ahead and
           sleeps first, ADI shouldn't accidentally wake up the BLE while
           initializing its GPIO even though it didn't send an explcit wakeup
           during initialization */
        //HWATCH_NRF_Delay(HWATCH_NRF_BLE_AD_WAIT_POR);

        /* first time power on reset. So go to deep sleep mode forever
           while waiting for wake up signal on GPIO(SPI MOSI pin) from ADI */
        HWATCH_NRF_BLE_EnterDeepSleepMode();
      }

      break;
  }

  // woke up from deep sleep
  bAdiWokeFromDeepSleep = true;
  nextState = HWATCH_NRF_STATE_HW_INIT;

  lastState = currentState;
  return nextState;
}

HWATCH_NRF_STATE_TYPE NRF_State_HardwareInit( void )
{
  currentState = nextState;
  NRF_LOG_INFO("[%d]%s -> %s", order++, HWATCH_State_To_Str(lastState), HWATCH_State_To_Str(currentState));

  ble_stack_init();
  uart_init();
  timers_init();
  gap_params_init();
  gatt_init();
  services_init();
  advertising_init();
  conn_params_init();

  nextState = HWATCH_NRF_STATE_CONFIRM_WAKEUP_TO_ADI;

  lastState = currentState;
  return nextState;
}

HWATCH_NRF_STATE_TYPE NRF_State_ConfirmWakeupToAdi( void )
{
  currentState = nextState;
  NRF_LOG_INFO("[%d]%s -> %s", order++, HWATCH_State_To_Str(lastState), HWATCH_State_To_Str(currentState));

  /* Nap waiting for few seconds(<<1sec) for ADI before sending the ACK signal */
  HWATCH_NRF_BLE_StartNapTimerMsec(HWATCH_NRF_BLE_AD_WAIT_TIMEOUT);
  HWATCH_NRF_BLE_EnterNapMode(&bNapSleepTimedOutFlag);

  /* Send ACK to ADI (HI to LO) */
  HWATCH_NRF_BLE_AssertAckToAdi();
  HWATCH_NRF_BLE_DeAssertAckToAdi();

  /* Next, initialize rest of the hardware to prepare for pairing */
  nextState = HWATCH_NRF_STATE_PAIR;

  lastState = currentState;
  return nextState;
}

HWATCH_NRF_STATE_TYPE NRF_State_Pair( void )
{
  currentState = nextState;
  NRF_LOG_INFO("[%d]%s -> %s", order++, HWATCH_State_To_Str(lastState), HWATCH_State_To_Str(currentState));

  /* Start advertising for 15 sec. If not Connected
     go back to nap for 5 sec. Wakeup and advertise
     again
  */
  uint32_t napTimeAccum = 0;
  uint32_t advTimeAccum = 0;

  do
  {
    /* 1. Start Advertising timer with 15 sec timeout */
    NRF_LOG_INFO("[%d]NRF_State_Pair: starting advertising for %u msec",
                 order++,
                 HWATCH_NRF_BLE_ADV_TIMEOUT );

    HWATCH_NRF_BLE_StartAdvTimerMsec(HWATCH_NRF_BLE_ADV_TIMEOUT);
    HWATCH_NRF_BLE_AdvStart();

    /* 2. Enter nap sleep between advertising events */
    bAdvTimedOutFlag = false;
    HWATCH_NRF_BLE_EnterNapMode(&bAdvTimedOutFlag);

    /* 3. If BLE successfully paired, come out of this loop.
          Else if advertising timer has timed out, then stop advertising and nap for 5 sec
          Else flag unexpected error
    */
    if( true == bIpaired )
    {
      NRF_LOG_INFO("[%d]NRF_State_Pair: successfully paired", order++);
      break;
    }
    else if( true == bAdvTimedOutFlag )
    {
      NRF_LOG_WARNING("[%d]NRF_State_Pair: stopping advertisement", order++);
      HWATCH_NRF_BLE_AdvStop();

      HWATCH_NRF_BLE_StartNapTimerMsec(HWATCH_NRF_BLE_NAP_TIMEOUT);

      NRF_LOG_WARNING("[%d]NRF_State_Pair: going to nap sleep for %u msec", order++, HWATCH_NRF_BLE_NAP_TIMEOUT);
      bNapSleepTimedOutFlag = false;
      HWATCH_NRF_BLE_EnterNapMode(&bNapSleepTimedOutFlag);
    }
    else
    {
      NRF_LOG_WARNING("[%d]NRF_State_Pair: stopping advertisement", order++);
      HWATCH_NRF_BLE_AdvStop();
      NRF_LOG_ERROR("[%d]N: neither paired not timer expired. Something wrong!!", order++);
      break;
    }

    if( true == bNapSleepTimedOutFlag )
    {
      NRF_LOG_WARNING("[%d]NRF_State_Pair: woke up from nap sleep", order++);
    }

    /* 4. When Nap timer times out, system will wake up and repeat step 1. We make three attempts
          before we exhaust total 60 sec of advertisement (45 sec) + nap interval(15 sec) and go back to deep sleep        
    */
    napTimeAccum += HWATCH_NRF_BLE_NAP_TIMEOUT;
    advTimeAccum += HWATCH_NRF_BLE_ADV_TIMEOUT;
  }while( (advTimeAccum+napTimeAccum) < HWATCH_NRF_BLE_ADV_NAP_TIMEOUT );

  if(true == bIpaired)
  {
    NRF_LOG_INFO("[%d]NRF_State_Pair: successfully paired", order++);

//    /* Nap waiting for few seconds(<<1sec) for ADI before sending the ACK signal */
//    HWATCH_NRF_BLE_StartNapTimerMsec(HWATCH_NRF_BLE_AD_WAIT_TIMEOUT);
//    HWATCH_NRF_BLE_EnterNapMode(&bNapSleepTimedOutFlag);
//
//    /* Send ACK to ADI (HI to LO) */
//    HWATCH_NRF_BLE_AssertAckToAdi();
//    HWATCH_NRF_BLE_DeAssertAckToAdi();

    nextState =  HWATCH_NRF_STATE_WAIT_FOR_USER_DETAILS;
  }
  else
  {
    NRF_LOG_WARNING("[%d]NRF_State_Pair: failed to pair!!", order++);
    nextState =  HWATCH_NRF_STATE_DEEP_SLEEP;
  }

  lastState = currentState;
  return nextState;
}

HWATCH_NRF_STATE_TYPE NRF_State_Nap( void )
{
  currentState = nextState;
  NRF_LOG_INFO("[%d]%s -> %s", order++, HWATCH_State_To_Str(lastState), HWATCH_State_To_Str(currentState));

  /* go to nap sleep mode */
  bNapSleepTimedOutFlag = false;
  HWATCH_NRF_BLE_EnterNapMode(&bNapSleepTimedOutFlag);

  lastState = currentState;
  return nextState;
}

HWATCH_NRF_STATE_TYPE NRF_State_Wait( void )
{
  currentState = nextState;
  NRF_LOG_INFO("[%d]%s -> %s", order++, HWATCH_State_To_Str(lastState), HWATCH_State_To_Str(currentState));

  nextState =  HWATCH_NRF_STATE_DEEP_SLEEP;

  lastState = currentState;
  return nextState;
}

HWATCH_NRF_STATE_TYPE NRF_State_WaitForUserDetails( void )
{
  currentState = nextState;
  NRF_LOG_INFO("[%d]%s -> %s", order++, HWATCH_State_To_Str(lastState), HWATCH_State_To_Str(currentState));

  bool bBlockingMode = false;
  spi_init(&hSpiSlaveDev0, HWATCH_SPI_MODE_SLAVE, bBlockingMode );

  nextState =  HWATCH_NRF_STATE_DEEP_SLEEP;

  lastState = currentState;
  return nextState;
}

HWATCH_NRF_STATE_TYPE NRF_State_WaitForUserToWear( void )
{
  currentState = nextState;
  NRF_LOG_INFO("[%d]%s -> %s", order++, HWATCH_State_To_Str(lastState), HWATCH_State_To_Str(currentState));

  nextState =  HWATCH_NRF_STATE_DEEP_SLEEP;

  lastState = currentState;
  return nextState;
}

HWATCH_NRF_STATE_TYPE NRF_State_StoringSensorData( void )
{
  currentState = nextState;
  NRF_LOG_INFO("[%d]%s -> %s", order++, HWATCH_State_To_Str(lastState), HWATCH_State_To_Str(currentState));

  nextState =  HWATCH_NRF_STATE_DEEP_SLEEP;

  lastState = currentState;
  return nextState;
}

HWATCH_NRF_STATE_TYPE NRF_State_StreamSensorData( void )
{
  currentState = nextState;
  NRF_LOG_INFO("[%d]%s -> %s", order++, HWATCH_State_To_Str(lastState), HWATCH_State_To_Str(currentState));

  nextState =  HWATCH_NRF_STATE_DEEP_SLEEP;

  lastState = currentState;
  return nextState;
}

HWATCH_NRF_STATE_TYPE NRF_State_Shutdown( void )
{
  currentState = nextState;
  NRF_LOG_INFO("[%d]%s -> %s", order++, HWATCH_State_To_Str(lastState), HWATCH_State_To_Str(currentState));

  nextState =  HWATCH_NRF_STATE_DEEP_SLEEP;

  lastState = currentState;
  return nextState;
}
