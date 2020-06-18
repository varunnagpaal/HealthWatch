/** 
 * \file fsm.c
 * \brief Finite State Machine functions for AD module
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
#include "test_common.h"
#include "assert.h"
#include "helpers.h"
#include "pin_mux.h"
#include <string.h>
#include <stdio.h>
#include "log.h"

HWATCH_AD_STATE_TYPE lastState;
HWATCH_AD_STATE_TYPE currentState;
HWATCH_AD_STATE_TYPE nextState;

/* Handler(s) for SPI device(s) */
extern ADI_SPI_DEV_HANDLE hSpiDev0;

/* Handler for WDT */
extern ADI_WDT_DEV_HANDLE hWatchdog;

/* Handler for General Purpose Timer(s) */
extern ADI_GPT_HANDLE hTimer1;

/* The below flags that are modified by the ISR's.
   The flags indicate which event has occured.
   Once the control returns to main, event handler
   code is executed depending upon which flag was set.
   The flags are declared volatile as they can be modified
   by both ISR and main code. R/W of these flags from main
   code requires that interrupts must be disabled before doing so.
*/
extern volatile bool_t bLongBtnIntFlag;
extern volatile bool_t bShortBtnIntFlag;
extern volatile bool_t bLowPowerExitFlag;
extern volatile bool_t bTimer1ExpiredFlag;
extern volatile bool_t bSleepWhileTimedWaiting;
extern volatile bool_t bBlePaired;
extern volatile bool_t bBleRespondedFlag;
extern volatile bool_t bBleWokeup;
bool_t bSpiDmaMode = true;
bool_t bSpiBlockingMode = false;
bool_t bSpiContinuousMode = true;
HWATCH_USER_ID_DATA_TYPE userId = (uint16_t)0x0000;

/* State logic functions */
HWATCH_AD_STATE_TYPE AD_State_HardwareInit( void )
{
  currentState = nextState;

  HWATCH_LOG_INFO("AD: Entered HWATCH_AD_STATE_HW_INIT");

  /* 1: Initialize Microcontroller subsystem */
  TRY_FAIL( InitSystemHardware(),
            HWATCH_AD_RESULT_SUCCESS,
            "Failed to Intialize ADI System Hardware" );

  /* Before going to sleep, make sure nRF BLE is not accidentally
     interrupted if it happens to have already gone into sleep(active low)
     mode by now
  */
  DeAssertWakeUpToBle();

  /* Turn LED A and B ON" */
  LED_A_TURN_ON;
  LED_B_TURN_ON;
  HWATCH_LOG_DEBUG("AD: LED A ON, LED B ON");

  /* TBU: Optionally wait for few seconds for hardware to settle */

  /* Next, go to sleep */
  nextState = HWATCH_AD_STATE_SLEEP;
  HWATCH_LOG_INFO("AD: Next State HWATCH_AD_STATE_SLEEP");

  lastState = currentState;

  HWATCH_LOG_INFO("AD: Exit HWATCH_AD_STATE_HW_INIT");
  return nextState;
}

HWATCH_AD_STATE_TYPE AD_State_Sleep( void )
{
  currentState = nextState;

  HWATCH_LOG_INFO("AD: Entered HWATCH_AD_STATE_SLEEP");

  switch(lastState)
  {
    case HWATCH_AD_STATE_HW_INIT:
    case HWATCH_AD_STATE_SLEEP:
    case HWATCH_AD_STATE_WAIT_FOR_BLE_TO_WAKEUP:
    case HWATCH_AD_STATE_WAIT_FOR_BLE_TO_PAIR:      
    case HWATCH_AD_STATE_WAIT_FOR_USER_DETAILS:
    case HWATCH_AD_STATE_WAIT_FOR_USER_TO_WEAR:
    case HWATCH_AD_STATE_WAIT:
    case HWATCH_AD_STATE_STORE_SENSOR_DATA:
    case HWATCH_AD_STATE_STREAM_SENSOR_DATA:
    default: 
      assert(false == bLongBtnIntFlag);
      assert(false == bShortBtnIntFlag);
      assert(false == bBleRespondedFlag);
      assert(false == bTimer1ExpiredFlag);
      assert(false == bBleWokeup);
      assert(false == bBlePaired);
      break;
  }

#if 0
  OLD_LOG_HELPER(HWATCH_LOG_LEVEL_INFO, "Hello d=%d, f=%f, F=%F, e=%e, E=%E, g=%g, G=%g, s=%s",
           2,
           1.00,
           2.00,
           3.00,
           4.00,
           5.00,
           6.00,
           "a str");

  HWATCH_LOG(HWATCH_LOG_LEVEL_INFO, "Hello d=%d, f=%f, F=%F, e=%e, E=%E, g=%g, G=%g, s=%s",
           2,
           1.00,
           2.00,
           3.00,
           4.00,
           5.00,
           6.00,
           "a str");
  printf("Hello d=%d, f=%f, F=%F, e=%e, E=%E, g=%g, G=%g, s=%s",
           2,
           1.00,
           2.00,
           3.00,
           4.00,
           5.00,
           6.00,
           "a str");
#endif

  HWATCH_LOG_INFO("AD: Entered HWATCH_AD_STATE_WAIT_FOR_BLE_TO_WAKEUP");

  /* Turn LED A OFF and LED B OFF" */
  LED_A_TURN_OFF;
  LED_B_TURN_OFF;

  HWATCH_LOG_DEBUG("AD: LED A OFF, LED B OFF");

  /* 4: Enable the external interrupt (CPU and peripheral interrupt flags)
        on the button port pin before going to sleep */
  TRY_FAIL( EnableBtnIrq(),
            HWATCH_AD_RESULT_SUCCESS,
            "Failed to enable ADI's button IRQ before going to sleep" );

  /* 5: Goto lowest power sleep mode forever until button is pressed */
  HWATCH_LOG_INFO("AD: Going to untimed sleep waiting for button wakeup from user.");
  TRY_FAIL( Sleep( HWATCH_AD_SLEEP_MODE, &bLowPowerExitFlag ),
            HWATCH_AD_RESULT_SUCCESS,
            "Failed to put ADI to sleep for forever" );

  /* 6: Wait here (in sleep mode) for the external IRQ (button push)
        If an external IRQ is received, it is either of the following:
          Short button press (negative edge triggered interrupt)
          Long button press (active low level triggered interrupt)

        The ISR handler of External IRQ  will
        - Clear the corresponding external interrupt (CPU and peripheral
          interrupt flags) to prevent getting interrupted while serving
          last external IRQ
        - Cancel the low power mode in the status register (saved on stack
          when ISR was called) so that upon return from ISR, restoring the
          status register (pop) from stack, changes CPU state to active mode
          thereby enabling it to execute event handler code in this main loop.
        - Set flag bLongBtnIntFlag or bLongBtnIntFlag indicating
          the event that caused the external IRQ. The flag which are set in ISR
          are later used in this loop to actually handle the corresponding event(s).
  */

  /* 7: When the ISR returns, the execution will start from here

        First, Disable the external interrupts to prevent re-triggering
        (another button press event) while the event handling code below
        which handles the last external interrupt request is yet to be executed.

        Next, Depending on which ISR flag was set during ISR execution,
        execute the event handler code starting from highest priority
        to lowest priority event.
  */
  HWATCH_LOG_INFO("AD: Woke up from sleep.");
  TRY_FAIL( DisableBtnIrq(),
            HWATCH_AD_RESULT_SUCCESS,
            "Failed to disable ADI's button IRQ after wakeup from sleep" );

  /* Indicate wakeup with LED A ON and LED B ON */
  LED_A_TURN_ON;
  LED_B_TURN_ON;

  HWATCH_LOG_DEBUG("AD: LED A ON, LED B ON");

  if( true == bLongBtnIntFlag )
  {
    /* Clear the flag */
    bLongBtnIntFlag = false;
    HWATCH_LOG_DEBUG("AD: Reset Long Button Interrupt Flag");

    /* Power down the system */
    nextState= HWATCH_AD_STATE_SHUT_DOWN;

    HWATCH_LOG_INFO("AD: Next State HWATCH_AD_STATE_SHUT_DOWN");
  }
  else if ( true == bShortBtnIntFlag )
  {
    /* Clear the flag */
    bShortBtnIntFlag = false;
    HWATCH_LOG_DEBUG("AD: Reset Short Button Interrupt Flag");

    /* next, wait for ble to wakeup */
    nextState = HWATCH_AD_STATE_WAIT_FOR_BLE_TO_WAKEUP;
    HWATCH_LOG_INFO("AD: Next State HWATCH_AD_STATE_WAIT_FOR_BLE_TO_WAKEUP");
  }
  else
  {
    /* Neither short button nor long button was pressed
       and some other spurious event caused ADI to wakeup.
       So go back to sleep and flag error. TBU
    */
    nextState = HWATCH_AD_STATE_SLEEP;
    HWATCH_LOG_ERR("AD: Next State HWATCH_AD_STATE_SLEEP");
  }

  /* TBU: Optionally wait for few seconds before going to next state */

  lastState = currentState;

  HWATCH_LOG_INFO("AD: Exiting HWATCH_AD_STATE_SLEEP");
  return nextState;
}

HWATCH_AD_STATE_TYPE AD_State_Wait( void )
{
  currentState = nextState;

  /* TBU */

  lastState = currentState;
  return nextState;
}

HWATCH_AD_STATE_TYPE AD_State_WaitForBleToWakeup( void )
{
  float timeOut = 0.0;
  currentState = nextState;

  HWATCH_LOG_INFO("AD: Entered HWATCH_AD_STATE_WAIT_FOR_BLE_TO_WAKEUP");

  assert(false == bBleRespondedFlag);
  assert(false == bTimer1ExpiredFlag);
  assert(false == bBleWokeup);
  assert(false == bBlePaired);

  /* Indicate wakeup with LED A ON and LED B OFF */
  LED_A_TURN_ON;
  LED_B_TURN_OFF;

  HWATCH_LOG_DEBUG("AD: LED A ON, LED B OFF");

  /* Before sending wake up signal to BLE, enable
     the interrupt on ACK port pin (SPIO MISO) so
     that we are ready for its ACK response
  */
  EnableBleAckIrq();

  /* For upto 1 sec, assert wake up signal(Active when held to low) to the BLE on the SPI MOSI Port pin */
  AssertWakeUpToBle();

  bSleepWhileTimedWaiting = true;
  timeOut = 0.5;
  HWATCH_LOG_INFO("AD: Waiting %s sleep for %#.2f sec for BLE to receive wakeup signal",
                 ((bSleepWhileTimedWaiting)?"with":"without"),
                 timeOut );

  TRY_FAIL( StartTimer( &hTimer1,
                        HWATCH_AD_TIMER_1_IRQn,
                        timeOut,
                        bSleepWhileTimedWaiting,
                        HWATCH_AD_TIMER_1_SLEEP_MODE,
                        &bLowPowerExitFlag ),
            HWATCH_AD_RESULT_SUCCESS,
            "Failed to start the timer with sleep mode" );

  DeAssertWakeUpToBle();

  HWATCH_LOG_INFO("AD: Woke up from sleep waiting for BLE to capture wakeup signal.");

  /* Reset the timer */
  TRY_FAIL( ResetTimer( &hTimer1, HWATCH_AD_TIMER_1_IRQn, &bTimer1ExpiredFlag),
            HWATCH_AD_RESULT_SUCCESS,
            "Failed to stop the timer" );

  /* Optionally confirm if nRF BLE has woken up by waiting
     for ACK(active low) from BLE on the SPI0 MISO port pin
     TBU: its better to sleep here actually with a timer of <<1 sec
  */
  // for(uint16_t i=0; i<UINT16_MAX; i++){ if ( true == bBleRespondedFlag ) break; }
  bSleepWhileTimedWaiting = true;
  timeOut = 2;
  HWATCH_LOG_INFO("AD: Waiting %s sleep for %#.2f sec for ACK from BLE",
                 ((bSleepWhileTimedWaiting)?"with":"without"),
                 timeOut );
  TRY_FAIL( StartTimer( &hTimer1,
                        HWATCH_AD_TIMER_1_IRQn,
                        timeOut,
                        bSleepWhileTimedWaiting,
                        HWATCH_AD_TIMER_1_SLEEP_MODE,
                        &bLowPowerExitFlag ),
            HWATCH_AD_RESULT_SUCCESS,
            "Failed to start the timer with sleep mode" );

  /* We wake up from sleep starting here when either the timer expires or
     BLE responds with an ACK.
     Note: ACK interrupt has higher priority over timer interrupt
  */
  HWATCH_LOG_INFO("AD: Woke up from sleep waiting for ACK from BLE.");

  /* Disable the interrupt on the SPI0 MISO port pin from BLE to AD
     to prevent retriggering while event handling code below executes */
  DisableBleAckIrq();

  /* Stop the timer to prevent triggering its ISR over timer expiry */
  TRY_FAIL( StopTimer( &hTimer1, HWATCH_AD_TIMER_1_IRQn ),
            HWATCH_AD_RESULT_SUCCESS,
            "Failed to stop the timer" );

  HWATCH_LOG_DEBUG("AD: Disabled Timer to prevent retriggering ISR for BLE ACK signal.");

  if ( true == bBleRespondedFlag )
  {
    HWATCH_LOG_INFO("AD: ACK received from Ble indicating it wokeup! ");

    /* reset last BLE response to wait for next response */
    bBleRespondedFlag = false;
    HWATCH_LOG_DEBUG("AD: Reset last BLE response flag.");

    // /* Check if ACK received (line pulled low) */
    // if( adi_GPIO_GetData(HWATCH_AD_BLE_ACK_PORT) & HWATCH_AD_BLE_ACK_PIN )
    // {
    //   /* Pin is still pulled high by its internal pull up
    //      and hence BLE couldn't wake up from sleep until now
    //      even though it responded
    //   */
    //   bBleWokeup = false;
    //
    //   /* TBU: go back to sleep ? */
    //   nextState = HWATCH_AD_STATE_SLEEP;
    // }
    // else
    // {
    /* Pin is pulled low by BLE and hence ACK is received
      from BLE indicating it has woken up from sleep and is
      about to begin its attempt to pair.
    */
    bBleWokeup = true;
    HWATCH_LOG_DEBUG("AD: Set BLE wokeup flag.");

    /* Next, wait for BLE to pair */
    nextState = HWATCH_AD_STATE_WAIT_FOR_BLE_TO_PAIR;
    HWATCH_LOG_INFO("AD: Next state HWATCH_AD_STATE_WAIT_FOR_BLE_TO_PAIR");
  }
  else if( true == bTimer1ExpiredFlag )
  {
    HWATCH_LOG_WARN("AD: Timer Expired before BLE could wakeup");

    bTimer1ExpiredFlag = false;
    HWATCH_LOG_DEBUG("AD: Reset flag Timer Expired.");
    
    /* Next, as timer expired before BLE could respond, it is most likely 
       BLE also went to sleep so go back to sleep */
    nextState = HWATCH_AD_STATE_SLEEP;
    HWATCH_LOG_WARN("AD: Next state HWATCH_AD_STATE_SLEEP");
  }else
  {
    /* Neither timer expired nor ble responded.
       Note: this is erroneous condition and shouldn't occur
       False wake up from sleep. Something fishy !!
    */
    // FAIL( "AD woken up from sleep without button press interrupt!");
    HWATCH_LOG_ERR("AD: Neither timer expired nor ble wokeup.");

    /* TBU: go back to sleep for the time being */
    nextState = HWATCH_AD_STATE_SLEEP;
    HWATCH_LOG_ERR("AD: Next state HWATCH_AD_STATE_SLEEP");
  }

  lastState = currentState;

  HWATCH_LOG_INFO("AD: Exiting HWATCH_AD_STATE_WAIT_FOR_BLE_TO_WAKEUP");
  return nextState;
}

HWATCH_AD_STATE_TYPE AD_State_WaitForBleToPair( void )
{
  float timeOut = 0.0;
  currentState = nextState;

  HWATCH_LOG_INFO("AD: Entered HWATCH_AD_STATE_WAIT_FOR_BLE_TO_PAIR");

  assert(false == bBleRespondedFlag);
  assert(false == bTimer1ExpiredFlag);
  assert(true == bBleWokeup);
  assert(false == bBlePaired);

  /* Indicate wakeup with LED A OFF and LED B ON */
  LED_A_TURN_OFF;
  LED_B_TURN_ON;

  HWATCH_LOG_DEBUG("AD: LED A OFF, LED B ON");

  // /* Re-enable the button IRQ
  //    TBU: in all states, except init and sleep, button press should
  //         lead to sleep(short button) or shutdown(long button).
  //         This requires deregistering old btn isr, and registering a
  //         new isr
  // */
  // TRY_FAIL( EnableBtnIrq(),
  //           HWATCH_AD_RESULT_SUCCESS,
  //           "Failed to enable ADI's button IRQ before pairing" );
  // As of now, btn is disabled

  /* While waiting for BLE to pair,
     get ready for its ACK response
     on the ACK port pin (SPIO MISO)
  */
  EnableBleAckIrq();

  /* Start timer for 60+ sec and sleep while waiting for
     either timer to expire or for an interrupt on SPI MISO
     ACK Port pin from the BLE confirming BLE paired.
     TBU: increase priority of interrupt on ACK port pin
          relative to Timer interrupt
   */
  bSleepWhileTimedWaiting = true;
  timeOut = 65.00;
  HWATCH_LOG_INFO("AD: Waiting %s sleep for %#.2f sec for ACK from BLE",
                 ((bSleepWhileTimedWaiting)?"with":"without"),
                 timeOut );
  TRY_FAIL( StartTimer( &hTimer1,
                       HWATCH_AD_TIMER_1_IRQn,
                       timeOut,
                       bSleepWhileTimedWaiting,
                       HWATCH_AD_TIMER_1_SLEEP_MODE,
                       &bLowPowerExitFlag ),
           HWATCH_AD_RESULT_SUCCESS,
           "Failed to start the timer with sleep mode" );

  /* We are sleeping and waiting here for either the timer to expire or
    to receive ACK from nRF BLE that indicates if it has paired with phone
  */
  // while( !(bBleRespondedFlag || bTimer1ExpiredFlag) );

  HWATCH_LOG_INFO("AD: Woke up from sleep.");
  /* Disable the interrupt on the SPI0 MISO port pin from BLE to AD
     to prevent retriggering while event handling code below executes
  */
  DisableBleAckIrq();

  /* Stop the timer to prevent triggering its ISR over timer expiry */
  TRY_FAIL( StopTimer( &hTimer1, HWATCH_AD_TIMER_1_IRQn ),
            HWATCH_AD_RESULT_SUCCESS,
            "Failed to stop the timer" );

  HWATCH_LOG_DEBUG("AD: Disabled Timer to prevent retriggering ISR for BLE ACK signal.");

  /* Possible race between timer and BLE response unless interrupt
     priority of ble ack interrupt is made higher than timer interrupt
     TBU
  */
  if ( true == bBleRespondedFlag )
  {
    HWATCH_LOG_INFO("AD: ACK received from Ble indicating it paired! ");

    /* reset last response from BLE and wait for next response */
    bBleRespondedFlag = false;
    HWATCH_LOG_DEBUG("AD: Reset last BLE response flag.");

    bBlePaired = true;
    HWATCH_LOG_DEBUG("AD: Set BLE paired flag.");

    /* Next, wait for user to wear the reader */
    nextState = HWATCH_AD_STATE_WAIT_FOR_USER_DETAILS;
    HWATCH_LOG_INFO("AD: Next state HWATCH_AD_STATE_WAIT_FOR_USER_DETAILS");
  }
  else if( true == bTimer1ExpiredFlag )
  {
    HWATCH_LOG_WARN("AD: Timer Expired before BLE could pair");

    bTimer1ExpiredFlag = false;
    HWATCH_LOG_DEBUG("AD: Reset flags Timer Expired.");

    /* Go back to sleep as BLE also most likely went to sleep */
    bBleWokeup = false;
    nextState = HWATCH_AD_STATE_SLEEP;
    HWATCH_LOG_WARN("AD: Next state HWATCH_AD_STATE_SLEEP");
  }
  else
  {
    /* Neither timer expired nor ble responded.
       Note: this is erroneous condition and shouldn't occur
       False wake up from sleep. Something fishy !!
    */
    // FAIL( "AD woken up from sleep without button press interrupt!");
    HWATCH_LOG_ERR("AD: Neither timer expired nor ble paired.");

    /* TBU: go back to sleep for the time being */
    nextState = HWATCH_AD_STATE_SLEEP;
    HWATCH_LOG_ERR("AD: Next state HWATCH_AD_STATE_SLEEP");
  }

  lastState = currentState;

  HWATCH_LOG_INFO("AD: Exiting HWATCH_AD_STATE_WAIT_FOR_BLE_TO_PAIR");
  return nextState;
}

HWATCH_AD_STATE_TYPE AD_State_WaitForUserDetails( void )
{
  currentState = nextState;

  HWATCH_LOG_INFO("AD: Entered HWATCH_AD_STATE_WAIT_FOR_USER_DETAILS");

  assert(false == bBleRespondedFlag);
  assert(false == bTimer1ExpiredFlag);
  assert(true == bBleWokeup);
  assert(true == bBlePaired);

  ADI_SPI_TRANSCEIVE_TYPE transceive;

  /* All SPI buffer sizes must be multiple of 2 as
     SPI DMA engine of AD device operates is 2-byte aligned.
     Note: prologue buffer in transceive data  structure
           cannot be used with SPI in slave mode
  */
  uint8_t rxMsgBuffer[2+sizeof(HWATCH_USER_ID_DATA_TYPE)];
  uint8_t txMsgBuffer[2+sizeof(HWATCH_USER_ID_DATA_TYPE)];
  bool_t bIsUserIdInValid = false;

  /* Reconfigure GPIOs shared with SPI0 back to SPI0 pins */
  adi_initpinmux_setSPI0();

  /* Initialzie SPI0 device as slave device*/
  bSpiBlockingMode = false;
  bSpiDmaMode = true;
  bSpiContinuousMode = true;
  TRY_FAIL( InitSpi(ADI_SPI_DEVID_0,
                    &hSpiDev0,
                    HWATCH_SPI_MODE_SLAVE,
                    bSpiBlockingMode,
                    bSpiDmaMode,
                    bSpiContinuousMode),
            HWATCH_AD_RESULT_SUCCESS,
            "Failed to initialize spi device 0 as slave" );

  /* Initialize response(tx) buffer with 1 response byte, rest dummy bytes */
  txMsgBuffer[0] = HWATCH_READER_MSG_ACK;
  for(uint16_t i=1; i< sizeof(txMsgBuffer); ++i) txMsgBuffer[i] = HWATCH_READER_MSG_DUMMY;

  /* Initialize rx buffer with null bytes */
  for(uint16_t i=0; i< sizeof(rxMsgBuffer); ++i) rxMsgBuffer[i] = (uint8_t)0x00;

  transceive.pPrologue = NULL;  /* Note: SPI cannot use prologue buffer in slave mode */
  transceive.PrologueSize = 0;
  transceive.DataSize = 2+sizeof(HWATCH_USER_ID_DATA_TYPE);
  transceive.pTxData = &txMsgBuffer[0];
  transceive.pRxData = &rxMsgBuffer[0];
  transceive.bTxIncrement = true;
  transceive.bRxIncrement = true;

  /* Before initializing slave transfer, start a timer for 1.5 min
     waiting to receive user details from BLE over SPI  */
  bSleepWhileTimedWaiting = false;
  TRY_FAIL( StartTimer( &hTimer1,
                       HWATCH_AD_TIMER_1_IRQn,
                       90.00,
                       bSleepWhileTimedWaiting,
                       HWATCH_AD_TIMER_1_SLEEP_MODE,
                       &bLowPowerExitFlag ),
           HWATCH_AD_RESULT_SUCCESS,
           "Failed to start the timer without sleep mode" );

  /* Initialize SPI Slave Transfer.
     Note: in case of blocking mode, the call will block and
           only return after transfer is completed. otherwise,
           it will return without witing for transfer to finish
   */
  TRY_FAIL( adi_SPI_SlaveTransfer(hSpiDev0, &transceive),
            ADI_SPI_SUCCESS,
            "Failed to initiate slave transfer for AD SPI device 0 " );

  /* Wait for transfer to complete in case of non-blocking transfer */
  if( false == bSpiBlockingMode )
  {
    if( true == bSpiDmaMode )
    {
      /* DMA polling */
      while ( ( ( false == adi_SPI_GetDmaTxComplete(hSpiDev0) ) ||
                ( false == adi_SPI_GetDmaRxComplete(hSpiDev0) ) ) &&
              ( false == bTimer1ExpiredFlag ) );
    }
    else
    {
      /* Block waiting */
      while( ( false == adi_SPI_SlaveComplete(hSpiDev0) ) &&
              ( false == bTimer1ExpiredFlag ) );
    }
  }

  /* At this point, either transfer has completed or timer has expired  */
  if( false == bTimer1ExpiredFlag )
  {
    /* Transfer has completed but timer has not
       yet expired. So first, lets stop the timer.
    */
    TRY_FAIL( StopTimer( &hTimer1, HWATCH_AD_TIMER_1_IRQn ),
              HWATCH_AD_RESULT_SUCCESS,
              "Failed to stop the timer" );

    /* Next, we expect to have received following bytes:
        1 command byte: valid commands are WAIT, USER_DETAILS, UNPAIR, SLEEP
        1 dummy byte: as guard byte
        Remaining sizeof(HWATCH_USER_ID_DATA_TYPE) bytes is the payload data
        whose content depends upon command:
          USER_DETAILS:  sizeof(HWATCH_USER_ID_DATA_TYPE) bytes user id
          WAIT, UNPAIR, SLEEP: sizeof(HWATCH_USER_ID_DATA_TYPE) bytes dummies
    */
    switch( rxMsgBuffer[0] )
    {
      case HWATCH_READER_MSG_USER_DETAILS:
        /* next byte should be dummy */
        if( HWATCH_READER_MSG_DUMMY != rxMsgBuffer[1] )
        {
          /* As next byte is not dummy, something wrong went during last tx-rx.
             SPI TX to BLE telling that last tx-rx had Error and request to Retry.
             TBU
          */
          nextState = currentState;
        }
        else
        {
          /* Validate remaining sizeof(HWATCH_USER_ID_DATA_TYPE) bytes to be
             a valid sizeof(HWATCH_USER_ID_DATA_TYPE) byte user id
          */
          bIsUserIdInValid = false;
          for( uint8_t i=0; i < sizeof(HWATCH_USER_ID_DATA_TYPE); ++i )
          {
            userId |= ( (HWATCH_USER_ID_DATA_TYPE)rxMsgBuffer[2+i] << (8*i) );
            if( (false == bIsUserIdInValid) && (HWATCH_READER_MSG_DUMMY == rxMsgBuffer[2+i]) )
            {
              bIsUserIdInValid = true;
            }
          }

          if( true == bIsUserIdInValid )
          {
            /* User id validated. Next, wait for user to wear the BLE */
            nextState = HWATCH_AD_STATE_WAIT_FOR_USER_TO_WEAR;
          }
          else
          {
            /* All bytes are dummies indicating invalid user id.
               SPI TX to BLE telling that user id rx from it is invalid(Error) and
               request to send again (Retry).
               TBU
            */
            nextState = currentState;
          }
        }
        break;

      case HWATCH_READER_MSG_WAIT:
        /* Add a new wait state. TBU */
        nextState = HWATCH_AD_STATE_WAIT;
        break;

      case HWATCH_READER_MSG_UNPAIR:
        nextState = HWATCH_AD_STATE_WAIT_FOR_BLE_TO_PAIR;
        break;

      case HWATCH_READER_MSG_SLEEP:
      default:
        nextState = HWATCH_AD_STATE_SLEEP;
        break;
    }
  }
  else
  {
    /* if the timer has expired, then it means BLE most likely went to sleep */
    HWATCH_LOG_WARN("AD: Timer Expired before receiving user details from BLE");

    /* Go back to sleep as BLE also most likely went to sleep */
    bTimer1ExpiredFlag = false;
    bBlePaired = false;
    bBleWokeup = false;
    HWATCH_LOG_DEBUG("AD: Reset flags Timer Expired, BLE Paired, BLE Wokeup.");
    nextState = HWATCH_AD_STATE_SLEEP;
    HWATCH_LOG_WARN("AD: Next state HWATCH_AD_STATE_SLEEP");
  }

  HWATCH_LOG_INFO("AD: Exiting HWATCH_AD_STATE_WAIT_FOR_USER_DETAILS");

  lastState = currentState;
  return nextState;
}

HWATCH_AD_STATE_TYPE AD_State_WaitForUserToWear( void )
{
  currentState = nextState;

  HWATCH_LOG_INFO("AD: Entered HWATCH_AD_STATE_WAIT_FOR_USER_TO_WEAR");
  nextState = HWATCH_AD_STATE_STORE_SENSOR_DATA;
  HWATCH_LOG_INFO("AD: Exiting HWATCH_AD_STATE_WAIT_FOR_USER_TO_WEAR");

  lastState = currentState;
  return nextState;
}

HWATCH_AD_STATE_TYPE AD_State_StoreSensorData( void )
{
  currentState = nextState;

  HWATCH_LOG_INFO("AD: Entered HWATCH_AD_STATE_STORE_SENSOR_DATA");
  nextState = HWATCH_AD_STATE_STREAM_SENSOR_DATA;
  HWATCH_LOG_INFO("AD: Exiting HWATCH_AD_STATE_STORE_SENSOR_DATA");

  lastState = currentState;
  return nextState;
}

HWATCH_AD_STATE_TYPE AD_State_StreamSensorData( void )
{
  currentState = nextState;

  HWATCH_LOG_INFO("AD: Entered HWATCH_AD_STATE_STREAM_SENSOR_DATA");

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

    We need to first read all flash memory records for current userid and tx them one by one to
    the BLE over SPI. Next, once all flash records have been sent, we need to build each such
    record in real-time and tx it to BLE over SPI
  */
  HWATCH_FLASH_MEM_RECORD_TYPE flashMemRecord;

  /* tx 1 byte command start data sync, rest dummy bytes */
  uint8_t txPrologue[HWATCH_DATA_SYNC_PROLOGUE_BUFFER_SIZE];
  assert( 0 == ( sizeof(txPrologue) % 2) ); /* DMA transfers are 2-byte aligned */

  /* 1. Payload Data: tx 1 record from flash or real-time generated
     2. Epilogue: tx 1 byte commmand stop data sync, rest dummy bytes
  */
  uint8_t txMsgBuffer[sizeof(HWATCH_FLASH_MEM_RECORD_TYPE) + HWATCH_DATA_SYNC_EPILOGUE_BUFFER_SIZE];
  assert( 0 == ( sizeof(txMsgBuffer) % 2) );  /* DMA transfers are 2-byte aligned */

  /* 1. Response to Payload: rx 1 ACK byte, sizeof(record)-1 bytes of dummies
     2. Response to Epilogue: rx 1 ACK byte, rest dummy bytes
  */
  uint8_t rxMsgBuffer[sizeof(HWATCH_FLASH_MEM_RECORD_TYPE) + HWATCH_DATA_SYNC_EPILOGUE_BUFFER_SIZE];
  assert( 0 == ( sizeof(rxMsgBuffer) % 2) );  /* DMA transfers are 2-byte aligned */

  /* Set prologue bytes in Prologue Tx buffer: 1 byte command start data sync, rest dummy bytes */
  txPrologue[0] = HWATCH_READER_MSG_START_DATA_SYNC;
  for(uint16_t i = 1; i< sizeof(txPrologue); ++i ) txPrologue[i] = HWATCH_READER_MSG_DUMMY;

  /* Set epilogue bytes in Data Tx buffer: 1 byte command stop data sync, rest dummy bytes */
  txMsgBuffer[sizeof(HWATCH_FLASH_MEM_RECORD_TYPE)] = HWATCH_READER_MSG_STOP_DATA_SYNC;
  for(uint16_t i = 1 + sizeof(HWATCH_FLASH_MEM_RECORD_TYPE); i < sizeof(txMsgBuffer); ++i ) txMsgBuffer[i] = HWATCH_READER_MSG_DUMMY;

  /* Initialize Rx buffer with null bytes */
  for(uint16_t i=0; i< sizeof(rxMsgBuffer); ++i) rxMsgBuffer[i] = (uint8_t)0x00;

  /* Initialize SPI0 device as master device*/
  ADI_SPI_TRANSCEIVE_TYPE transceive;
  bSpiBlockingMode = false;
  bSpiDmaMode = true;
  bSpiContinuousMode = true;
  TRY_FAIL( InitSpi(ADI_SPI_DEVID_0,
                    &hSpiDev0,
                    HWATCH_SPI_MODE_MASTER,
                    bSpiBlockingMode,
                    bSpiDmaMode,
                    bSpiContinuousMode ),
            HWATCH_AD_RESULT_SUCCESS,
            "Failed to initialize spi device 0 as a master" );

  /* Configure SPi transceiver */
  transceive.pPrologue = &txPrologue[0];
  transceive.pTxData = &txMsgBuffer[0];
  transceive.pRxData = &rxMsgBuffer[0];
  transceive.PrologueSize = sizeof(txPrologue);
  transceive.DataSize = sizeof(txMsgBuffer);
  transceive.bTxIncrement = true;
  transceive.bRxIncrement = true;

  /* Fill payload data in tx buffer either by reading a flash record or generate record data
     in real-time (use RNG engine to generate random record data. TBU) and send record over
     SPI to BLE */
  flashMemRecord.userId = userId;
  flashMemRecord.devId = (HWATCH_DEV_ID_DATA_TYPE)0x0A; /* TBU */
  for( uint16_t i=0; i < (uint16_t)HWATCH_DATA_SYNC_PAYLOAD_SAMPLE_CNT; ++i)
  {
    /* build a random record */
    flashMemRecord.timestamp = (HWATCH_TIMESTAMP_DATA_TYPE)(i + (uint16_t)0x11);
    flashMemRecord.frequency = (HWATCH_FREQ_DATA_TYPE)(i + (uint16_t)0x2A);
    flashMemRecord.temperature = (HWATCH_TEMP_DATA_TYPE)(i+(uint16_t)0xBB);
    flashMemRecord.rh = (HWATCH_RH_DATA_TYPE)(i+(uint16_t)0xA0);
    flashMemRecord.zmod.baseFloatValue = 0.5;
    flashMemRecord.zphase.baseFloatValue = 3.14;
    flashMemRecord.channel = i%10;

    /* copy record to the payload data region of Data tx buffer */
    memcpy(&txMsgBuffer[0], &flashMemRecord, sizeof(flashMemRecord));

    /* Next, send the prologue, record and epilogue to BLE over SPI
       Initialize SPI Master Transfer.
       Note: in case of blocking mode, the call will block and
             only return after transfer is completed. otherwise,
             it will return without witing for transfer to finish.
             Also, AD ignores all rx bytes when tx prologue bytes
     */
    TRY_FAIL( adi_SPI_MasterTransfer(hSpiDev0, &transceive),
              ADI_SPI_SUCCESS,
              "Failed to initiate master transfer for AD SPI device 0 " );

    /* Wait for transfer to complete in case of non-blocking transfer */
    if( false == bSpiBlockingMode )
    {
      if( true == bSpiDmaMode )
      {
        /* DMA polling */
        while ( ( false == adi_SPI_GetDmaTxComplete(hSpiDev0) ) ||
                ( false == adi_SPI_GetDmaRxComplete(hSpiDev0) ) );
      }
      else
      {
        /* Block waiting */
        while( false == adi_SPI_MasterComplete(hSpiDev0) );
      }
    }

    /* At this point the tx-rx has completed. Check the rx message */
    switch( rxMsgBuffer[0] )
    {
      case HWATCH_READER_MSG_ACK:
      case HWATCH_READER_MSG_CONT_DATA_SYNC:
        /* Just continue streaming records */
        break;

      case HWATCH_READER_MSG_WAIT:
        /* Add a new wait state. TBU */
        nextState = HWATCH_AD_STATE_WAIT;
        break;

      case HWATCH_READER_MSG_STOP_DATA_SYNC:
        nextState = HWATCH_AD_STATE_STORE_SENSOR_DATA;
        break;

      case HWATCH_READER_MSG_UNPAIR:
        nextState = HWATCH_AD_STATE_WAIT_FOR_BLE_TO_PAIR;
        break;

      case HWATCH_READER_MSG_UNWORN:
        nextState = HWATCH_AD_STATE_WAIT_FOR_USER_TO_WEAR;
        break;

      case HWATCH_READER_MSG_SLEEP:
      default:
        nextState = HWATCH_AD_STATE_SLEEP;
        break;
    }

    /* Stop streaming data if next state is different than current state */
    if( nextState != currentState ) break;
  }

  HWATCH_LOG_INFO("AD: Exiting HWATCH_AD_STATE_STREAM_SENSOR_DATA");

  lastState = currentState;
  return nextState;
}

HWATCH_AD_STATE_TYPE AD_State_Shutdown( void )
{
  currentState = nextState;

  HWATCH_LOG_INFO("AD: Entered HWATCH_AD_STATE_SHUT_DOWN");
  HWATCH_LOG_INFO("AD: Exiting HWATCH_AD_STATE_SHUT_DOWN");

  lastState = currentState;
  return nextState;
}

/* Helper Functions */
static HWATCH_AD_RESULT_TYPE InitSystemHardware(void)
{
  HWATCH_AD_RESULT_TYPE err;

  TRY_CHK_ERR( InitClks(), HWATCH_AD_RESULT_SUCCESS, err, CATCH_ERROR_CLK_INIT );
  TRY_CHK_ERR( InitWatchDogTimer(), HWATCH_AD_RESULT_SUCCESS, err, CATCH_ERROR_WDT_INIT );
  TRY_CHK_ERR( InitGpio(), HWATCH_AD_RESULT_SUCCESS, err, CATCH_ERROR_GPIO_INIT );
  TRY_CHK_ERR( ConfigureGpio(), HWATCH_AD_RESULT_SUCCESS, err, CATCH_ERROR_GPIO_CONFIG );
  TRY_CHK_ERR( InitIntervalTimer(), HWATCH_AD_RESULT_SUCCESS, err, CATCH_ERROR_INIT_TIMER );

  HWATCH_LOG_INFO("AD: Initialized AD Hardware");

  return HWATCH_AD_RESULT_SUCCESS;

/* Cleanup */
//CATCH_ERROR_INIT_LOGGING:
  /* Failed to enable logging */
CATCH_ERROR_INIT_TIMER:
  /* Failed to initialize timer */
CATCH_ERROR_GPIO_CONFIG:
  /* Failed to Configure GPIO Port Pins */
CATCH_ERROR_GPIO_INIT:
  /* Failed to initialize GPIO */
CATCH_ERROR_WDT_INIT:
  /* Failed to configure WDT */
CATCH_ERROR_CLK_INIT:
  /* Failed to configure Clocks */
  return err;
}
