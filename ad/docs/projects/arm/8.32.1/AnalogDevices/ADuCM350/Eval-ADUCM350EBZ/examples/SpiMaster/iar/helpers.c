/** 
 * \file helpers.c
 * \brief Helper functions for AD module
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */
#include "helpers.h"
#include "gpt.h"
#include "assert.h"
#include "common.h"
#include "gpio.h"
#include "log.h"

HWATCH_AD_RESULT_TYPE Sleep(const ADI_SYS_POWER_MODE sleepMode,
                          bool_t volatile * const pbLowPowerExitFlag )
{
  HWATCH_AD_RESULT_TYPE err;

  /* the interrupt flag to wake from sleep should be cleared
     prior to enabling any interrupt that may set it
     (otherwise interrupts may be missed) */
  *pbLowPowerExitFlag = false;

  TRY_SET_ERR( SystemEnterLowPowerMode( sleepMode, pbLowPowerExitFlag , 0 ),
               ADI_SYS_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_SLEEP_ENTER,
               CATCH_ERROR_ENTER_SLEEP );

  return HWATCH_AD_RESULT_SUCCESS;

CATCH_ERROR_ENTER_SLEEP:
  /* Failed to enter low power mode */
  return err;
}

static uint16_t calcTimerCountValue(float timeout_sec)
{
  /*
    M = max count of timer counter (2^k full scale or user specified value < 2^k for k-bit timer counter )
    f = chosen base clock frequency
    d = clock frequency pre-scaler or divider
    T = desired time period of timer

    User needs to choose M, f and d to achieve desired period T of the timer
    now, T = M*d/f, such that 1 <= M <= 2^k (full scale value)

    => 1 <= T*f/d <= 2^k
    => d/f <= T <= 2^k * d/f

    f = 32 KHz
      d = 1      => T [31.25 us, 2.048 sec]  resolution = d/f = 31.25 us
      d = 4      => T [125 us, 8.192 sec]    resolution = d/f = 125 us
      d = 16     => T [500 us, 32.768 sec ]  resolution = d/f = 500 us
      d = 256    => T [8 ms, 524.288 sec ]   resolution = d/f = 8 ms
      d = 32768  => T [1.024 sec, 67108.864 sec] resolution = d/f = 1.024 sec

    f = 16 MHz
      d = 1      => T [62.5 ns, 4.096 ms]    resolution = d/f = 62.5 ns
      d = 4      => T [250 ns, 16.384 ms]    resolution = d/f = 250 ns
      d = 16     => T [1 us, 65.536 ms ]     resolution = d/f = 1 us
      d = 256    => T [16 us, 1.048576 sec ]   resolution = d/f = 16 us
      d = 32768  => T [2.048 ms, 134.217728 sec] resolution = d/f = 2.048 ms

      As BLE will try to pair for 30 sec, AD must wait atleast 30 sec.
      T = 35 sec.

      As we require delays of msec upto 120+ sec, we can use: f=32, d =256 or f=16 MHz, d = 32768
      we use system clock with f = 32 KHz, d= 256
  */
  const float clockFrequency = (float) (1<<15);// f = 32768 Hz ~ 32 KHz
  const float prescaler = (float) (1<<8);     // d = 256
  const float maxTime = (float)UINT16_MAX * prescaler/clockFrequency;

  assert( timeout_sec <=  maxTime );
  uint32_t counterValue = (uint32_t) (timeout_sec * clockFrequency/prescaler); // counter value

  // Make sure the counter value is less than full scale value of the timer counter (2^16-1 = 65536-1= 65535)
  assert( counterValue <= UINT16_MAX );

  return (uint16_t)counterValue;
}

HWATCH_AD_RESULT_TYPE StartTimer( ADI_GPT_HANDLE * const pTimer,
                                const IRQn_Type timerIRQn,
                                const float timeout_sec,
                                const bool_t sleepAfter,
                                const ADI_SYS_POWER_MODE sleepMode,
                                bool_t volatile * pbLowPowerExitFlag )
{
  HWATCH_AD_RESULT_TYPE err;

  uint16_t timerCounterValue = calcTimerCountValue(timeout_sec);

  /* Load the counter value corresponding to required time out */
  TRY_SET_ERR( adi_GPT_SetLdVal(*pTimer, timerCounterValue ),
               ADI_GPT_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_TIMER_LD_VAL,
               CATCH_ERROR_TIMER_LD_VAL );

  /* Enable the timer */
  TRY_SET_ERR( EnableTimer( pTimer, timerIRQn ),
               HWATCH_AD_RESULT_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_TIMER_ENABLE,
               CATCH_ERROR_TIMER_ENABLE );

  if( true  == sleepAfter )
  {
    /* Sleep until timeout */
    *pbLowPowerExitFlag = false;
    TRY_SET_ERR( SystemEnterLowPowerMode( sleepMode, pbLowPowerExitFlag, 0 ),
                 ADI_SYS_SUCCESS,
                 err,
                 HWATCH_AD_RESULT_ERROR_SLEEP_ENTER,
                 CATCH_ERROR_ENTER_SLEEP );
  }

  return HWATCH_AD_RESULT_SUCCESS;

/* Cleanup */
CATCH_ERROR_ENTER_SLEEP:
  /* Failed to enter low power mode */

CATCH_ERROR_TIMER_ENABLE:
  /* Failed to enable the timer */

CATCH_ERROR_TIMER_LD_VAL:
  /* Failed to load timer with count value */

  return err;
}

HWATCH_AD_RESULT_TYPE EnableTimer( ADI_GPT_HANDLE * const pTimer,
                                 const IRQn_Type timerIRQn )
{
  HWATCH_AD_RESULT_TYPE err;

  /* Enable timer interrupt in NVIC */
  NVIC_EnableIRQ( timerIRQn );

  /* Start the timer */
  TRY_SET_ERR( adi_GPT_SetTimerEnable(*pTimer, true),
               ADI_GPT_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_TIMER_START,
               CATCH_ERROR_TIMER_START );

  return HWATCH_AD_RESULT_SUCCESS;

/* Cleanup */
CATCH_ERROR_TIMER_START:
  /* Failed to start the timer */
  return err;
}

 HWATCH_AD_RESULT_TYPE StopTimer( ADI_GPT_HANDLE * const pTimer,
                                const IRQn_Type timerIRQn )
{
  HWATCH_AD_RESULT_TYPE err;

  /* Disable timer interrupt in NVIC */
  NVIC_DisableIRQ( timerIRQn );

  /* Stop the timer */
  TRY_SET_ERR( adi_GPT_SetTimerEnable(*pTimer, false),
               ADI_GPT_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_TIMER_STOP,
               CATCH_ERROR_TIMER_STOP );

  return HWATCH_AD_RESULT_SUCCESS;

/* Cleanup */
CATCH_ERROR_TIMER_STOP:
  /* Failed to stop the timer */
  return err;
}

HWATCH_AD_RESULT_TYPE ResetTimer( ADI_GPT_HANDLE * const pTimer,
                                const IRQn_Type timerIRQn,
                                bool_t volatile * const pbTimerExpiredFlag )
{
  HWATCH_AD_RESULT_TYPE err;

  if( NULL != pbTimerExpiredFlag )
  {
    *pbTimerExpiredFlag = false;
  }

  /* Disable timer interrupt in NVIC */
  NVIC_DisableIRQ( timerIRQn );

  /* Reset the timer */
  TRY_SET_ERR( adi_GPT_ResetTimer(*pTimer),
               ADI_GPT_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_TIMER_RESET,
               CATCH_ERROR_TIMER_RESET );

  return HWATCH_AD_RESULT_SUCCESS;

/* Cleanup */
CATCH_ERROR_TIMER_RESET:
  /* Failed to reset the timer */
  return err;
}

HWATCH_AD_RESULT_TYPE EnableBtnIrq(void)
{
  HWATCH_AD_RESULT_TYPE err;
  ADI_GPIO_CONFIG_TYPE ConfigReg;

  /* Enable external interrupt on button port */
  TRY_SET_ERR( adi_GPIO_EnableIRQ(HWATCH_AD_BTN_EINT_IRQn, HWATCH_AD_BTN_IRQ_TRIG_EDGE),
               ADI_GPIO_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_IRQ_ENABLE,
               CATCH_ERROR_IRQ_ENABLE );

  /* Only needed for group A/B type interrupts are used and not for external interrupts */
  // ADI_ENABLE_INT( HWATCH_AD_BTN_EINT_IRQn );

  /* verify that the interrupt was enabled by reading appropriate bits in
    > Bits [2:0] in External Interrupt Configuration 0 Register (EI2CFG)
      corresponding to the external interrupt 8 mode registers (IRQ8MDE)
    > Bits [10:8] in External Interrupt Configuration 0 Register (E10CFG)
      corresponding to the external interrupt 2 mode register (IRQ2MDE)
  */
  ConfigReg = pADI_IDU->HWATCH_AD_BTN_EICFG;
  ConfigReg >>= HWATCH_AD_BTN_EICFG_SHIFT_CNT;

  HWATCH_LOG_DEBUG("AD: Enabled Button Wakeup Interrupt.");
  return HWATCH_AD_RESULT_SUCCESS;

CATCH_ERROR_IRQ_ENABLE:
  /* Failed to Enable IRQ */
  return err;
}

HWATCH_AD_RESULT_TYPE DisableBtnIrq(void)
{
  HWATCH_AD_RESULT_TYPE err;

  /* Disable external interrupt on button port */
  TRY_SET_ERR( adi_GPIO_DisableIRQ(HWATCH_AD_BTN_EINT_IRQn),
               ADI_GPIO_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_IRQ_DISABLE,
               CATCH_ERROR_IRQ_DISABLE );

  HWATCH_LOG_DEBUG("AD: Disabled Button Wakeup interrupt.");

  return HWATCH_AD_RESULT_SUCCESS;

CATCH_ERROR_IRQ_DISABLE:
  /* Failed to Disable IRQ */
  return err;
}

void EnableBleAckIrq(void)
{
  /* Enable group interrupt on BLE ACK port pin (SPI0 MISO)*/
  NVIC_EnableIRQ( HWATCH_AD_BLE_ACK_IRQn );
  HWATCH_LOG_DEBUG("AD: Enabled ACK Input Interrupt from BLE.");
}

void DisableBleAckIrq(void)
{
  /* Disable group interrupt on BLE ACK port pin (SPI0 MISO)*/
  NVIC_DisableIRQ( HWATCH_AD_BLE_ACK_IRQn );
  HWATCH_LOG_DEBUG("AD: Disabled ACK Input Interrupt from BLE.");
}

void ClearBleAckPendingIrq(void)
{
  NVIC_ClearPendingIRQ( HWATCH_AD_BLE_ACK_IRQn );
  HWATCH_LOG_DEBUG("AD: Cleared pending ACK IRQ from BLE.");
}

void SetBleAckIrqPriority( uint32_t const priority )
{
  NVIC_SetPriority( HWATCH_AD_BLE_ACK_IRQn, priority );
  HWATCH_LOG_DEBUG("AD: Set IRQ priority of ACK from BLE to %u.", priority);
}

void AssertWakeUpToBle(void)
{
  adi_GPIO_SetLow(HWATCH_AD_BLE_WAKEUP_PORT, HWATCH_AD_BLE_WAKEUP_PIN);
  HWATCH_LOG_DEBUG("AD: Assert wake up signal to BLE");
}

void DeAssertWakeUpToBle(void)
{
  adi_GPIO_SetHigh(HWATCH_AD_BLE_WAKEUP_PORT, HWATCH_AD_BLE_WAKEUP_PIN);
  HWATCH_LOG_DEBUG("AD: Deassert wake up signal to BLE");
}
