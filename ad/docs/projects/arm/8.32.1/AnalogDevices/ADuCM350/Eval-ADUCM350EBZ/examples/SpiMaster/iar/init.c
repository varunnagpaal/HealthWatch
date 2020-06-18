/** 
 * \file init.c
 * \brief Hardware initializing functions for AD module
 *
 * \author Varun Nagpal
 *
 * \date Oct, 14 2019
 *
 * \version 0.9
 *
 */
#include "init.h"
#include "system.h"
#include "wdt.h"
#include "gpt.h"
#include <assert.h>
#include "test_common.h"
#include "pin_mux.h"
#include "helpers.h"
#include "log.h"

/* Handler(s) for SPI device(s) */
ADI_SPI_DEV_HANDLE hSpiDev0;

/* Handler for WDT */
ADI_WDT_DEV_HANDLE hWatchdog;

/* Handler for General Purpose Timer(s) */
ADI_GPT_HANDLE hTimer1;

/* The below flags that are modified by the ISR's.
   The flags indicate which event has occured.
   Once the control returns to main, event handler
   code is executed depending upon which flag was set.
   The flags are declared volatile as they can be modified
   by both ISR and main code. R/W of these flags from main
   code requires that interrupts must be disabled before doing so.
*/
volatile bool_t bLongBtnIntFlag = false;
volatile bool_t bShortBtnIntFlag = false;
volatile bool_t bLowPowerExitFlag = false;
volatile bool_t bTimer1ExpiredFlag = false;
volatile bool_t bSleepWhileTimedWaiting = false;
volatile bool_t bBleRespondedFlag = false;
volatile bool_t bBlePaired = false;
volatile bool_t bBleWokeup = false;

HWATCH_AD_RESULT_TYPE InitClks(void)
{
  HWATCH_AD_RESULT_TYPE err;

  /* Enables internal LFOSC and HFOSC.
     Uses HFOSC as input source for SPLL
     Disable external HF crystal(hfxctal) but doesn't disable LF external(LFXTAL)
  */
  SystemInit();

  /* Upon reset, the default divisor is HCLKDIVCNT = PCLKDIVCNT = 0x10 = 16. So we should see clocks as 16MHz/16 = 1 Mhz */
  HWATCH_LOG_DEBUG("AD: Upon SystemInit SPIH/CORE/BUS/FCLK/HCLK_CT/ACLK/CT/AFE Clock Frequency (Hz): %u", SystemGetClockFrequency(ADI_SYS_CLOCK_CORE));
  HWATCH_LOG_DEBUG("AD: Upon SystemInit PCLK/I2C/UART/I2S/SPI0/SPI1 Clock Frequency (Hz): %u", SystemGetClockFrequency(ADI_SYS_CLOCK_SPI0));

  /* Set Core clock (HCLK_Core) and Peripheral clock (PCLK) with HFOSC as source, to 16 MHz
     PCLK maybe used as clock source for timers

     Make sure: HWATCH_CORE_CLK_DIV <= HWATCH_SPI0_CLK_DIV
  */
  TRY_SET_ERR( SetSystemClockDivider(ADI_SYS_CLOCK_CORE, HWATCH_AD_CORE_CLK_DIV),
               ADI_SYS_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_CLK_DIV,
               CATCH_ERROR_CORE_CLK_DIV );
  HWATCH_LOG_DEBUG("AD: Upon Core Clk division SPIH/CORE/BUS/FCLK/HCLK_CT/ACLK/CT/AFE Clock Frequency (Hz): %u", SystemGetClockFrequency(ADI_SYS_CLOCK_CORE));

//  TRY_SET_ERR( SetSystemClockDivider(ADI_SYS_CLOCK_SPI0, HWATCH_AD_SPI0_CLK_DIV),
//              ADI_SYS_SUCCESS,
//              err,
//              HWATCH_AD_RESULT_ERROR_CLK_DIV,
//              CATCH_ERROR_SPI0_CLK_DIV );
//  HWATCH_LOG_DEBUG("AD: PCLK/I2C/UART/I2S/SPI0/SPI1 Clock Frequency (Hz): %u", SystemGetClockFrequency(ADI_SYS_CLOCK_SPI0));

  /* Turn on external 32 KHz LFXTAL which may be used for timers */
  TRY_SET_ERR( SystemEnableClockSource(ADI_SYS_CLOCK_SOURCE_LFXTAL, true),
               ADI_SYS_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_CLK_SRC_EN,
               CATCH_ERROR_CLK_SRC_EN );

  /* Wait for LFXTAL to stabilize */
  while (!(pADI_SYSCLK->CLKSTAT0 & BITM_SYSCLK_CLKSTAT0_LFXTALSTATUS));

  return HWATCH_AD_RESULT_SUCCESS;

/* Cleanup */
CATCH_ERROR_CLK_SRC_EN:
  /* Error failed to enable a clock source */
//CATCH_ERROR_SPI0_CLK_DIV:
    /* Error dividing the PCLK/SPI0 clock */
CATCH_ERROR_CORE_CLK_DIV:
  /* Error clock dividing the core clock */
  return err;
}

HWATCH_AD_RESULT_TYPE UnInitClks(void)
{
  /* TBU */
  return HWATCH_AD_RESULT_SUCCESS;
}

HWATCH_AD_RESULT_TYPE InitGpio(void)
{
  HWATCH_AD_RESULT_TYPE err;

  TRY_SET_ERR( adi_GPIO_Init(),
               ADI_GPIO_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_GPIO_INIT,
               CATCH_ERROR_GPIO_INIT );

  return HWATCH_AD_RESULT_SUCCESS;

/* Cleanup */
CATCH_ERROR_GPIO_INIT:
  /* GPIO Initialization Failed */
  return err;
}

HWATCH_AD_RESULT_TYPE UnInitGpio(void)
{
  HWATCH_AD_RESULT_TYPE err;

  TRY_SET_ERR( adi_GPIO_UnInit(),
               ADI_GPIO_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_GPIO_UNINIT,
               CATCH_ERROR_GPIO_UNINIT );

  return HWATCH_AD_RESULT_SUCCESS;

CATCH_ERROR_GPIO_UNINIT:
  /* Failed to uninitialize GPIO */

  return err;
}

/*
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
  are later used in main loop to actually handle the corresponding event(s).
*/
static void IsrExtIntPushButton(void *pCBParam,
                                uint32_t interruptEventIRQ,
                                void *pArg )
{
  IRQn_Type currentIRQ = (IRQn_Type) interruptEventIRQ;
  ADI_GPIO_DATA_TYPE portValue = 0;

  /* Exit low power mode */
  SystemExitLowPowerMode(&bLowPowerExitFlag);

  /* Clear the interrupt */
  adi_GPIO_ClearIRQ(currentIRQ);

  /* Delay to allow GPIO value to settle (debounce) */
  for (uint16_t delay = 0; delay < 0x02FF; delay++);

  /* Read the GPIO Port value */
  portValue = adi_GPIO_GetData( HWATCH_AD_BTN_PORT );

  switch(currentIRQ)
  {
    case HWATCH_AD_BTN_EINT_IRQn:

      /* Check for long or short button press.
        For the time being, we are only testing for short button press.
        Detecting long button press will require use of a timer. */
      if( !HWATCH_CHECK_BIT_MSK( portValue, HWATCH_AD_BTN_PIN ) )
      {
        bShortBtnIntFlag = true;
      }

      break;

    default:
      FAIL("Unexpected interrupt event caused false triggering of IsrExtIntPushButton");
      break;
  }
}

/*  ISR for handling ACK(active low) from nRF BLE to AD on the SPI MISO port pin (P3.1) */
static void IsrBleAck(void *pCBParam,
                      uint32_t interruptEventIRQ,
                      void *pArg )
{
  IRQn_Type currentIRQ = (IRQn_Type) interruptEventIRQ;

  /* Exit low power mode if happened to have entered into it */
  if( true == bSleepWhileTimedWaiting )
  {
    SystemExitLowPowerMode(&bLowPowerExitFlag);
  }

  /* Clear pending group interrupts */
  ClearBleAckPendingIrq();

  /* Disable group interrupts to prevent retriggering*/
  DisableBleAckIrq();

  /* Make sure we didn't forget to reset BLEs last response */
  assert( false == bBleRespondedFlag );

  switch( currentIRQ )
  {
    case GPIOA_IRQn:
      bBleRespondedFlag = true;
      break;

  default:
      FAIL("IsrBleAck: unexpected GPIO interrupt received by ADI. Expected to get Group A interrupt");
      break;
  }
}

HWATCH_AD_RESULT_TYPE ConfigureGpio(void)
{
  HWATCH_AD_RESULT_TYPE err;

  /* As initially SPI0 won't be used so change SPI0 pins to GPIO's
     so that SPI MOSI pin (P3.2) can be used as output GPIO pin
     to wakeup(Interrupt) the nRF BLE from sleep and SPI MISO pin (P3.1)
     can be used as ACK(active low)from nRF BLE to AD
     (say when BLE wants to confirm pairing with phone)
  */
  adi_initpinmux_clearSPI0();

  /* Disable input drivers on the SPI MOSI port pin (P3.2) that wakes up nRF BLE */
  TRY_SET_ERR( adi_GPIO_SetInputEnable(HWATCH_AD_BLE_WAKEUP_PORT, HWATCH_AD_BLE_WAKEUP_PIN, false),
              ADI_GPIO_SUCCESS,
              err,
              HWATCH_AD_RESULT_ERROR_GPIO_GENERIC,
              CATCH_ERROR_WKUP_PIN_1 );

  /*  Enable internal hardwired pull up on the SPI MOSI port pin (P3.2) that wakes up nRF BLE */
  TRY_SET_ERR( adi_GPIO_SetPullUpEnable(HWATCH_AD_BLE_WAKEUP_PORT, HWATCH_AD_BLE_WAKEUP_PIN, true),
               ADI_GPIO_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_GPIO_GENERIC,
               CATCH_ERROR_WKUP_PIN_3 );

  /* Before enabling the output driver on the SPI MOSI port pin (P3.2) that wakes up nRF BLE,
     make sure the output wakeup port pin is set to an inactive state (high) i.e. deasserted. 
     This is required because the state of this output wakeup pin is unknown at the moment
     and enabling output driver with an unknown state may accidentally send an unintended
     wakeup signal (active low) to the BLE. This could accidentally wakeup the BLE iff the
     BLE has managed to initialize its GPIO/ISRs and went into deep sleep before ADI has
     reached this point. So to prevent this, we put this pin to inactive state before enabling it.
  */
  adi_GPIO_SetHigh(HWATCH_AD_BLE_WAKEUP_PORT, HWATCH_AD_BLE_WAKEUP_PIN);

  /* Enable output drivers on the SPI MOSI port pin (P3.2) that wakes up nRF BLE */
  TRY_SET_ERR( adi_GPIO_SetOutputEnable(HWATCH_AD_BLE_WAKEUP_PORT, HWATCH_AD_BLE_WAKEUP_PIN, true),
              ADI_GPIO_SUCCESS,
              err,
              HWATCH_AD_RESULT_ERROR_GPIO_GENERIC,
              CATCH_ERROR_WKUP_PIN_2 );

  /* Enable input driver on the SPI MISO port pin (P3.1) used as ACK(active low) from nRF BLE to AD */
  TRY_SET_ERR( adi_GPIO_SetInputEnable(HWATCH_AD_BLE_ACK_PORT, HWATCH_AD_BLE_ACK_PIN, true),
             ADI_GPIO_SUCCESS,
             err,
             HWATCH_AD_RESULT_ERROR_GPIO_GENERIC,
             CATCH_ERROR_ACK_PIN_1 );

  /* Disable output driver on the SPI MISO port pin (P3.1) used as ACK(active low) from nRF BLE to AD */
  TRY_SET_ERR( adi_GPIO_SetOutputEnable(HWATCH_AD_BLE_ACK_PORT, HWATCH_AD_BLE_ACK_PIN, false),
             ADI_GPIO_SUCCESS,
             err,
             HWATCH_AD_RESULT_ERROR_GPIO_GENERIC,
             CATCH_ERROR_ACK_PIN_2 );

  /* Enable internal hardwired pull up on the SPI MISO port pin (P3.1) used as ACK(active low) from nRF BLE to AD */
  TRY_SET_ERR( adi_GPIO_SetPullUpEnable(HWATCH_AD_BLE_ACK_PORT, HWATCH_AD_BLE_ACK_PIN, true),
              ADI_GPIO_SUCCESS,
              err,
              HWATCH_AD_RESULT_ERROR_GPIO_GENERIC,
              CATCH_ERROR_ACK_PIN_3 );

  /* Map internal SPI0 MISO port pin used as ACK(active low) to Group A Interrupt */
  TRY_SET_ERR( adi_GPIO_RegisterCallback( GPIOA_IRQn, IsrBleAck, NULL ),
               ADI_GPIO_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_ISR_REG,
               CATCH_ERROR_SPI0_MISO_ISR_REG );

  /* Configure expected interrupt polarity of SPI0 MISO ACK port pin as active low
    (only one edge or the other avaliable here; not both)
  */
  TRY_SET_ERR( adi_GPIO_SetGroupInterruptPolarity(HWATCH_AD_BLE_ACK_PORT, ~HWATCH_AD_BLE_ACK_PIN),
               ADI_GPIO_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_IRQ_TRIG_EDGE,
               CATCH_ERROR_SPI0_MISO_ISR_TRIG_EDGE );

  /* Set the SPI0 MISO ACK port pin's Group A interrupt pin mask */
  TRY_SET_ERR( adi_GPIO_SetGroupInterruptPins(HWATCH_AD_BLE_ACK_PORT, GPIOA_IRQn, HWATCH_AD_BLE_ACK_PIN),
               ADI_GPIO_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_IRQ_PIN_MASK_SET,
               CATCH_ERROR_SPI0_MISO_ISR_PIN_MASK_SET );

  /* Enable input drivers on the button port pin */
  TRY_SET_ERR( adi_GPIO_SetInputEnable(HWATCH_AD_BTN_PORT, HWATCH_AD_BTN_PIN, true),
               ADI_GPIO_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_GPIO_GENERIC,
               CATCH_ERROR_BTN_PIN_1 );

  /* Disable output drivers on the button port pin */
  TRY_SET_ERR( adi_GPIO_SetOutputEnable(HWATCH_AD_BTN_PORT, HWATCH_AD_BTN_PIN, false),
               ADI_GPIO_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_GPIO_GENERIC,
               CATCH_ERROR_BTN_PIN_2 );

  /*  Enable internal hardwired pull up on the button port pin */
  TRY_SET_ERR( adi_GPIO_SetPullUpEnable(HWATCH_AD_BTN_PORT, HWATCH_AD_BTN_PIN, true),
               ADI_GPIO_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_GPIO_GENERIC,
               CATCH_ERROR_BTN_PIN_3 );

  /* Setup ISR when button is pushed */
  TRY_SET_ERR( adi_GPIO_RegisterCallback(HWATCH_AD_BTN_EINT_IRQn, IsrExtIntPushButton, NULL),
              ADI_GPIO_SUCCESS,
              err,
              HWATCH_AD_RESULT_ERROR_ISR_REG,
              CATCH_ERROR_BTN_ISR_REG );

  /* Disable input driver on the LED A port pin */
  TRY_SET_ERR( adi_GPIO_SetInputEnable(HWATCH_AD_LED_A_PORT, HWATCH_AD_LED_A_PIN, false),
              ADI_GPIO_SUCCESS,
              err,
              HWATCH_AD_RESULT_ERROR_GPIO_GENERIC,
              CATCH_ERROR_LED_A_PIN_1 );

  /* Enable output driver on the LED A port pin */
  TRY_SET_ERR( adi_GPIO_SetOutputEnable(HWATCH_AD_LED_A_PORT, HWATCH_AD_LED_A_PIN, true),
              ADI_GPIO_SUCCESS,
              err,
              HWATCH_AD_RESULT_ERROR_GPIO_GENERIC,
              CATCH_ERROR_LED_A_PIN_2 );

  /*  Enable internal hardwired pull down on the LED A port pin */
  TRY_SET_ERR( adi_GPIO_SetPullUpEnable(HWATCH_AD_LED_A_PORT, HWATCH_AD_LED_A_PIN, true),
              ADI_GPIO_SUCCESS,
              err,
              HWATCH_AD_RESULT_ERROR_GPIO_GENERIC,
              CATCH_ERROR_LED_A_PIN_3 );

  /* Disable input driver on the LED B port pin */
  TRY_SET_ERR( adi_GPIO_SetInputEnable(HWATCH_AD_LED_B_PORT, HWATCH_AD_LED_B_PIN, false),
              ADI_GPIO_SUCCESS,
              err,
              HWATCH_AD_RESULT_ERROR_GPIO_GENERIC,
              CATCH_ERROR_LED_B_PIN_1 );

  /* Enable output driver on the LED B port pin */
  TRY_SET_ERR( adi_GPIO_SetOutputEnable(HWATCH_AD_LED_B_PORT, HWATCH_AD_LED_B_PIN, true),
              ADI_GPIO_SUCCESS,
              err,
              HWATCH_AD_RESULT_ERROR_GPIO_GENERIC,
              CATCH_ERROR_LED_B_PIN_2 );

  /*  As internally pull down is hardwired on the LED B port pin, we dont configure it
      as LED needs to be pulled up to turn it on */

  /* Just before entering the ISR, the CPU first disables the
     corresponding external interrupt flags (CPU and peripheral interrupt flags)
     to prevent the ISR from being interrupted while in middle of execution.
     Note that this maybe done by CPU or compiler automatically before
     ISR code is entered */

  /*1: Short button press: IsrWakeUpFromSleep */

  /*2: Long button press: IsrPowerUpDown */

  /*3: Clear any pending IRQs and Enable external interrupt */

  /* Upon exit from ISR, the corresponding extrnal interrupt flags and low power mode
     are reenabled automatically by CPU because upon exit from ISR, the PC
     and other status registers (with enabled external interrupt flags and low power mode)
     were pushed onto the stack before entering the ISR. Exiting the ISR will pop
     the stack thereby restoring the status registers (enabling external interrupt and, low power
     mode)
  */

  /* Install the CommonInterruptHandler for button port interrupt.
     Only needed for group A/B type interrupts are used and not for external interrupts
 */
  // ADI_INSTALL_HANDLER(HWATCH_AD_BTN_EINT_IRQn, CommonInterruptHandler);

  /* Configure pins
   For SPI device 0 (IRQ source 15):
     CS - P3.3 (F10)(active low)
     SCLK- P3.0 (F14)
     MISO - P3.1 (G14)
     MOSI - P3.2 (F15)
   Note: Interrupt source (IRQ source 15) for SPI0 can wake up for core sleep only
         and not from sys_sleep or hibernate so it is not useful.

   For push button :
     short press- negative edge triggered interrupt
     long press - negative level triggered interrupt
     P0.1/CAPT_B (J15) (active low)(vector: external interrupt 2, IRQ source 3)
     can wake up from core/sys/hibernate sleep. note this pin is not available
     on breakout board. So instead, consider using either of the following port
     pins instead which are also external interrupts  i.e allow to wake up CPU
     from core/sys/hibernate sleep:
       - P4.0 (L1)(pull up)(ext int 0)(digital header 2)(J5-4)(vector: external interrupt 0, IRQ source 1). Shared with I2C as clock
       - or P3.4 (ext int 8)(digital header 1)(J20-41)(vector: external interrupt 7, IRQ source 8) - used by spi1. cant use it
       there is already s4 switch on pin p3.4 on eval-adu350 board
       - or P0.10 (R2)(pull up)(ext int 8)(digital header 2)(J5-7)(vector: external interrupt 8, IRQ source 9). Shared with Timer C as output.
         There is already a push button switch s2 and test point T9 on P0.10 pin on the eval-aducm350 board.
         So we use P0.10 (s2) instead.
  */

  return HWATCH_AD_RESULT_SUCCESS;

CATCH_ERROR_BTN_ISR_REG:
  /* Failed to register function as Isr (callback) */
CATCH_ERROR_LED_B_PIN_2:
CATCH_ERROR_LED_B_PIN_1:

CATCH_ERROR_LED_A_PIN_3:
CATCH_ERROR_LED_A_PIN_2:
CATCH_ERROR_LED_A_PIN_1:

CATCH_ERROR_BTN_PIN_3:
  /* Failed to enable internal pull up */
CATCH_ERROR_BTN_PIN_2:
  /* Failed to disable output driver */
CATCH_ERROR_BTN_PIN_1:
  /* Failed to enable input driver */

CATCH_ERROR_SPI0_MISO_ISR_PIN_MASK_SET:
CATCH_ERROR_SPI0_MISO_ISR_TRIG_EDGE:
CATCH_ERROR_SPI0_MISO_ISR_REG:

CATCH_ERROR_ACK_PIN_3:
  /* Failed to enable internal pull up */
CATCH_ERROR_ACK_PIN_2:
  /* Failed to disable output driver */
CATCH_ERROR_ACK_PIN_1:
  /* Failed to enable input driver */

CATCH_ERROR_WKUP_PIN_3:
  /* Failed to enable internal pull up */
CATCH_ERROR_WKUP_PIN_2:
  /* Failed to enable output driver */
CATCH_ERROR_WKUP_PIN_1:
  /* Failed to disable input driver */
  return err;
}

HWATCH_AD_RESULT_TYPE InitWatchDogTimer(void)
{
  HWATCH_AD_RESULT_TYPE err;

  /* Intialize WDT device */
  TRY_SET_ERR( adi_WDT_Init(ADI_WDT_DEVID_0, &hWatchdog),
               ADI_WDT_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_WDT_INIT,
               CATCH_ERROR_WDT_INIT );

  /* Disable WDT */
  TRY_SET_ERR( adi_WDT_SetEnable(hWatchdog, false),
               ADI_WDT_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_WDT_DISABLE,
               CATCH_ERROR_WDT_DISABLE );

  /* Unitialize WDT device */
  TRY_SET_ERR( adi_WDT_UnInit(hWatchdog),
               ADI_WDT_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_WDT_UNINIT,
               CATCH_ERROR_WDT_UNINIT_1 );

  return HWATCH_AD_RESULT_SUCCESS;

/* Cleanup */
CATCH_ERROR_WDT_UNINIT_1:
  /* Failed to Uninitialize WDT */

CATCH_ERROR_WDT_DISABLE:
  /* Failed to  Disable WDT. Unitialize WDT */
  TRY_SET_ERR( adi_WDT_UnInit(hWatchdog),
               ADI_WDT_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_WDT_UNINIT,
               CATCH_ERROR_WDT_UNINIT_2 );

CATCH_ERROR_WDT_UNINIT_2:
  /* Failed to Uninitialize WDT */

CATCH_ERROR_WDT_INIT:
  /* Failed to Initialize WDT */

  return err;
}

HWATCH_AD_RESULT_TYPE UnInitWatchDogTimer(void)
{
  HWATCH_AD_RESULT_TYPE err;

  TRY_SET_ERR( adi_WDT_UnInit(hWatchdog),
               ADI_WDT_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_WDT_UNINIT,
               CATCH_ERROR_WDT_UNINIT );

  return HWATCH_AD_RESULT_SUCCESS;

/* Cleanup */
CATCH_ERROR_WDT_UNINIT:
  /* Failed to UnInitialize WDT */

  return err;
}

static void IsrOnTimer1_Expiry(void *pCBParam,
                               uint32_t interruptEvent,
                               void *interruptEventArg )
{
  ADI_GPT_EVENT_TYPE gptEvent = (ADI_GPT_EVENT_TYPE)interruptEvent;

  switch( gptEvent )
  {
    case ADI_GPT_EVENT_TIMEOUT:

      /* Exit low power mode */
      /* TBU: Checkflag if timer is with or without sleep */
      if( true == bSleepWhileTimedWaiting )
      {
        SystemExitLowPowerMode(&bLowPowerExitFlag);
      }

      /* Timer timed out */
      bTimer1ExpiredFlag = true;

      /* Stop and reset the timer */
      TRY_FAIL( StopTimer( &hTimer1, HWATCH_AD_TIMER_1_IRQn ),
                HWATCH_AD_RESULT_SUCCESS,
                "Failed to stop the timer" );

      TRY_FAIL( ResetTimer( &hTimer1, HWATCH_AD_TIMER_1_IRQn, NULL ),
                HWATCH_AD_RESULT_SUCCESS,
                "Failed to reset the timer" );
      break;

    default:
      FAIL(" Unexpected timer 1 event");
      break;
  }
}

HWATCH_AD_RESULT_TYPE InitIntervalTimer(void)
{
  HWATCH_AD_RESULT_TYPE err;
  const uint16_t defaultCounterValue = 35 * (1<<24)/(1<<15);

  TRY_SET_ERR( adi_GPT_Init(HWATCH_AD_TIMER_1_DEVID, &hTimer1),
               ADI_GPT_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_TIMER_INIT,
               CATCH_ERROR_TIMER_INIT );

  /* For f = 32 KHz MHz, d= 256 (prescaler),each count is equivalent to
   d/f = 7.8125 ms. To have delay of 35 sec requires count of 35/7.8125e-3 ~= 4480
  */

  /* Select f=32 KHz clock source, preferably system clock source */
  TRY_SET_ERR( adi_GPT_SetClockSelect(hTimer1, HWATCH_AD_TIMER_CLK_32KHZ),
               ADI_GPT_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_TIMER_CLK_SEL,
               CATCH_ERROR_TIMER_CLK_SEL );

  /* Set prescaler value to d=256 */
  TRY_SET_ERR( adi_GPT_SetPrescaler(hTimer1, ADI_GPT_PRESCALER_256),
               ADI_GPT_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_TIMER_PRESCALER,
               CATCH_ERROR_TIMER_PRESCALER );

  /* Set periodic mode */
  TRY_SET_ERR( adi_GPT_SetPeriodicMode(hTimer1, true, defaultCounterValue ),
              ADI_GPT_SUCCESS,
              err,
              HWATCH_AD_RESULT_ERROR_TIMER_MODE,
              CATCH_ERROR_TIMER_MODE );

  /* Count down to 0 */
  TRY_SET_ERR( adi_GPT_SetCountMode(hTimer1, ADI_GPT_COUNT_DOWN),
               ADI_GPT_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_TIMER_CNT_MODE,
               CATCH_ERROR_TIMER_CNT_MODE );

  /* Register callback function when timeout event occurs */
  TRY_SET_ERR( adi_GPT_RegisterCallback(hTimer1, IsrOnTimer1_Expiry, NULL),
               ADI_GPT_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_TIMER_CNT_MODE,
               CATCH_ERROR_TIMER_CNT_MODE );

  return HWATCH_AD_RESULT_SUCCESS;

/* Cleanup */
CATCH_ERROR_TIMER_CNT_MODE:
  /* Error setting count mode for GPT */

CATCH_ERROR_TIMER_PRESCALER:
  /* Error setting prescaler for clock of GPT */

CATCH_ERROR_TIMER_CLK_SEL:
  /* Error setting clock for GPT */

CATCH_ERROR_TIMER_MODE:
  /* Error setting mode of GPT */
  adi_GPT_UnInit(hTimer1);

CATCH_ERROR_TIMER_INIT:
  /* Error initializing GPT */

  return err;
}

HWATCH_AD_RESULT_TYPE UnInitIntervalTimer(void)
{
  HWATCH_AD_RESULT_TYPE err;

  TRY_SET_ERR( adi_GPT_UnInit(hTimer1),
               ADI_GPT_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_TIMER_UNINIT,
               CATCH_ERROR_TIMER_UNINIT );

  return HWATCH_AD_RESULT_SUCCESS;

CATCH_ERROR_TIMER_UNINIT:
  /* Error unitializing GPT */

  return err;
}

HWATCH_AD_RESULT_TYPE InitSpi( ADI_SPI_DEV_ID_TYPE const devId,
                             ADI_SPI_DEV_HANDLE * const phSpiDev,
                             const HWATCH_SPI_MODE_TYPE spiMode,
                             const bool_t bBlockingMode,
                             const bool_t bDmaMode,
                             const bool_t bContinousMode )
{
  HWATCH_AD_RESULT_TYPE err;

#if 0
  const uint32_t bpsArray[] = {HWATCH_SPI_FREQ_1K,
                               HWATCH_SPI_FREQ_2K,
                               HWATCH_SPI_FREQ_4K,
                               HWATCH_SPI_FREQ_5K,
                               HWATCH_SPI_FREQ_10K,
                               HWATCH_SPI_FREQ_20K,
                               HWATCH_SPI_FREQ_40K,
                               HWATCH_SPI_FREQ_50K,
                               HWATCH_SPI_FREQ_100K,
                               HWATCH_SPI_FREQ_200K,
                               HWATCH_SPI_FREQ_400K,
                               HWATCH_SPI_FREQ_500K,
                               HWATCH_SPI_FREQ_1M,
                               HWATCH_SPI_FREQ_2M,
                               HWATCH_SPI_FREQ_4M,
                               HWATCH_SPI_FREQ_8M};
#endif

  /* Initialize SPI Device */
  if( HWATCH_SPI_MODE_MASTER == spiMode )
  {
    TRY_SET_ERR( adi_SPI_MasterInit(devId, phSpiDev),
                 ADI_SPI_SUCCESS,
                 err,
                 HWATCH_AD_RESULT_ERROR_SPI_INIT,
                 CATCH_ERROR_SPI_INIT );
  }
  else if( HWATCH_SPI_MODE_SLAVE == spiMode )
  {
    TRY_SET_ERR( adi_SPI_SlaveInit(devId, phSpiDev),
                 ADI_SPI_SUCCESS,
                 err,
                 HWATCH_AD_RESULT_ERROR_SPI_INIT,
                 CATCH_ERROR_SPI_INIT );
  }

#if 0

  for( int i = 0; i < sizeof(bpsArray)/sizeof(bpsArray[0]); ++i)
  {
    if( ADI_SPI_SUCCESS != adi_SPI_SetBitrate(*phSpiDev, bpsArray[i]) )
      HWATCH_LOG_DEBUG("AD: SPI0 Init FAIL: %u bps", bpsArray[i]);
    else
      HWATCH_LOG_DEBUG("AD: SPI0 Init PASS: %u bps", bpsArray[i]);
  }

  for( uint32_t bps = 0; bps < 100000; bps+=100)
  {
    if( ADI_SPI_SUCCESS != adi_SPI_SetBitrate(*phSpiDev, bps) )
      HWATCH_LOG_DEBUG("AD: SPI0 Init FAIL: %u bps", bps);
  }
#endif

  /* Set Bit rate to something the master/slave device support */
  TRY_SET_ERR( adi_SPI_SetBitrate(*phSpiDev, HWATCH_SPI_BIT_RATE),
               ADI_SPI_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_SPI_SET_BIT_RATE,
               CATCH_ERROR_SET_BIT_RATE );

  /* Set blocking/non-blocking mode */
  TRY_SET_ERR( adi_SPI_SetBlockingMode(*phSpiDev, bBlockingMode),
                ADI_SPI_SUCCESS,
                err,
                HWATCH_AD_RESULT_ERROR_SPI_SET_BLK_MODE,
                CATCH_ERROR_SET_BLK_MODE );

  /* Enable/Disable DMA mode */
  TRY_SET_ERR( adi_SPI_SetDmaMode(*phSpiDev, bDmaMode),
                ADI_SPI_SUCCESS,
                err,
                HWATCH_AD_RESULT_ERROR_SPI_SET_DMA_MODE,
                CATCH_ERROR_SET_DMA_MODE );

  /* Enable/Disable clock stalls between bytes */
  TRY_SET_ERR(adi_SPI_SetContinousMode(*phSpiDev, bContinousMode),
              ADI_SPI_SUCCESS,
              err,
              HWATCH_AD_RESULT_ERROR_SPI_SET_CONT_MODE,
              CATCH_ERROR_SET_CONT_MODE );

  /* Set Clock Polarity */
  TRY_SET_ERR(adi_SPI_SetClockPolarity(*phSpiDev, HWATCH_SPI_CPOL),
              ADI_SPI_SUCCESS,
              err,
              HWATCH_AD_RESULT_ERROR_SPI_SET_CPOL,
              CATCH_ERROR_SET_SPI_CPOL );

  /* Set Clock Phase */
  TRY_SET_ERR(adi_SPI_SetClockPhase(*phSpiDev, HWATCH_SPI_CPHASE),
              ADI_SPI_SUCCESS,
              err,
              HWATCH_AD_RESULT_ERROR_SPI_SET_CPHASE,
              CATCH_ERROR_SET_SPI_CPHASE );

  /* Set if LSB/MSB is tx-rx first */
  TRY_SET_ERR(adi_SPI_SetLsbFirst(*phSpiDev, HWATCH_SPI_BIT_ORDER),
              ADI_SPI_SUCCESS,
              err,
              HWATCH_AD_RESULT_ERROR_SPI_SET_BIT_ORDER,
              CATCH_ERROR_SET_SPI_SET_BIT_ORDER );

  return HWATCH_AD_RESULT_SUCCESS;

/* Cleanup */
CATCH_ERROR_SET_SPI_SET_BIT_ORDER:
  /* Failed to set SPI Bit order */
CATCH_ERROR_SET_SPI_CPHASE:
  /* Failed to set SPI CPHASE */
CATCH_ERROR_SET_SPI_CPOL:
  /* Failed to set SPI CPOL */
CATCH_ERROR_SET_CONT_MODE:
  /* Failed to enable/disable SPI continuous mode */
CATCH_ERROR_SET_DMA_MODE:
  /* Failed to enable/disable SPI DMA mode */
CATCH_ERROR_SET_BLK_MODE:
  /* Failed to enable/disable SPI blocking mode */
CATCH_ERROR_SET_BIT_RATE:
  /* Failed to set SPI Bit rate */
  TRY_SET_ERR( adi_SPI_UnInit(*phSpiDev),
               ADI_SPI_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_SPI_UNINIT,
               CATCH_ERROR_SPI_UNINIT );

CATCH_ERROR_SPI_UNINIT:
 /* Failed to Uninitialize SPI Device 0 */

CATCH_ERROR_SPI_INIT:
  /* Failed to initialize SPI device 0 */

  return err;
}

HWATCH_AD_RESULT_TYPE UnInitSpi( ADI_SPI_DEV_HANDLE * const phSpiDev )
{
  HWATCH_AD_RESULT_TYPE err;

  TRY_SET_ERR( adi_SPI_UnInit(*phSpiDev),
               ADI_SPI_SUCCESS,
               err,
               HWATCH_AD_RESULT_ERROR_SPI_UNINIT,
               CATCH_ERROR_SPI_UNINIT );

  return HWATCH_AD_RESULT_SUCCESS;

/* Cleanup */
CATCH_ERROR_SPI_UNINIT:
  /* Failed to uninitialize SPI device 0 */
  return err;
}
