/**************************************************************************************************
  Filename:       mac_mcu_timer.c
  Revised:        $Date: 2011-09-21 08:29:21 -0700 (Wed, 21 Sep 2011) $
  Revision:       $Revision: 27659 $

  Description:    Describe the purpose and contents of the file.


  Copyright 2006-2011 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

/* hal */
#include "hal_types.h"
#include "hal_defs.h"
#include "hal_mac_cfg.h"
#include "hal_sleep.h"

/* high-level */
#include "mac_spec.h"
#include "mac_pib.h"

/* exported low-level */
#include "mac_low_level.h"

/* low-level specific */
#include "mac_mcu_timer.h"
#include "mac_backoff_timer.h"
#include "mac_tx.h"
#include "mac_rx.h"
#include "mac_rx_onoff.h"

/* radio specific */
#include "mac_radio_defs.h"

/* debug */
#include "mac_assert.h"
#include "hal_board.h"

/* ------------------------------------------------------------------------------------------------
 *                                         Global Variables
 * ------------------------------------------------------------------------------------------------
 */
uint32 macMcuTimerBackoffCount;
uint32 macMcuTimerBackoffCompare;
uint32 macMcuTimerBackoffRollover;
uint8 macMcuTimerRecordMaxRssiFlag;
uint8 macMcuTimerBackoffs;
#ifdef HAL_BOARD_LM3S
  uint16 macMcuTimerLeftoverUsecs;
#else
  uint16 macMcuTimerCount;
#endif
/* snapshot of backoffCount when SFD occurs. */
uint32 macMcuTimerBackoffCountCapture;
/* snapshot of TAR when SFD occurs. */
uint16 macMcuTimerTickCountCapture;
/* SFD falling edge interrupt processing */
uint8 macMcuTimerProcessFallingEdgeSFDSync = 0;
/* Free running counter driven by MAC timer (per higher layer requirement) */
uint32 mcuPrecisionCount = 0;

void (* pFuncMacMcuTimerCallback)(void);

/* ------------------------------------------------------------------------------------------------
 *                                          Local Functions
 * ------------------------------------------------------------------------------------------------
 */
void macMcuTimerConfigureUsecTimer(uint16 usecs); /* intentionally not static so compiler can optimize out */
void macMcuTimerCapture(void);
void macMcuTimerCompare(void);

#ifdef HAL_BOARD_LM3S
void hal_mac_timer_compare_isr_function(void);
void hal_mac_timer_rollover_isr_function(void);
#endif

/**************************************************************************************************
 * @fn          macMcuTimerInit
 *
 * @brief       Initialize the MCU timer functionality.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macMcuTimerInit(void)
{
  macMcuTimerRecordMaxRssiFlag = 0;

#ifdef HAL_BOARD_LM3S

  HAL_MAC_TIMER_A_INIT();
  HAL_MAC_TIMER_SET_ROLLOVER( MAC_RADIO_TIMER_TICKS_PER_BACKOFF() );
  HAL_MAC_TIMER_ROLLOVER_ENABLE_INTERRUPT();
  HAL_MAC_TIMER_START();

  HAL_MAC_TIMER_B_INIT();
  /* Timer B starts when ever it is required */

  HAL_SLEEP_TIMER_INIT();

#else

  HAL_MAC_TIMER_INIT();
  HAL_MAC_TIMER_SET_ROLLOVER( MAC_RADIO_TIMER_TICKS_PER_BACKOFF() );
  HAL_MAC_TIMER_START();

#endif /* HAL_BOARD_LM3S */

  /* Configure Timer_A_1 in input capture mode */
  HAL_MAC_TIMER_CONFIGURE_INPUT_CAPTURE();
}


/**************************************************************************************************
 * @fn          macMcuTimerWaitUsecs
 *
 * @brief       Wait the specified number of microseconds and then return.
 *
 * @param       usecs - microseconds to delay
 *
 * @return      none
 **************************************************************************************************
 */
void macMcuTimerWaitUsecs(uint16 usecs)
{
  MAC_ASSERT(!HAL_MAC_TIMER_COMPARE_INTERRUPT_IS_ENABLED());

  macMcuTimerConfigureUsecTimer(usecs);

#ifdef HAL_BOARD_LM3S

  HAL_MAC_TIMER_COMPARE_DISABLE_INTERRUPT();
  HAL_MAC_TIMER_CLEAR_COMPARE_FLAG();

  HAL_MAC_TIMER_COMPARE_TIMER_START();
  	
  for (;;)
  {
    while(!HAL_MAC_TIMER_COMPARE_FLAG())
    {

    }
    HAL_MAC_TIMER_CLEAR_COMPARE_FLAG();

    if(macMcuTimerLeftoverUsecs)
    {
      macMcuTimerLeftoverUsecs = 0;
      if(macMcuTimerBackoffs != 0)
      {
        HAL_MAC_TIMER_COMPARE_TIMER_STOP();
        HAL_MAC_TIMER_COMPARE_CONFIGURE_CONT();
        HAL_MAC_TIMER_SET_COMPARE(MAC_SPEC_USECS_PER_BACKOFF);
        HAL_MAC_TIMER_CLEAR_COMPARE_FLAG();
        HAL_MAC_TIMER_COMPARE_TIMER_START();	
      }
      else
      {
        HAL_MAC_TIMER_COMPARE_TIMER_STOP();			
        break;
      }
    }
    else
    {
      macMcuTimerBackoffs--;
      if (macMcuTimerBackoffs == 0)
      {
        HAL_MAC_TIMER_COMPARE_TIMER_STOP();			
        break;
      }	
    }

#else

  for (;;)
  {
    if (macMcuTimerBackoffs == 0)
    {
      break;
    }

    while(!HAL_MAC_TIMER_COMPARE_FLAG());
    HAL_MAC_TIMER_CLEAR_COMPARE_FLAG();

    macMcuTimerBackoffs--;

#endif /* HAL_BOARD_LM3S */

  }
}

/**************************************************************************************************
 * @fn          macMcuTimerUsec
 *
 * @brief       Initiate MCU timer to trigger in the specified number of microseconds.
 *              When timer expires the function pointed to by pFuncMacMcuTimerCallback is executed.
 *              Normally accessed via macro supplied in this file's corresponding .h file.
 *
 * @param       usecs - timer expiration in microseconds
 *
 * @return      none
 **************************************************************************************************
 */
void macMcuTimerUsecs(uint16 usecs)
{
  MAC_ASSERT(!HAL_MAC_TIMER_COMPARE_INTERRUPT_IS_ENABLED());
  MAC_ASSERT(usecs > 100); /* must be reasonable value */

  macMcuTimerConfigureUsecTimer(usecs);

#ifdef HAL_BOARD_LM3S

  HAL_MAC_TIMER_CLEAR_COMPARE_FLAG();
  HAL_MAC_TIMER_COMPARE_ENABLE_INTERRUPT();
  HAL_MAC_TIMER_COMPARE_TIMER_START();

#else

  HAL_MAC_TIMER_COMPARE_ENABLE_INTERRUPT();

#endif /* HAL_BOARD_LM3S */
}

/**************************************************************************************************
 * @fn          macMcuTimerConfigureUsecTimer
 *
 * @brief       Configures MCU timer to trigger in the specified number of microseconds.
 *
 * @param       usecs - microseconds to expiration
 *
 * @return      none
 **************************************************************************************************
 */
void macMcuTimerConfigureUsecTimer(uint16 usecs)
{
#ifdef HAL_BOARD_LM3S

  /* calculate number of backoffs and number of intra-backoff microseconds */
  macMcuTimerBackoffs = (usecs / MAC_SPEC_USECS_PER_BACKOFF);
  macMcuTimerLeftoverUsecs = usecs % MAC_SPEC_USECS_PER_BACKOFF;

  if(macMcuTimerLeftoverUsecs != 0)
  {
    HAL_MAC_TIMER_COMPARE_CONFIGURE_ONESHOT();
    HAL_MAC_TIMER_SET_COMPARE(macMcuTimerLeftoverUsecs);
  }
  else
  {
    HAL_MAC_TIMER_COMPARE_CONFIGURE_CONT();
    HAL_MAC_TIMER_SET_COMPARE(MAC_SPEC_USECS_PER_BACKOFF);
  }

#else

  uint16 intraBackoffUsecs;
  uint16 triggerCountNoWrap;
  uint16 currentCountNoWrap;

  /* calculate number of backoffs and number of intra-backoff microseconds */
  macMcuTimerBackoffs = (usecs / MAC_SPEC_USECS_PER_BACKOFF) + 1;
  intraBackoffUsecs = usecs % MAC_SPEC_USECS_PER_BACKOFF;

  /*
   *  Calculate the trigger point to account for the intrabackoff portion of
   *  the delay.  At this point, the value is not "wrapped around" to fit in the
   *  counter.  This is so the value can be directly compared to the current
   *  count which will be read and normalized to be "non wrap around."
   */
  triggerCountNoWrap = macMcuTimerCount + (intraBackoffUsecs * (MAC_RADIO_TIMER_TICKS_PER_BACKOFF() / MAC_SPEC_USECS_PER_BACKOFF));

  /* set the hardware compare count and immediately clear the compare flag */
  HAL_MAC_TIMER_SET_COMPARE(triggerCountNoWrap % MAC_RADIO_TIMER_TICKS_PER_BACKOFF());
  HAL_MAC_TIMER_CLEAR_COMPARE_FLAG();

  /* record the latest current count and normalize so there is no wraparound */
  currentCountNoWrap = HAL_MAC_TIMER_COUNT();
  if (currentCountNoWrap < macMcuTimerCount)
  {
    currentCountNoWrap += MAC_RADIO_TIMER_TICKS_PER_BACKOFF();
  }

  /*
   *  Catch any missed compare event that may have happened in the time it took
   *  to read the current count and set the trigger point.
   */
  if (!HAL_MAC_TIMER_COMPARE_FLAG() && (currentCountNoWrap >= triggerCountNoWrap))
  {
    macMcuTimerBackoffs--;
  }

#endif /* HAL_BOARD_LM3S */
}

/**************************************************************************************************
 * @fn          macMcuTimerUnslottedBackoffs
 *
 * @brief       Initiate MCU timer to trigger in the specified number of backoffs.
 *              When timer expires the function pointed to by pFuncMacMcuTimerCallback is executed.
 *              Normally accessed via macro supplied in this file's corresponding .h file.
 *
 * @param       backoffs - timer expiration in backoffs
 *
 * @return      none
 **************************************************************************************************
 */
void macMcuTimerUnslottedBackoffs(uint8 backoffs)
{
  MAC_ASSERT(!HAL_MAC_TIMER_COMPARE_INTERRUPT_IS_ENABLED());
  MAC_ASSERT(backoffs != 0); /* must be at least one backoff */

#ifdef HAL_BOARD_LM3S

  macMcuTimerBackoffs = backoffs;
  macMcuTimerLeftoverUsecs = 0;

  HAL_MAC_TIMER_COMPARE_CONFIGURE_CONT();
  HAL_MAC_TIMER_SET_COMPARE(MAC_SPEC_USECS_PER_BACKOFF);

  HAL_MAC_TIMER_CLEAR_COMPARE_FLAG();
  HAL_MAC_TIMER_COMPARE_ENABLE_INTERRUPT();

  HAL_MAC_TIMER_COMPARE_TIMER_START();

#else

  macMcuTimerBackoffs = backoffs;
  HAL_MAC_TIMER_SET_COMPARE(macMcuTimerCount);

  HAL_MAC_TIMER_CLEAR_COMPARE_FLAG();
  HAL_MAC_TIMER_COMPARE_ENABLE_INTERRUPT();

#endif /* HAL_BOARD_LM3S */
}

/**************************************************************************************************
 * @fn          macMcuTimerSlottedBackoffs
 *
 * @brief       Initiate MCU timer to trigger in the specified number of backoffs on rollover.
 *              When timer expires the function pointed to by pFuncMacMcuTimerCallback is executed.
 *              Normally accessed via macro supplied in this file's corresponding .h file.
 *
 * @param       backoffs - timer expiration in backoffs
 *
 * @return      none
 **************************************************************************************************
 */
void macMcuTimerSlottedBackoffs(uint8 backoffs)
{
  MAC_ASSERT(!HAL_MAC_TIMER_COMPARE_INTERRUPT_IS_ENABLED());
  MAC_ASSERT(backoffs != 0); /* must be at least one backoff */

  macMcuTimerBackoffs = backoffs;
  HAL_MAC_TIMER_SET_COMPARE(MAC_RADIO_TIMER_TICKS_PER_BACKOFF()-1);

  HAL_MAC_TIMER_CLEAR_COMPARE_FLAG();
  HAL_MAC_TIMER_COMPARE_ENABLE_INTERRUPT();
}

/**************************************************************************************************
 * @fn          halMacTimerCaptureCompareIsr
 *
 * @brief       -
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
#ifdef HAL_BOARD_LM3S

void hal_mac_timer_compare_isr_function(void)
{
  if(HAL_MAC_TIMER_COMPARE_FLAG())
  {
    HAL_MAC_TIMER_CLEAR_COMPARE_FLAG();
    macMcuTimerCompare();	
  }
}

#else /* !HAL_BOARD_LM3S */

HAL_MAC_TIMER_CAPTURE_COMPARE_ISR_FUNCTION()
{
  uint16 value = HAL_MAC_TAIV();

  /* Reading the TAIV value has a special effect.
     If more than one interrupt is pending (TAIV, TACCR1, TACCR2),
     the highest priority interrupt is cleared on reading the TAIV register.
     As soon as the RETI (return from isr) instruction is executed, the
     next pending interrupt is triggered.

     Hence in the following code, we return immediately upon processing
     each interrupt without checking for the other interrupt flags.
   */
  if (value & TAIV_TACCR1_CCIFG)
  {
    /* RFC_SFD_SYNC signal from radio */

    /* The timer value will be captured on both rising edge as well as the
     * falling edge. But we are only interested in the value at the time
     * of rising edge.
     */
    if(HAL_MAC_TIMER_CAPTURED_INPUT_IS_HIGH())
    {
      /* Since this is a rising edge, capture the tickCount and
       * backoffCount value.
       */
      macMcuTimerCapture();
    }
    else
    {
      /* We must read the value on the falling edge transition (even though we
       * don't care about it). Otherwise an overflow will occur and
       * future capture will not occur.
       */
      HAL_MAC_TIMER_SFD_CAPTURE();

      /* Now, if SFD processing flag is enabled it means we are interested in
       * processing this interrupt. We were waiting for transmission to
       * complete. TBD - the TX done is no longer indicated by SFD. The
       * HAL_MAC_SFD_INT related codes are candidates for removal. But some
       * of the logic may be used by CC2430/CC2530 so leave them alone for now.
       */
      if(HAL_MAC_SFD_INT_IS_ENABLED())
      {
        HAL_MAC_DISABLE_SFD_INT();
      }
    }
    return;
  }

  if (value & TAIV_TACCR2_CCIFG)
  {
    /* timer compare interrupt */
    macMcuTimerCompare();
    return;
  }

   /* Should never come here. Unrecognized interrupt */
  MAC_ASSERT(FALSE);
}

/**************************************************************************************************
 * @fn          halMacTimerCapture
 *
 * @brief       -
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macMcuTimerCapture(void)
{
  halIntState_t  s;

  HAL_ENTER_CRITICAL_SECTION(s);

  /* Copy the tick value at the time of SFD.
   * This is the tickCount used to populate timestamp2.
   */
  macMcuTimerTickCountCapture = HAL_MAC_TIMER_SFD_CAPTURE();

  /* Copy the backoffCount at this instant.
   * The tickCount capture and backoffCount capture should happen at exact
   * same moment.
   * But since one of that is done in h/w (tick) while other (backoffCount) is
   * done in s/w (right here), we should be cautious. May be we could increment
   * the backoffCountCapture value by 1 if a pending rollover is observed
   * at this moment.
   */
  macMcuTimerBackoffCountCapture = macMcuTimerBackoffCount;

  HAL_EXIT_CRITICAL_SECTION(s);
}

#endif /* HAL_BOARD_LM3S */

/**************************************************************************************************
 * @fn          macMcuTimerBackoffCapture
 *
 * @brief       Returns the backoffCount value captured when SFD is detected.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
uint32 macMcuTimerBackoffCapture(void)
{
  return macMcuTimerBackoffCountCapture;
}

/**************************************************************************************************
 * @fn          macMcuTimerTickCapture
 *
 * @brief       Returns the tickCount value captured when SFD is detected.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
uint16 macMcuTimerTickCapture(void)
{
  return macMcuTimerTickCountCapture;
}

/**************************************************************************************************
 * @fn          macTimerCompareIsr
 *
 * @brief       Interrupt service routine that fires on a timer compare.  This is used to
 *              implement the MCU timer functionality.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macMcuTimerCompare(void)
{
#ifdef HAL_BOARD_LM3S

  if(macMcuTimerLeftoverUsecs)
  {
    macMcuTimerLeftoverUsecs = 0;
    if(macMcuTimerBackoffs != 0)
    {
      HAL_MAC_TIMER_COMPARE_TIMER_STOP();
      HAL_MAC_TIMER_COMPARE_CONFIGURE_CONT();
      HAL_MAC_TIMER_SET_COMPARE(MAC_SPEC_USECS_PER_BACKOFF);
      HAL_MAC_TIMER_CLEAR_COMPARE_FLAG();
      HAL_MAC_TIMER_COMPARE_TIMER_START();	
    }
    else
    {
      HAL_MAC_TIMER_COMPARE_TIMER_STOP();
      HAL_MAC_TIMER_COMPARE_DISABLE_INTERRUPT();
      (* pFuncMacMcuTimerCallback)();
    }
  }
  else
  {
    macMcuTimerBackoffs--;
    if (macMcuTimerBackoffs == 0)
    {
      HAL_MAC_TIMER_COMPARE_TIMER_STOP();
      HAL_MAC_TIMER_COMPARE_DISABLE_INTERRUPT();
      (* pFuncMacMcuTimerCallback)();
    }	
  }

#else

  macMcuTimerBackoffs--;
  if (macMcuTimerBackoffs == 0)
  {
    HAL_MAC_TIMER_COMPARE_DISABLE_INTERRUPT();
    (* pFuncMacMcuTimerCallback)();
  }

#endif /* HAL_BOARD_LM3S */

}

/**************************************************************************************************
 * @fn          macMcuTimerForceDelay
 *
 * @brief       Stretches the clock by tickDelay number of ticks.
 *
 * @param       tickDelay
 *
 * @return      none
 **************************************************************************************************
 */
void macMcuTimerForceDelay(uint16 tickDelay)
{
  halIntState_t  s;
  uint16 originalTickCount;
  uint16 originalRolloverTicks;
  uint8  restorePendingRolloverFlag = 0;

  if(tickDelay <= MAC_MCU_TIMER_MIN_TICK_DELAY)
  {
    /* Since the CPU and timer are both running on the same clk,
     * there is an overhead of about 120 ticks for stretching the timer.
     * So a request for tickDelay less than 120 ticks will result in
     * a delay that is longer than necessary.
     * In that case, we can just ignore the delay operation.
     */
    return;
  }
  else
  {
    /* Reduce the delay ticks by our overhead.
     * The compare count is programmed to (tickDelay-1). If tickDealy is
     * 1 or less it will result in compare value of zero and
     * the compare flag will never be set.
     * Hence we deduct 119 instead of 120 so that tickDelay will be
     * atleast 2.
     */
    tickDelay = tickDelay - MAC_MCU_TIMER_MIN_TICK_DELAY + 1;
  }

  HAL_ENTER_CRITICAL_SECTION(s);

  HAL_MAC_TIMER_STOP();

  if(HAL_MAC_TIMER_ROLLOVER_FLAG())
  {
    /* Must set this pending flag again after tick delay is done. */
    restorePendingRolloverFlag = 1;
    /* Clear the interrupt pending flag. */
    HAL_MAC_TIMER_ROLLOVER_CLEAR_INTERRUPT();
  }

  /* Save the current count value to be restore at the end of delay */
  originalTickCount     = HAL_MAC_TIMER_COUNT();

  /* Save the rollover tick count. Must add 1 since the SET_ROLLOVER macro
   * will substract one when we call it to restore this value.
   */
  originalRolloverTicks = HAL_MAC_TIMER_GET_ROLLOVER() + 1;


  /* Prepare timer for delay loop */
  HAL_MAC_TIMER_SET_COUNT(0);
  HAL_MAC_TIMER_SET_ROLLOVER(tickDelay);
  HAL_MAC_TIMER_ROLLOVER_DISABLE_INTERRUPT();

  /* Delay loop */
  HAL_MAC_TIMER_START();
  while(!HAL_MAC_TIMER_ROLLOVER_FLAG());
  HAL_MAC_TIMER_STOP();

  /* - Clear the interrupt flag
   * - Restore the tickCount and rolloverTicks
   * - Re-enable the rollover interrupt
   */
  HAL_MAC_TIMER_ROLLOVER_CLEAR_INTERRUPT();
  HAL_MAC_TIMER_SET_COUNT(originalTickCount);
  HAL_MAC_TIMER_SET_ROLLOVER(originalRolloverTicks);
  HAL_MAC_TIMER_ROLLOVER_ENABLE_INTERRUPT();

  if(restorePendingRolloverFlag)
  {
    /* Must force the interrupt pending flag since this last rollover
     * interrupt occurred while we were doing the realignment and it
     * has not been processed yet.
     */
    HAL_MAC_TIMER_ROLLOVER_FORCE_INTERRUPT();
  }

  /* Re-start the timer */
  HAL_MAC_TIMER_START();

  HAL_EXIT_CRITICAL_SECTION(s);
}

/**************************************************************************************************
 * @fn          macMcuTimerRolloverIsr
 *
 * @brief       Interrupt service routine that fires every timer rollover.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
#ifdef HAL_BOARD_LM3S

void hal_mac_timer_rollover_isr_function(void)
{
  halIntState_t  s;

#ifdef POWER_SAVING
  halSleepExit();
#endif
  HAL_MAC_TIMER_ROLLOVER_CLEAR_INTERRUPT();

  HAL_ENTER_CRITICAL_SECTION(s);

  macMcuTimerBackoffCount++;

  if (macMcuTimerBackoffCount == macMcuTimerBackoffCompare)
  {
    macBackoffTimerCompareIsr();
  }

  if (macMcuTimerBackoffCount == macMcuTimerBackoffRollover)
  {
    MAC_RADIO_BACKOFF_SET_COUNT(0);
    macBackoffTimerPeriodIsr();
  }

  if (macMcuTimerRecordMaxRssiFlag)
  {
    macDualchipRecordMaxRssiIsr();
  }

  /* Free running counter used by higher layer */
  mcuPrecisionCount++;

  HAL_EXIT_CRITICAL_SECTION(s);
}

#else /* !HAL_BOARD_LM3S */

HAL_MAC_TIMER_ROLLOVER_ISR_FUNCTION()
{
  halIntState_t  s;

#ifdef POWER_SAVING
  __low_power_mode_off_on_exit();
#endif

  HAL_ENTER_CRITICAL_SECTION(s);

  macMcuTimerBackoffCount++;

  if (macMcuTimerBackoffCount == macMcuTimerBackoffCompare)
  {
    macBackoffTimerCompareIsr();
  }
  
  if (macMcuTimerBackoffCount == macMcuTimerBackoffRollover)
  {
    MAC_RADIO_BACKOFF_SET_COUNT(0);
    macBackoffTimerPeriodIsr();
  }  

  if (macMcuTimerRecordMaxRssiFlag)
  {
    macDualchipRecordMaxRssiIsr();
  }

  /* Free running counter used by higher layer */
  mcuPrecisionCount++;

  HAL_EXIT_CRITICAL_SECTION(s);
}

#endif /* HAL_BOARD_LM3S */

/**************************************************************************************************
 * @fn          macMcuPrecisionCount
 *
 * @brief       This function is used by higher layer to read a free running counter driven by
 *              MAC timer.
 *
 * @param       none
 *
 * @return      mcuPrecisionCount
 **************************************************************************************************
 */
uint32 macMcuPrecisionCount(void)
{
  return(mcuPrecisionCount);
}

#ifdef POWER_SAVING
/**************************************************************************************************
 * @fn          macMcuPrecisionCountSleepUpdate
 *
 * @brief       This function is used by sleep module to update the macPrecisionCount after
 *              waking up from sleep.
 *
 * @param       elapsed time in sleep
 *
 * @return      none
 **************************************************************************************************
 */
void macMcuPrecisionCountSleepUpdate(uint32 elapsed_time)
{
  mcuPrecisionCount += elapsed_time;
}
#endif /* POWER_SAVING */

/**************************************************************************************************
 * @fn          macMcuAccumulatedOverFlow
 *
 * @brief       This function is used to accumulate overflow if applicable on the relevant platform
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macMcuAccumulatedOverFlow(void)
{
  /* Stub */
}

