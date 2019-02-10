/**************************************************************************************************
  Filename:       mac_mcu_timer.h
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

#ifndef MAC_MCU_TIMER_H
#define MAC_MCU_TIMER_H


/* ------------------------------------------------------------------------------------------------
 *                                         Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "hal_types.h"
#include "hal_mac_cfg.h"
#include "mac_spec.h"


/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */
#define MAC_MCU_TIMER_ENABLE_ENERGY_DETECT()    st( macMcuTimerRecordMaxRssiFlag = 1; )
#define MAC_MCU_TIMER_DISABLE_ENERGY_DETECT()   st( macMcuTimerRecordMaxRssiFlag = 0; )


/* ------------------------------------------------------------------------------------------------
 *                                           Defines
 * ------------------------------------------------------------------------------------------------
 */
/* from MSP430 datasheet */
#define TAIV_TACCR1_CCIFG   0x02    /* TAIV value for capture/compare 1 interrupt */
#define TAIV_TACCR2_CCIFG   0x04    /* TAIV value for capture/compare 2 interrupt */

/* Since the CPU and timer are both running on the same clk,
 * there is an overhead of about 120 ticks for stretching the timer.
 * So a request for tickDelay less than 120 ticks will result in
 * a delay that is longer than necessary.
 * In that case, we can just ignore the delay operation.
 */
#define MAC_MCU_TIMER_MIN_TICK_DELAY 120

/* ------------------------------------------------------------------------------------------------
 *                                   Macros : Timer Model #1
 * ------------------------------------------------------------------------------------------------
 */
#if (!defined(HAL_MAC_TIMER_MODEL) || (defined(HAL_MAC_TIMER_MODEL) && (HAL_MAC_TIMER_MODEL == 1)))

/* - - - - - - - - - - - - - - - - - - - - - -
 *   Timer Model #1 is the the default.
 *   It uses one timer with a single compare.
 * - - - - - - - - - - - - - - - - - - - - - -
 */
#ifdef HAL_BOARD_LM3S

#define MAC_MCU_TIMER_WAIT_USECS(x)             macMcuTimerWaitUsecs(x);

#define MAC_MCU_TIMER_USECS(x,pF)               st( pFuncMacMcuTimerCallback = pF; \
                                                    macMcuTimerUsecs(x); )

#define MAC_MCU_TIMER_UNSLOTTED_BACKOFFS(x,pF)  st( pFuncMacMcuTimerCallback = pF; \
                                                    macMcuTimerUnslottedBackoffs(x); )

#else

#define MAC_MCU_TIMER_WAIT_USECS(x)             st( macMcuTimerCount = HAL_MAC_TIMER_COUNT(); \
                                                    macMcuTimerWaitUsecs(x); )

#define MAC_MCU_TIMER_USECS(x,pF)               st( macMcuTimerCount = HAL_MAC_TIMER_COUNT(); \
                                                    pFuncMacMcuTimerCallback = pF; \
                                                    macMcuTimerUsecs(x); )

#define MAC_MCU_TIMER_UNSLOTTED_BACKOFFS(x,pF)  st( macMcuTimerCount = HAL_MAC_TIMER_COUNT(); \
                                                    pFuncMacMcuTimerCallback = pF; \
                                                    macMcuTimerUnslottedBackoffs(x); )

#endif /* HAL_BOARD_LM3S */

#define MAC_MCU_TIMER_CANCEL()                  st( HAL_MAC_TIMER_COMPARE_DISABLE_INTERRUPT(); )

#define MAC_MCU_TIMER_SLOTTED_BACKOFFS(x,pF)    st( pFuncMacMcuTimerCallback = pF; \
                                                    macMcuTimerSlottedBackoffs(x); )

/* ------------------------------------------------------------------------------------------------
 *                                   Macros : Timer Model #2
 * ------------------------------------------------------------------------------------------------
 */
#elif (defined(HAL_MAC_TIMER_MODEL) && (HAL_MAC_TIMER_MODEL == 2))

/* - - - - - - - - - - - - - - - - - - - - - -
 *   Timer Model #2 uses two timers but
 *   does not use a compare.
 * - - - - - - - - - - - - - - - - - - - - - -
 */
#define TICKS_PER_1000_USECS                      (((uint32)HAL_MAC_TIMER_TICKS_PER_USEC) * 1000)

#define MAC_MCU_TIMER_START_COUNTDOWN(x)          st( HAL_MAC_TIMER_COMPARE_TIMER_STOP(); \
                                                      HAL_MAC_TIMER_SET_COMPARE((x) - 1); \
                                                      HAL_MAC_TIMER_CLEAR_COMPARE_FLAG(); \
                                                      HAL_MAC_TIMER_COMPARE_TIMER_START(); )

#define MAC_MCU_TIMER_START_COUNTDOWN_USECS(x)    st( MAC_MCU_TIMER_START_COUNTDOWN(((x) * (uint32)(HAL_MAC_TIMER_TICKS_PER_USEC * 1000)) / 1000); )

#define MAC_MCU_TIMER_WAIT_USECS(x)               st( MAC_ASSERT(!HAL_MAC_TIMER_COMPARE_INTERRUPT_IS_ENABLED()); \
                                                      MAC_MCU_TIMER_START_COUNTDOWN_USECS(x); \
                                                      while(!HAL_MAC_TIMER_COMPARE_FLAG()); )

#define MAC_MCU_TIMER_USECS(x,pF)                 st( MAC_ASSERT(!HAL_MAC_TIMER_COMPARE_INTERRUPT_IS_ENABLED()); \
                                                      pFuncMacMcuTimerCallback = pF; \
                                                      macMcuTimerBackoffs = 1; \
                                                      MAC_MCU_TIMER_START_COUNTDOWN_USECS(x); \
                                                      HAL_MAC_TIMER_COMPARE_ENABLE_INTERRUPT(); )

#define MAC_MCU_TIMER_UNSLOTTED_BACKOFFS(x,pF)    st( MAC_ASSERT(!HAL_MAC_TIMER_COMPARE_INTERRUPT_IS_ENABLED()); \
                                                      MAC_ASSERT(x != 0); /* must be at least one backoff */ \
                                                      pFuncMacMcuTimerCallback = pF; \
                                                      macMcuTimerBackoffs = x; \
                                                      MAC_MCU_TIMER_START_COUNTDOWN( MAC_RADIO_TIMER_TICKS_PER_BACKOFF() ); \
                                                      HAL_MAC_TIMER_COMPARE_ENABLE_INTERRUPT(); )

#define MAC_MCU_TIMER_CANCEL()                    st( HAL_MAC_TIMER_COMPARE_DISABLE_INTERRUPT(); )


/* - - - - - - - - - - - - - - - - - - - - - -
*/
#elif defined(HAL_MAC_TIMER_MODEL)
#error "ERROR: Unknown timer model."
#endif


/* ------------------------------------------------------------------------------------------------
 *                                        Externs
 * ------------------------------------------------------------------------------------------------
 */
extern uint32  macMcuTimerBackoffCount;
extern uint32  macMcuTimerBackoffCompare;
extern uint32  macMcuTimerBackoffRollover;
extern void (* pFuncMacMcuTimerCallback)(void);
extern uint8   macMcuTimerRecordMaxRssiFlag;
extern uint8   macMcuTimerBackoffs;
extern uint16  macMcuTimerCount;


/* ------------------------------------------------------------------------------------------------
 *                                         Prototypes
 * ------------------------------------------------------------------------------------------------
 */
void macMcuTimerInit(void);
void macMcuTimerWaitUsecs(uint16 usecs);
void macMcuTimerUsecs(uint16 usecs);
void macMcuTimerUnslottedBackoffs(uint8 backoffs);
void macMcuTimerSlottedBackoffs(uint8 backoffs);
uint32 macMcuTimerBackoffCapture(void);
uint16 macMcuTimerTickCapture(void);
void macMcuTimerForceDelay(uint16 usec);
uint32 macMcuPrecisionCount(void);
void macMcuAccumulatedOverFlow(void);

/**************************************************************************************************
 */
#endif
