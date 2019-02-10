/**************************************************************************************************
  Filename:       mac_dualchip_tx.c
  Revised:        $Date: 2011-07-13 12:08:51 -0700 (Wed, 13 Jul 2011) $
  Revision:       $Revision: 26770 $

  Description:    Describe the purpose and contents of the file.


  Copyright 2006-2010 Texas Instruments Incorporated. All rights reserved.

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

/* high-level */
#include "mac_spec.h"
#include "mac_pib.h"
#include "mac_main.h"

/* exported low-level */
#include "mac_low_level.h"

/* low-level specific */
#include "mac_dualchip.h"
#include "mac_dualchip_tx.h"
#include "mac_mcu_timer.h"
#include "mac_tx.h"
#include "mac_rx.h"
#include "mac_rx_onoff.h"

/* target specific */
#include "mac_radio_defs.h"

/* debug */
#include "mac_assert.h"
#include "hal_board.h"


/* ------------------------------------------------------------------------------------------------
 *                                         Local Prototypes
 * ------------------------------------------------------------------------------------------------
 */
static void dualchipTxUnslottedCsmaRxOn(void);
static void dualchipTxUnslottedCsmaCca(void);
static void dualchipTxSlotted(void);
static void dualchipTxSlottedCollisionDetection(void);
static void dualchipTxSlottedCsmaCca(void);
static void dualchipTxSlottedCsma(void);


/**************************************************************************************************
 * @fn          macDualchipTxReset
 *
 * @brief       Initialize all code in this 'C' module.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macDualchipTxReset(void)
{
  MAC_MCU_TIMER_CANCEL();
  MAC_RADIO_CANCEL_ACK_TX_DONE_CALLBACK();
}


/**************************************************************************************************
 * @fn          macDualchipTxGoCsma
 *
 * @brief       Starts unslotted CSMA transmit algorithm.  Receiver is turned on (it must be on
 *              for at least one backoff for CCA to be valid) and then a timer is set for CSMA
 *              delay time.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macDualchipTxGoCsma(void)
{
  MAC_ASSERT(macTxType == MAC_TX_TYPE_UNSLOTTED_CSMA);  /* only unslotted CSMA is supported */

  /* Turn off RX to save power even if RX falg is enabled for MAC_RX_POLL */
  if (!macRxEnableFlags || (macRxEnableFlags | MAC_RX_POLL) == MAC_RX_POLL)
  {
    macRxOff();
  }
  
  /*
   *  Delay will break into two parts, dealy and delay with Rx on for one backoff period. The 
   *  receiver must be on for at least one backoff for CCA to be valid.
   */
  if (macTxCsmaBackoffDelay < 2)
  {
    /* Delay is zero or one backoff, turn on the RX for one backoff */
    dualchipTxUnslottedCsmaRxOn();
  }
  else
  {
    /* 
     *  Delay is two or more backoffs, request callback for the CSMA delay - 1, then turn on 
     *  the RX for one backoff.
     */
    MAC_MCU_TIMER_UNSLOTTED_BACKOFFS(macTxCsmaBackoffDelay - 1, &dualchipTxUnslottedCsmaRxOn);
  }
}


/**************************************************************************************************
 * @fn          dualchipTxUnslottedCsmaRxOn
 *
 * @brief       Turn on RX before starting unslotted CSMA transmit algorithm and then a timer is 
 *              set for one backoff.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
static void dualchipTxUnslottedCsmaRxOn(void)
{
  /* turn on receiver and request callback for the CSMA delay for one backoff */
  macRxOn();
  MAC_MCU_TIMER_UNSLOTTED_BACKOFFS(1, &dualchipTxUnslottedCsmaCca);
}


/*=================================================================================================
 * @fn          dualchipTxUnslottedCsmaCca
 *
 * @brief       Performs CCA for unslotted CSMA transmit.
 *              If channel is clear, transmit is started.
 *              If channel is not clear, "channel busy" callback is executed.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void dualchipTxUnslottedCsmaCca(void)
{
  halIntState_t  s;

  /* peform CCA */
  HAL_ENTER_CRITICAL_SECTION(s);

  macSpiCmdStrobe(STXONCCA);

  if (MAC_RADIO_READ_SAMPLED_CCA())
  {
    /* CCA succeeded - request TX done interrupt, flag receiver as forced on */
    HAL_MAC_CLEAR_SFD_INT_FLAG();
    HAL_MAC_ENABLE_SFD_INT();
    MAC_RX_WAS_FORCED_ON();
    HAL_EXIT_CRITICAL_SECTION(s);
    return;
  }

  HAL_EXIT_CRITICAL_SECTION(s);

  /* CCA failed */
  macTxChannelBusyCallback();
}

/*=================================================================================================
 * @fn          macDualchipTxPrepCsmaSlotted
 *
 * @brief       This function prepares CC2520 for a slotted csma transmission.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
void macDualchipTxPrepCsmaSlotted(void)
{
  /* Configure the Trigger and CCA pins for slotted transmission */
  MAC_RADIO_CONFIG_TRIGGER_CCA();
}


/*=================================================================================================
 * @fn          macDualchipTxPrepSlotted
 *
 * @brief       This function prepares CC2520 for a slotted transmission.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
void macDualchipTxPrepSlotted(void)
{
  /* Configure the Trigger and CCA pins for slotted transmission */
  MAC_RADIO_CONFIG_TRIGGER_CCA();

  /* Set compare output trigger setting to strobe STXONCCA */
  MAC_RADIO_SET_TRIGGER_STXON();
}


/*=================================================================================================
 * @fn          macDualchipTxGoSlotted
 *
 * @brief       This function starts a backoff timer. When the backoff countdown reaches 0,
 *              a "trigger" GPIO is toggled. The "trigger" GPIO pin will cause the CC2520
 *              to transmit per slotted timing.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
void macDualchipTxGoSlotted(void)
{
#if (defined HAL_PA_LNA || defined HAL_PA_LNA_CC2590)
  /*
   * Cannot support beacon enabled network when PA/LNA is installed in EM board.
   * CC2520 GPIO4 is uesed to control PA/LNA and not available for SFD.
   * Beacon enabled network need SFD to capture beacon time base.
   */
  MAC_ASSERT(0);
#endif

  /* Disable Rx and flush RXFIFO due to chip bug #1546 */
  if (macChipVersion == REV_A)
  {
    macRxHardDisable();
  }
  macRxOn();

  MAC_MCU_TIMER_SLOTTED_BACKOFFS(macTxSlottedDelay - 1, &dualchipTxSlotted);
}


/*=================================================================================================
 * @fn          macDualchipTxGoSlottedCsma
 *
 * @brief       This function prepares CC2520 for a slotted CSMA transmission.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
void macDualchipTxGoSlottedCsma(void)
{
  MAC_ASSERT(macTxType == MAC_TX_TYPE_SLOTTED_CSMA);  /* only slotted CSMA is supported */

  /*
   *  Delay of zero is not possible as receiver must be on for at least one backoff for
   *  CCA to be valid.
   */
  if (macTxCsmaBackoffDelay == 0)
  {
    macTxCsmaBackoffDelay = 1;
  }

  /* turn on receiver and request callback for the CSMA delay */
  macRxOn();
  MAC_MCU_TIMER_SLOTTED_BACKOFFS(macTxCsmaBackoffDelay, &dualchipTxSlottedCsmaCca);
}


/**************************************************************************************************
 * @fn          macDualchipTxRequestAckTimeoutCallback
 *
 * @brief       Requests a callback to function macTxAckNotReceivedCallback() after the timeout
 *              period for receiving an ACK expires.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macDualchipTxRequestAckTimeoutCallback(void)
{
  MAC_MCU_TIMER_USECS(macPib.ackWaitDuration * MAC_SPEC_USECS_PER_SYMBOL, &macTxAckNotReceivedCallback);
}


/**************************************************************************************************
 * @fn          macDualchipTxCancelAckTimeoutCallback
 *
 * @brief       Cancels ACK timeout callback.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macDualchipTxCancelAckTimeoutCallback(void)
{
  MAC_MCU_TIMER_CANCEL();
}


/**************************************************************************************************
 * @fn          macDualchipTxForceTxDoneIfPending
 *
 * @brief       If a 'transmit done' callback is pending, execute it here immediately
 *              Cancel the interrupt as well so the callback won't execute twice.
 *              (Note however, it is possible it could execute twice or rare timing situations.)
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macDualchipTxForceTxDoneIfPending(void)
{
  if (HAL_MAC_SFD_INT_IS_ENABLED())
  {
    HAL_MAC_DISABLE_SFD_INT();

    /* execute the transmit complete callback */
    macTxDoneCallback();
  }
}


/**************************************************************************************************
 * @fn          macDualchipTxAckDoneIsr
 *
 * @brief       Interrupt service routine for TX_ACK_DONE interrupts.  The function pointer variable
 *              pFuncSfdIsr is used as state variable to execute the needed functionality.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macDualchipTxAckDoneIsr(void)
{
  MAC_RADIO_CANCEL_ACK_TX_DONE_CALLBACK();

  /* call the ACK transmit complete logic */
  macRxAckTxDoneCallback();
}


/**************************************************************************************************
 * @fn          macDualchipTxFrmDoneIsr
 *
 * @brief       Interrupt service routine for TX_FRM_DONE interrupts.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macDualchipTxFrmDoneIsr(void)
{
  /* call the TX frame transmit complete logic */
  macTxDoneCallback();
}


/*=================================================================================================
 * @fn          dualchipTxSlotted
 *
 * @brief       This is called one backoff period before slotted TX.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void dualchipTxSlotted(void)
{
  HAL_MAC_CLEAR_SFD_INT_FLAG();
  HAL_MAC_ENABLE_SFD_INT();
  MAC_RX_WAS_FORCED_ON();

  /* Start the timer to expire on the backoff boundary.
   * At the backoff boundary (rollover) the timer compare output will toggle
   * from low to high. This transition will trigger the RF chip to start
   * data transmission.
   */
  HAL_MAC_TIMER_ENABLE_OUTPUT_COMPARE();
  MAC_MCU_TIMER_SLOTTED_BACKOFFS(1, &dualchipTxSlottedCollisionDetection);
}


/*=================================================================================================
 * @fn          dualchipTxSlottedCollisionDetection
 *
 * @brief       This is called when slotted TX is transimitted.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void dualchipTxSlottedCollisionDetection(void)
{
  /* Disable the output compare here now that we have already triggered the
   * transmission.
   */
  HAL_MAC_TIMER_DISABLE_OUTPUT_COMPARE();

  /* Let CRC handle the collision detecton */
}


/*=================================================================================================
 * @fn          dualchipTxSlottedCsmaCca
 *
 * @brief       This is called to perform the first CCA check.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void dualchipTxSlottedCsmaCca(void)
{
  halIntState_t  s;

  /* There is a potential improvement on the CCA.
   * Ideally, we should read sampled CCA but the
   * way we setup the timer compare made it difficult.
   */
  if (HAL_MAC_READ_CCA_PIN())
  {
    HAL_ENTER_CRITICAL_SECTION(s);

    /* Set compare output trigger setting to STXONCCA */
    MAC_RADIO_SET_TRIGGER_STXONCCA();

    /* Start timer to trigger Tx line on the backoff boundary.
     * At the backoff boundary (rollover) the timer compare output will toggle
     * from low to high. This transition will trigger the RF chip to start
     * data transmission.
     */
    HAL_MAC_TIMER_ENABLE_OUTPUT_COMPARE();

    MAC_MCU_TIMER_SLOTTED_BACKOFFS(1, &dualchipTxSlottedCsma);

    HAL_EXIT_CRITICAL_SECTION(s);
  }
  else
  {
    /* CCA failed */
    macTxChannelBusyCallback();
  }
}


/*=================================================================================================
 * @fn          dualchipTxSlottedCsma
 *
 * @brief       This is called to perform the second CCA check and transmit if CCA passed.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void dualchipTxSlottedCsma(void)
{
  /* Disable the output compare here now that we have already triggered the
   * transmission.
   */
  HAL_MAC_TIMER_DISABLE_OUTPUT_COMPARE();

  /* Peform CCA */
  if(MAC_RADIO_READ_SAMPLED_CCA())
  {
    /* CCA succeeded - request TX done interrupt, flag receiver as forced on */
    HAL_MAC_CLEAR_SFD_INT_FLAG();
    HAL_MAC_ENABLE_SFD_INT();
    MAC_RX_WAS_FORCED_ON();
  }
  else
  {
    /* CCA failed */
    macTxChannelBusyCallback();
  }
}


