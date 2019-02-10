/**************************************************************************************************
  Filename:       hal_mac_cfg.c
  Revised:        $Date: 2011-05-09 13:22:48 -0700 (Mon, 09 May 2011) $
  Revision:       $Revision: 25921 $

  Description:    Describe the purpose and contents of the file.


  Copyright 2009-2011 Texas Instruments Incorporated. All rights reserved.

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

/*
 *   Board Configuration File for low-level MAC
 *  --------------------------------------------
 *   Manufacturer : Generic
 *   Part Number  : -
 *   Processor    : Texas Instruments MSP430 variants
 *
 */


/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "hal_mac_cfg.h"
#include "mac_assert.h"
#include "mac_rx.h"
#include "mac_tx.h"
#include "mac_radio_defs.h"


/* ------------------------------------------------------------------------------------------------
 *                                           Defines
 * ------------------------------------------------------------------------------------------------
 */

/* Macro and Defines for FIFOP interrupt */
#define FIFOP_VECTOR()      HAL_MAC_FIFOP_INT_VECTOR()
#if (FIFOP_VECTOR() == PORT1_VECTOR)
#define PXIE    P1IE
#define PXIFG   P1IFG
#elif (FIFOP_VECTOR() == PORT2_VECTOR)
#define PXIE    P2IE
#define PXIFG   P2IFG
#else
#error "ERROR: Unknown or unsupported vector specified for FIFOP interrupt."
#endif


/**************************************************************************************************
 * @fn          halMacInit
 *
 * @brief       Transiver interface initializations.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void halMacInit(void)
{
  /* configure GPIO pins */
  HAL_MAC_CONFIG_FIFOP_PIN_AS_INPUT();
  HAL_MAC_CONFIG_FIFO_PIN_AS_INPUT();
  HAL_MAC_CONFIG_TX_ACK_DONE_PIN_AS_INPUT();
  HAL_MAC_CONFIG_CCA_PIN_AS_INPUT();

  /* The trigger pin is configured dynamically per beacon mode */

  HAL_MAC_CONFIG_VREG_EN_PIN_AS_OUTPUT();
  HAL_MAC_CONFIG_RESETN_PIN_AS_OUTPUT();

  /* configure input capture */
  HAL_MAC_CONFIG_SFD_PIN_AS_INPUT();

  /* configure interrupts */
  HAL_MAC_CONFIG_FIFOP_RISING_EDGE_INT();
  HAL_MAC_CONFIG_TX_ACK_DONE_RISING_EDGE_INT();

  /* configure SPI pins */
  HAL_MAC_SPI_CONFIG_CSN_PIN_AS_OUTPUT();
  HAL_MAC_SPI_CONFIG_SCLK_PIN_AS_OUTPUT();
  HAL_MAC_SPI_CONFIG_SI_PIN_AS_OUTPUT();
  HAL_MAC_SPI_CONFIG_SO_PIN_AS_INPUT();

  /* We are done with the ID pin, disable pullup to save power */
  HAL_BOARD_ID_DISABLE_PULLUP();
}


/**************************************************************************************************
 * @fn          halMacIoPortIsr
 *
 * @brief       Radio interrupt service routine.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
HAL_ISR_FUNCTION( halMacFifopIsr, FIFOP_VECTOR() )
{
  uint8 pxie, excflag0;

  /*
   *  Copy interrupt enable register.  Used to avoid compiler warning regarding
   *  undefined order of volatile acesses.
   */
  pxie = PXIE;

  /* currently FIFO, FIFOP, and TX_ACK_DONE are the only signals that cause an interrupt on this port */
  MAC_ASSERT(pxie & PXIFG & (BV(HAL_MAC_FIFOP_GPIO_BIT) | BV(HAL_MAC_TX_ACK_DONE_GPIO_BIT) | BV(HAL_MAC_FIFO_GPIO_BIT))); /* unrecognized interrupt */

  /*
   *  Test all bits in port that are used as interrupts.  If the enable flag is set and
   *  the interrupt pending flag is set, call the corresponding ISR.
   *
   *  Note: the MSP430 requires that the software clear the interrupt pending flag.
   *  This logic design of the ISR's themselves handles this.
   */

  /* TX_ACK_DONE interrupt - processes TX_ACK_DONE before FIFOP when interrupts are simultaneous */
  if (pxie & PXIFG & BV(HAL_MAC_TX_ACK_DONE_GPIO_BIT))
  {
    excflag0 = macSpiReadReg(EXCFLAG0);

    /* Determine what caused the GPIO to go high */
    if (MAC_RADIO_TX_ACK_AND_TX_FRM_DONE_EXCEPTION(excflag0))
    {
      macDualchipTxAckDoneIsr();
      macDualchipTxFrmDoneIsr();
      MAC_RADIO_CLEAR_TX_ACK_AND_TX_FRM_DONE_PIN();
    }
    else
    {
      if (MAC_RADIO_TX_ACK_DONE_EXCEPTION(excflag0))
      {
        macDualchipTxAckDoneIsr();
        MAC_RADIO_CLEAR_TX_ACK_DONE_PIN();
      }
      else
      if (MAC_RADIO_TX_FRM_DONE_EXCEPTION(excflag0))
      {
        macDualchipTxFrmDoneIsr();
        MAC_RADIO_CLEAR_TX_FRM_DONE_PIN();
      }
    }

    if ( !HAL_MAC_READ_TX_ACK_DONE_PIN() )
    {
      /* Clear the flag only when the pin is phisically low */
      HAL_MAC_CLEAR_TX_ACK_DONE_INT_FLAG();
    }
  }

  /* 
   * TX interrupt is very time consuming and it may trigger more transmissions. 
   * Don't let RX run until all TXs are completed. 
   */
  else if (pxie & PXIFG & BV(HAL_MAC_FIFOP_GPIO_BIT)) /* FIFOP interrupt */
  {
    /* To get here: FIFOP = 1, FIFO = X
     * See CC2520 Errata Note Bug 2, Glitches on the FIFOP Signal.
     */
    if (!HAL_MAC_READ_FIFOP_PIN() && HAL_MAC_READ_FIFO_PIN())
    {
      /* To get here: FIFOP = 0, FIFO = 1
       * It appears that after rxPayloadIsr(), a FIFOP glitch can occur. If
       * this is happening, the FIFOP interrupt flag should not be cleared.
       * Instead, do nothing so that the FIFOP interrupt can be triggered
       * again when the RX FIFO threshold is reached. FIFOP pin should stay
       * high for the real FOFOP interrupt.
       */
      if (!macRxActive)
      {
        /* If the FIFOP glitch occurs before RX is active, go ahead and
         * clear the FIFOP interrupt flag as if the glitch has never happened.
         */
        HAL_MAC_CLEAR_FIFOP_INT_FLAG();
      }
    }
    else if (HAL_MAC_READ_FIFOP_PIN())
    {
      /* To get here: FIFOP = 1, FIFO = X */
      do
      {
        /* Clear the FIFOP int flag. */
        HAL_MAC_CLEAR_FIFOP_INT_FLAG();

        macRxThresholdIsr();

      } while(HAL_MAC_READ_FIFOP_PIN());
    }
    else
    {
      /* To get here: FIFOP = 0, FIFO = 0
       * This is the second half of the FIFOP glitch workaround.
       * Need to clear FIFOP interrupt so that the FIFOP interrupt will be 
       * fired on the next rising edge. 
       */
      HAL_MAC_CLEAR_FIFOP_INT_FLAG();
    }
  }
  
  /* The RXFIFO overflow must be checked last to ensure new FIFOP interrupt */
  if (pxie & PXIFG & BV(HAL_MAC_FIFO_GPIO_BIT)) /* FIFO interrupt */
  {
    /* Handle RX FIFO overflow */
    if (MAC_RADIO_RX_FIFO_HAS_OVERFLOWED())
    {
      macRxFifoOverflowIsr();
 
      /* Clear RX FIFO overflow exception flag */
      macSpiWriteReg(EXCFLAG0, RX_OVERFLOW_FLAG);
    }
    
    /* clear interrupt */
    HAL_MAC_CLEAR_FIFO_INT_FLAG();
  }
}




