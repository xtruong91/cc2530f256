/**************************************************************************************************
    Filename:       zap_phy.c
    Revised:        $Date: 2010-12-01 15:31:18 -0800 (Wed, 01 Dec 2010) $
    Revision:       $Revision: 24529 $

    Description:

    This file declares the functionality of the ZNP Application Processor Physical Link Layer.


    Copyright 2009-2010 Texas Instruments Incorporated. All rights reserved.

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

#include "hal_board.h"
#include "zap_phy.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#if !defined SB_FORCE_BOOT
#define SB_FORCE_BOOT               0xF8
#define SB_FORCE_RUN               (SB_FORCE_BOOT ^ 0xFF)
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

#if ZAP_PHY_SPI
#include "zap_phy_spi.c"
#elif ZAP_PHY_UART
#include "zap_phy_uart.c"
#else
#error Must define a valid PHY for the ZAP
#endif

static void zapPhyRun(uint8 port);

/**************************************************************************************************
 * @fn          zapPhyInit
 *
 * @brief       This function initializes the ZAP physical link(s).
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zapPhyInit(void)
{
  HAL_ZNP_RST_CFG();
  HAL_ZNP_CFG0_HI();

#if ZAP_PHY_SPI
  HAL_ZNP_CFG1_HI();
  zapPhySpiInit();
#endif
#if ZAP_PHY_UART
  HAL_ZNP_CFG1_LO();
  zapPhyUartInit();
#endif

  MicroWait(100);
  HAL_ZNP_RUN();
  zapPhyRun(zapAppPort);
}

/**************************************************************************************************
 * @fn          zapPhyExec
 *
 * @brief       This function polls the ZAP physical link(s).
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zapPhyExec(uint16 evts)
{
#if ZAP_PHY_SPI
  if (evts & ZAP_PHY_SPI_EVT)
  {
    zapPhySpiExec();
  }
#endif
#if ZAP_PHY_UART
  if (evts & ZAP_PHY_UART_EVT)
  {
    zapPhyUartExec();
  }
#endif
}

/**************************************************************************************************
 * @fn          zapPhyReset
 *
 * @brief       This function resets the ZNP slave.
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP to reset.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zapPhyReset(uint8 port)
{
  /* TODO - perhaps remove the 2 NOP's immediately before a final production release of a product.
   * This is the crucial place to always have a debugger break-point to catch any newly added
   * SREQ's that require more than the default wait time for the SRSP from the ZNP.
   * Stop and examine the call stack here to see what went wrong. Note that if the ZNP is built
   * with ZNP_RUN_WDOG=TRUE, it will be continually resetting and therefore toggling SRDY while the
   * ZAP is stopped here or at any break-point.
   */
  asm("NOP");
  HAL_ZNP_RST();
  MicroWait(100);
  HAL_ZNP_RUN();
  zapLostSync(port);
  zapPhyRun(zapAppPort);
  asm("NOP");
}

/**************************************************************************************************
 * @fn          zapPhySend
 *
 * @brief       This function sends an RPC message buffer created with zap_msg_allocate().
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP to which to send the message.
 * @param       pBuf - A the buffer pointer returned by zap_msg_allocate().
 *
 * output parameters
 *
 * None.
 *
 * @return      SUCCESS or FAILURE.
 **************************************************************************************************
 */
uint8 zapPhySend(uint8 port, uint8 *pBuf)
{
#if ZAP_PHY_SPI
  return zapPhySpiSend(port, pBuf);
#endif
#if ZAP_PHY_UART
  return zapPhyUartSend(port, pBuf);
#endif
}

/**************************************************************************************************
 * @fn          zapPhySync
 *
 * @brief       This function sets sync TRUE for the ZNP specified.
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP that is in sync.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zapPhySync(uint8 port)
{
#if ZAP_PHY_SPI
  zapPhySpiSync(port);
#endif
#if ZAP_PHY_UART
  zapPhyUartSync(port);
#endif
}

/**************************************************************************************************
 * @fn          zapPhyWait
 *
 * @brief       This function sets the SRSP wait timeout as specified by the parameter or to the
 *              specific medium's default value if the parameter is zero.
 *
 * input parameters
 *
 * @param       wait - the maximum wait delay for an SRSP.
 * @param       port - Port Id corresponding to the ZNP wait.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zapPhyWait(uint8 port, uint16 wait)
{
#if ZAP_PHY_SPI
  zapPhySpiWait(port, wait);
#endif
#if ZAP_PHY_UART
  zapPhyUartWait(port, wait);
#endif
}

/**************************************************************************************************
 * @fn          zapPhyRun
 *
 * @brief       This function starts the ZNP slave transport running.
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP to Run.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void zapPhyRun(uint8 port)
{
#if ZAP_PHY_SPI
  zapPhySpiRun(zapAppPort);
#endif
#if ZAP_PHY_UART
  zapPhyUartRun(zapAppPort);
#endif
}

/**************************************************************************************************
*/
