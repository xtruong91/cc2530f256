/**************************************************************************************************
    Filename:       zap_phy_spi.c
    Revised:        $Date: 2013-11-12 16:45:07 -0800 (Tue, 12 Nov 2013) $
    Revision:       $Revision: 36060 $

    Description:

    This file defines the functionality of the ZNP Application Processor.
    This file declares the functionality of the ZAP Physical Link Layer by SPI.


    Copyright 2009-2013 Texas Instruments Incorporated. All rights reserved.

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
#include "hal_spi.h"
#include "MT.h"
#include "MT_RPC.h"
#include "MT_UART.h"
#include "OSAL.h"
#include "zap_app.h"
#include "zap_phy.h"
#include "ZComDef.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

#if !defined ZAP_PHY_SPI_PORT0
#define ZAP_PHY_SPI_PORT0                  0
#endif

// Hook for supporting 2 ZNP's on 2 different SPI's.
#if !defined ZAP_PHY_SPI_PORT_CNT
#define ZAP_PHY_SPI_PORT_CNT               1
#endif

#if !defined ZAP_PHY_SPI_WAIT
#define ZAP_PHY_SPI_WAIT                   2000
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

static uint16 zapPhySpiDly;

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static void getSRDY1(uint8 port);
static uint8 getSRDY2(uint8 port);
static uint8 spiSREQ(uint8 port, uint8 *pBuf);
static uint8 spiAREQ(uint8 port, uint8 *pBuf);
static void zapPhySpiRun(uint8 port);

/**************************************************************************************************
 * @fn          zapPhySpiPoll
 *
 * @brief       This function polls the ZNP slave according to the RPC protocol for POLL.
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP to poll.
 *
 * output parameters
 *
 * @param       pMsg - OSAL message sent to the zapTaskId.
 *
 * @return      None.
 **************************************************************************************************
 */
void zapPhySpiPoll(uint8 port);
void zapPhySpiPoll(uint8 port)
{
#if ZAP_SBL_PROXY
  extern uint8 zapSBL_Active;
#endif
  uint8 pPoll[MT_RPC_FRAME_HDR_SZ];
  
#if ZAP_SBL_PROXY
  if (!HAL_ZNP_SRDY_SET() || zapSBL_Active)
#else
  if (!HAL_ZNP_SRDY_SET())
#endif
  {
    return;
  }
  
  HAL_ZNP_MRDY_SET();  // MRDY must be set before talking to the slave.
  osal_memset(pPoll, 0, MT_RPC_FRAME_HDR_SZ);
  HalSpiWrite(port, pPoll, MT_RPC_FRAME_HDR_SZ);

  if (getSRDY2(port))
  {
    osal_memset(pPoll, 0, MT_RPC_FRAME_HDR_SZ);
    HalSpiWrite(port, pPoll, MT_RPC_FRAME_HDR_SZ);

    if (MT_RPC_CMD_AREQ == (pPoll[MT_RPC_POS_CMD0] & MT_RPC_CMD_TYPE_MASK))
    {
      mtOSALSerialData_t *pMsg = (mtOSALSerialData_t *)osal_msg_allocate(sizeof(mtOSALSerialData_t)
                                                                    + MT_RPC_FRAME_HDR_SZ + *pPoll);
      if (NULL != pMsg)
      {
        pMsg->hdr.event = CMD_SERIAL_MSG;
        pMsg->hdr.status = port;
        pMsg->msg = (uint8 *)(pMsg + 1);

        osal_memcpy(pMsg->msg, pPoll, MT_RPC_FRAME_HDR_SZ);
        if (*pPoll)
        {
          HalSpiWrite(port, pMsg->msg+MT_RPC_POS_DAT0, *pPoll);
        }
        HAL_ZNP_MRDY_CLR();  // Faster release of ZNP with redundant mrdy clear here.

        osal_msg_send(zapTaskId, (uint8 *)pMsg);
      }
      else  // Cannot leave the slave hanging in its "ready-to-send" state.
      {
        HalSpiFlush(port, *pPoll);
      }
    }
  }

  HAL_ZNP_MRDY_CLR();
}

/**************************************************************************************************
 * @fn          zapPhySpiInit
 *
 * @brief       This function initializes the ZAP physical link by SPI.
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
static void zapPhySpiInit(void)
{
  HAL_ZNP_MRDY_CFG();
  HAL_ZNP_SRDY_CFG();
  HAL_ZNP_SRDY1_CFG();
  
  HAL_ZAP_DEBUG_CFG();
  zapPhySpiDly = ZAP_PHY_SPI_WAIT;
}

/**************************************************************************************************
 * @fn          zapPhySpiExec
 *
 * @brief       This function polls the ZAP physical link by SPI.
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
static void zapPhySpiExec(void)
{
}

/**************************************************************************************************
 * @fn          zapPhySpiSend
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
static uint8 zapPhySpiSend(uint8 port, uint8 *pBuf)
{
  pBuf -= MT_RPC_FRAME_HDR_SZ;

  if (MT_RPC_CMD_SREQ == (pBuf[MT_RPC_POS_CMD0] & MT_RPC_CMD_TYPE_MASK))
  {
    if ((port = spiSREQ(port, pBuf)) == FAILURE)
    {
      // Load the failure values.
      pBuf[MT_RPC_POS_LEN] = 1;
      pBuf[MT_RPC_POS_CMD0] |= (uint8)MT_RPC_CMD_SRSP;
      pBuf[MT_RPC_POS_DAT0] = FAILURE;
    }
  }
  else
  {
    port = spiAREQ(port, pBuf);
  }

  return port;
}

/**************************************************************************************************
 * @fn          zapPhySpiSync
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
static void zapPhySpiSync(uint8 port)
{
  (void)port;
}

/**************************************************************************************************
 * @fn          zapPhySpiRun
 *
 * @brief       This function forces the waiting SBL on the ZNP to run.
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP to run.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void zapPhySpiRun(uint8 port)
{
  HalBoardDelay(zapPhySpiDly, FALSE);
  while (!HAL_ZNP_SRDY_SET() && HalBoardDelayed())
  {
    HAL_BOARD_WAIT_MODE();
  }

  if (HAL_ZNP_SRDY_SET())
  {
    /* Should only be used when Serial Bootloader is present
     * Otherwise sending a single byte violates ZNP Protocol.
     * This byte tells the SBL to run the application image
     */
#if 0   
    uint8 ch = SB_FORCE_RUN;
    HAL_ZNP_MRDY_SET();  // MRDY must be set before talking to the slave.
    HalSpiWrite(port, &ch, 1);
    HAL_ZNP_MRDY_CLR();  // MRDY must be cleared before SBL will clear SRDY.
#endif    
    (void)getSRDY2(port);
  }
}

/**************************************************************************************************
 * @fn          zapPhySpiWait
 *
 * @brief       This function sets the SPI wait timeout as specified by the parameter or to the
 *              default value if the parameter is zero.
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
static void zapPhySpiWait(uint8 port, uint16 wait)
{
  (void)port;
  zapPhySpiDly = (wait) ? wait : ZAP_PHY_SPI_WAIT;
}

/**************************************************************************************************
 * @fn          spiSREQ
 *
 * @brief       This function effects a synchronous transaction with the ZNP slave
 *              according to the RPC protocol for the SREQ.
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP to which to send the message.
 * @param       pBuf - Pointer to the buffer to Tx across the SPI.
 *
 * output parameters
 *
 * @param       pBuf - Pointer to the data received across the SPI.
 *
 * @return      SUCCESS or FAILURE.
 **************************************************************************************************
 */
static uint8 spiSREQ(uint8 port, uint8 *pBuf)
{
  uint8 rtrn = FAILURE;

  getSRDY1(port);
  // Send the host SREQ-message across SPI.
  HalSpiWrite(port, pBuf, (*pBuf+MT_RPC_FRAME_HDR_SZ));

  if (getSRDY2(port))
  {
    /* Now setup the POLL command in the buffer which has just been transmitted and which will now
     * be used to receive the SREQ in response.
     */
    *pBuf = 0;                    // POLL command has zero data bytes.
    *(pBuf+1) = MT_RPC_CMD_POLL;  // POLL command MSB.
    *(pBuf+2) = 0;                // POLL command LSB.

    HalSpiWrite(port, pBuf, MT_RPC_FRAME_HDR_SZ);  // Send the POLL across the SPI.

    // Receive the rest of the slave's message if it is longer than the POLL.
    if (*pBuf != 0)
    {
      HalSpiWrite(port, pBuf+MT_RPC_FRAME_HDR_SZ, *pBuf);
    }

    rtrn = SUCCESS;
  }

  HAL_ZNP_MRDY_CLR();  // MRDY must be cleared before setting it again to talk again.
  return rtrn;
}

/**************************************************************************************************
 * @fn          spiAREQ
 *
 * @brief       This function effects an asynchronous transaction with the ZNP slave
 *              according to the RPC protocol for the AREQ.
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP to which to send the message.
 * @param       pBuf - Pointer to the buffer to Tx across the SPI.
 *
 * output parameters
 *
 * None.
 *
 * @return      SUCCESS or FAILURE.
 **************************************************************************************************
 */
static uint8 spiAREQ(uint8 port, uint8 *pBuf)
{
  uint8 rtrn = FAILURE;

  getSRDY1(port);
  // Send the host AREQ-message across SPI.
  HalSpiWrite(port, pBuf, (*pBuf+MT_RPC_FRAME_HDR_SZ));

  if (getSRDY2(port))
  {
    rtrn = SUCCESS;
  }

  HAL_ZNP_MRDY_CLR();  // MRDY must be cleared before setting it again to talk again.
  return rtrn;
}

/**************************************************************************************************
 * @fn          getSRDY1
 *
 * @brief       This function cycles waiting for SRDY_SET & resetting the ZNP, until success.
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP to which to send the message.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void getSRDY1(uint8 port)
{
  do {
    HAL_ZNP_MRDY_SET();  // MRDY must be set before talking to the slave.
    HalBoardDelay(zapPhySpiDly, FALSE);
    while (!HAL_ZNP_SRDY_SET() && HalBoardDelayed())
    {
      HAL_BOARD_WAIT_MODE();
    }

    if (!HAL_ZNP_SRDY_SET())
    {
      zapPhyReset(port);
    }
  } while (!HAL_ZNP_SRDY_SET());
}

/**************************************************************************************************
 * @fn          getSRDY2
 *
 * @brief       This function delays ZAP_PHY_SPI_WAIT time, awaiting the SRDY_CLR.
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP to which to send the message.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the slave signals SRDY_CLR.
 **************************************************************************************************
 */
static uint8 getSRDY2(uint8 port)
{
  uint8 srdy2;
  (void)port;

  HAL_ZNP_SRDY2_CFG();
  HalBoardDelay(zapPhySpiDly, FALSE);
  srdy2 = HAL_ZNP_SRDY_CLR();
  
  while (!srdy2 && HalBoardDelayed())
  {
    HAL_BOARD_WAIT_MODE();
    srdy2 = HAL_ZNP_SRDY_CLR();
  }

  HAL_ZNP_SRDY1_CFG(port);
  return srdy2;
}

/**************************************************************************************************
*/
