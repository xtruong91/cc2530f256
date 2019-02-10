/**************************************************************************************************
    Filename:       zap_phy_uart.c
    Revised:        $Date: 2010-12-01 15:31:18 -0800 (Wed, 01 Dec 2010) $
    Revision:       $Revision: 24529 $

    Description:

    This file defines the functionality of the ZNP Application Processor.
    This file declares the functionality of the ZAP Physical Link Layer by UART.


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
#include "hal_drivers.h"
#include "hal_uart.h"
#include "mt.h"
#include "mt_rpc.h"
#include "mt_uart.h"
#include "OSAL_Clock.h"
#include "OSAL_Memory.h"
#include "zap_app.h"
#include "zap_phy.h"
#include "ZComDef.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

#if !defined ZAP_PHY_UART_PORT0
#define ZAP_PHY_UART_PORT0                 0
#endif

// Hook for supporting 2 ZNP's on 2 different UART's.
#if !defined ZAP_PHY_UART_PORT_CNT
#define ZAP_PHY_UART_PORT_CNT              1
#endif

#if !defined ZAP_PHY_UART_WAIT
#define ZAP_PHY_UART_WAIT                  2000
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

typedef enum {
  SOP,
  LEN,
  CMD0,
  CMD1,
  DATA,
  FCS
} zapPhyUartState_t;

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

static uint8 zapPhyUartSRspF[ZAP_PHY_UART_PORT_CNT];
static uint8 zapPhyUartSyncF[ZAP_PHY_UART_PORT_CNT];
static uint8 zapPhyUartIdx[ZAP_PHY_UART_PORT_CNT];
static uint8 zapPhyUartLen[ZAP_PHY_UART_PORT_CNT];

static uint8 *zapPhyUartSreq[ZAP_PHY_UART_PORT_CNT];
static uint8 *zapPhyUartAreq[ZAP_PHY_UART_PORT_CNT];

static zapPhyUartState_t zapPhyUartState[ZAP_PHY_UART_PORT_CNT];

static uint16 zapPhyUartDly;

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static void zapPhyUartCB(uint8 port, uint8 event);
static uint8 zapPhyUartFcs(uint8 *pBuf, uint8 len);

#if !ZAP_ZNP_MT
/**************************************************************************************************
 * @fn          zapPhyUartExec
 *
 * @brief       This function manages the ZAP physical link by UART.
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
static void zapPhyUartExec(void)
{
}
#endif

/**************************************************************************************************
 * @fn          zapPhyUartInit
 *
 * @brief       This function initializes the ZAP physical link by UART.
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
static void zapPhyUartInit(void)
{
  halUARTCfg_t uartConfig;

  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = HAL_UART_BR_115200;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 256;
  uartConfig.rx.maxBufSize        = 512;
  uartConfig.tx.maxBufSize        = 512;
  uartConfig.idleTimeout          = 1;
  uartConfig.intEnable            = TRUE;
  uartConfig.callBackFunc         = zapPhyUartCB;

  HalUARTOpen(zapAppPort, &uartConfig);
  zapPhyUartSRspF[zapAppPort] = TRUE;
  zapPhyUartDly = ZAP_PHY_UART_WAIT;
}

/**************************************************************************************************
 * @fn          zapPhyUartSend
 *
 * @brief       This function sends an RPC message buffer created with zap_msg_allocate().
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP to which to send the message.
 * @param       pBuf - The buffer pointer returned by zap_msg_allocate().
 *
 * output parameters
 *
 * None.
 *
 * @return      SUCCESS or FAILURE.
 **************************************************************************************************
 */
static uint8 zapPhyUartSend(uint8 port, uint8 *pBuf)
{
  const uint8 len = *(pBuf - MT_RPC_FRAME_HDR_SZ + MT_RPC_POS_LEN);
  const uint16 sz = (uint16)len + MT_RPC_FRAME_HDR_SZ + 2;
  uint8 rtrn = FAILURE;

  pBuf[len] ^= zapPhyUartFcs(pBuf, len);

  if (MT_RPC_CMD_SREQ == (*(pBuf-MT_RPC_FRAME_HDR_SZ+MT_RPC_POS_CMD0) & MT_RPC_CMD_TYPE_MASK))
  {
    uint16 cnt;

    if ((zapPhyUartSyncF[port]) &&  // Block SREQ's until an AREQ sync is established with the ZNP.
        (zapPhyUartSRspF[port]))    // Prevent nested SREQ's.
    {
      cnt = HalUARTWrite(port, (pBuf - MT_RPC_FRAME_HDR_SZ - 1), sz);
    }
    else
    {
      cnt = 0;
    }

    // Setup for failure.
    zapPhyUartSreq[port] = pBuf - MT_RPC_FRAME_HDR_SZ;
    zapPhyUartSreq[port][MT_RPC_POS_LEN] = 1;
    zapPhyUartSreq[port][MT_RPC_POS_CMD0] |= (uint8)MT_RPC_CMD_SRSP;
    zapPhyUartSreq[port][MT_RPC_POS_DAT0] = FAILURE;

    if (cnt == sz)
    {
      zapPhyUartSRspF[port] = FALSE;
      HalBoardDelay(zapPhyUartDly, FALSE);
      do  // Block, waiting an arbitrary/reasonable time for the SRSP by UART transport.
      {
        osalTimeUpdate();
        Hal_ProcessPoll();
      } while ((FALSE == zapPhyUartSRspF[port]) && HalBoardDelayed());

      if (zapPhyUartSRspF[port] == FALSE)
      {
        zapPhyUartSRspF[port] = TRUE;
        zapPhyUartSyncF[port] = FALSE;
        zapLostSync(port);
      }
      else
      {
        rtrn = SUCCESS;
      }
    }

    zapPhyUartSreq[port] = NULL;
  }
  else
  {
    if (HalUARTWrite(port, (pBuf - MT_RPC_FRAME_HDR_SZ - 1), sz) == sz)
    {
      rtrn = SUCCESS;
    }
  }

  return rtrn;
}

#if !ZAP_ZNP_MT
/**************************************************************************************************
 * @fn          zapPhyUartSync
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
static void zapPhyUartSync(uint8 port)
{
  zapPhyUartSyncF[port] = TRUE;
}

/**************************************************************************************************
 * @fn          zapPhyUartRun
 *
 * @brief       This function forces the waiting SBL to run.
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
static void zapPhyUartRun(uint8 port)
{
  uint8 ch = SB_FORCE_RUN;

  (void)HalUARTWrite(port, &ch, 1);
  (void)HalUARTWrite(port, &ch, 1);
}

/**************************************************************************************************
 * @fn          zapPhyUartWait
 *
 * @brief       This function sets the UART wait timeout as specified by the parameter or to the
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
static void zapPhyUartWait(uint8 port, uint16 wait)
{
  (void)port;
  zapPhyUartDly = (wait) ? wait : ZAP_PHY_UART_WAIT;
}
#endif

/**************************************************************************************************
 * @fn          zapPhyUartCB
 *
 * @brief       This function receives the callback for the ZAP physical link by UART.
 *
 * input parameters
 *
 * @param       port - UART port
 * @param       event - Event that causes the callback
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void zapPhyUartCB(uint8 port, uint8 event)
{
  uint8 ch;
  (void)event;

  while (HalUARTRead(port, &ch, 1))
  {
    switch (zapPhyUartState[port])
    {
    case SOP:
      if (MT_UART_SOF == ch)
      {
        zapPhyUartState[port] = LEN;
      }
      break;

    case LEN:
      zapPhyUartState[port] = CMD0;
      zapPhyUartLen[port] = ch;
      zapPhyUartIdx[port] = 0;
      break;

    case CMD0:
#if ZAP_ZNP_MT
      zapPhyUartAreq[port] = zap_msg_allocate(zapPhyUartLen[port], ch, 0);
      if (NULL != zapPhyUartAreq[port])
      {
        // zap_msg_allocate() returns a pointer to the data but the following logic is expecting
        // a pointer to the message.
        zapPhyUartAreq[port] -= MT_RPC_FRAME_HDR_SZ;
      }
#else
      if (MT_RPC_CMD_SRSP == (ch & MT_RPC_CMD_TYPE_MASK))
      {
        zapPhyUartAreq[port] = zapPhyUartSreq[port];
      }
      else
      {
        mtOSALSerialData_t *pMsg = (mtOSALSerialData_t *)osal_msg_allocate(
                           sizeof(mtOSALSerialData_t) + MT_RPC_FRAME_HDR_SZ + zapPhyUartLen[port]);

        if (NULL != pMsg)
        {
          pMsg->hdr.event = CMD_SERIAL_MSG;
          pMsg->hdr.status = port;
          pMsg->msg = (uint8 *)(pMsg + 1);
          zapPhyUartAreq[port] = pMsg->msg;
        }
        else
        {
          zapPhyUartAreq[port] = NULL;
        }
      }
#endif

      // If getting an unexpected SRSP (should not happen), or no memory for an AREQ.
      if (NULL == zapPhyUartAreq[port])
      {
        zapPhyUartState[port] = SOP;
      }
      else
      {
        zapPhyUartState[port] = CMD1;
        zapPhyUartAreq[port][MT_RPC_POS_LEN] = zapPhyUartLen[port];
        zapPhyUartAreq[port][MT_RPC_POS_CMD0] = ch;
      }
      break;

    case CMD1:
      zapPhyUartAreq[port][MT_RPC_POS_CMD1] = ch;

      if (0 == zapPhyUartLen[port])  // If there is no data, skip to FCS state.
      {
        zapPhyUartState[port] = FCS;
      }
      else
      {
        zapPhyUartState[port] = DATA;
      }
      break;

    case DATA:
      zapPhyUartAreq[port][MT_RPC_POS_DAT0 + zapPhyUartIdx[port]++] = ch;

      if (zapPhyUartIdx[port] != zapPhyUartLen[port])
      {
        uint16 rxAvail = Hal_UART_RxBufLen(port);

        if (rxAvail <= (zapPhyUartLen[port] - zapPhyUartIdx[port]))
        {
          HalUARTRead(port, &(zapPhyUartAreq[port][MT_RPC_POS_DAT0 + zapPhyUartIdx[port]]), rxAvail);
          zapPhyUartIdx[port] += rxAvail;
        }
        else
        {
          HalUARTRead(port, &(zapPhyUartAreq[port][MT_RPC_POS_DAT0 + zapPhyUartIdx[port]]),
                             (zapPhyUartLen[port] - zapPhyUartIdx[port]));
          zapPhyUartIdx[port] = zapPhyUartLen[port];
        }
      }

      if (zapPhyUartIdx[port] == zapPhyUartLen[port])
      {
        zapPhyUartState[port] = FCS;
      }
      break;

    case FCS:
      zapPhyUartState[port] = SOP;

      if ((zapPhyUartAreq[port][MT_RPC_POS_LEN]  ^
           zapPhyUartAreq[port][MT_RPC_POS_CMD0] ^
           zapPhyUartAreq[port][MT_RPC_POS_CMD1] ^
           zapPhyUartFcs((zapPhyUartAreq[port] + MT_RPC_POS_DAT0), zapPhyUartLen[port]) == ch))
      {
#if ZAP_ZNP_MT
        // zapPhyAction-N() funcs expect a pointer to the data, so add MT_RPC_FRAME_HDR_SZ.
        uint8 *pBuf = zapPhyUartAreq[port] + MT_RPC_FRAME_HDR_SZ;

#if ZAP_APP_MSG
        if (MT_APP_MSG == zapPhyUartAreq[port][MT_RPC_POS_CMD1])
        {
          mtOSALSerialData_t *pMsg = (mtOSALSerialData_t *)osal_msg_allocate(
                           sizeof(mtOSALSerialData_t) + MT_RPC_FRAME_HDR_SZ + zapPhyUartLen[port]);

          if (NULL != pMsg)
          {
            pMsg->hdr.event = CMD_SERIAL_MSG;
            pMsg->hdr.status = port;
            pMsg->msg = (uint8 *)(pMsg + 1);
            (void)osal_memcpy(pMsg->msg, zapPhyUartAreq[port],
                                         MT_RPC_FRAME_HDR_SZ + zapPhyUartLen[port]);
            osal_msg_send(zapTaskId, (uint8 *)pMsg);
          }
        }
#endif

        zapPhySend(zapAppPort, pBuf);
        if (MT_RPC_CMD_SRSP == (zapPhyUartAreq[port][MT_RPC_POS_CMD0] & MT_RPC_CMD_TYPE_MASK))
        {
          uint8 len = zapPhyUartAreq[port][MT_RPC_POS_LEN];
          // Pre-seed the FCS for UART transport.
          *(pBuf + len) = len ^ zapPhyUartAreq[port][MT_RPC_POS_CMD0] ^
                                zapPhyUartAreq[port][MT_RPC_POS_CMD1];
          zapPhyUartSend(zapAppPort, pBuf);
        }
#else
        if (MT_RPC_CMD_SRSP == (zapPhyUartAreq[port][MT_RPC_POS_CMD0] & MT_RPC_CMD_TYPE_MASK))
        {
          zapPhyUartSRspF[port] = TRUE;
        }
        else
        {
          osal_msg_send(zapTaskId, (uint8 *)(((mtOSALSerialData_t *)zapPhyUartAreq[port]) - 1));
        }
      }
      else if (MT_RPC_CMD_AREQ == (zapPhyUartAreq[port][MT_RPC_POS_CMD0] & MT_RPC_CMD_TYPE_MASK))
      {
        osal_msg_deallocate((uint8 *)(((mtOSALSerialData_t *)zapPhyUartAreq[port]) - 1));
#endif
      }

#if ZAP_ZNP_MT
      zapPhyUartAreq[port] += MT_RPC_FRAME_HDR_SZ;
      zap_msg_deallocate(&zapPhyUartAreq[port]);
#endif
      break;

    default:
      break;
    }
  }
}

/**************************************************************************************************
 * @fn          zapPhyUartFcs
 *
 * @brief       This function calculates the FCS according to the parameters passed.
 *
 * input parameters
 *
 * @param       pBuf - A buffer pointer to data to calculate FCS.
 * @param       len - Length of the data in pBuf.
 *
 * output parameters
 *
 * None.
 *
 * @return      The FCS calculated.
 **************************************************************************************************
 */
static uint8 zapPhyUartFcs(uint8 *pBuf, uint8 len)
{
  uint8 idx, fcs = 0;

  for (idx = 0; idx < len; idx++)
  {
    fcs ^= pBuf[idx];
  }

  return fcs;
}

/**************************************************************************************************
*/
