/**************************************************************************************************
    Filename:       zap_sbl.c
    Revised:        $Date: 2010-07-28 17:08:02 -0700 (Wed, 28 Jul 2010) $
    Revision:       $Revision: 23200 $

    Description:

    This file implements a ZAP Sample Application to load a new image onto the ZNP via its SBL.


    Copyright 2010 Texas Instruments Incorporated. All rights reserved.

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
#include "hal_lcd.h"
#include "hal_spi.h"
#include "hal_uart.h"
#include "OSAL_Tasks.h"
#include "sb_exec.h"
#include "zap_app.h"
#include "zap_phy.h"
#include "zap_sbl.h"
#include "ZComDef.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

// The SBL image installation will be driven externally via a UART, probably to a PC Tool.
// Otherwise the ZAP will need some fileIO to read and packetize the image itself. 
#if !defined ZAP_SBL_EXT_EXEC
#define ZAP_SBL_EXT_EXEC  1
#endif
#if !defined ZAP_SBL_EXT_PORT
#define ZAP_SBL_EXT_PORT  0
#endif

//efine SYS_EVENT_MSG     0x8000
#define ZAP_SBL_EVT_EXIT  0x4000

#define ZAP_SBL_DLY_EXIT  15000
#define ZAP_SBL_DLY_WAIT  100

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

#if ZAP_ZNP_MT || ZAP_APP_MSG
#error Must have ZAP_ZNP_MT && ZAP_APP_MSG defined as TRUE for this SBL sample app.
#endif

#if ZAP_PHY_SPI
#define sbBuf  zapSBL_Buf
#endif

/* ------------------------------------------------------------------------------------------------
 *                                         External Variables
 * ------------------------------------------------------------------------------------------------
 */

#if ZAP_PHY_UART
extern halUARTCfg_t uartRecord;
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

uint8 zapSBL_TaskId;
uint8 zapSBL_Active;

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

static uint8 sbFcs, sbIdx, sbLen, sbSte;

#if ZAP_PHY_SPI
static uint8 *zapSBL_Buf;
#endif

#if ZAP_PHY_UART
static halUARTCBack_t usurpedCB;
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static void zapSBL_SysEvtMsg(void);
#if ZAP_PHY_SPI
static uint8 zapSBL_GetSRDY(void);
static uint8 zapSBL_GetSRDY2(void);
static void zapSBL_ResetZNP(void);
#endif
#if ZAP_SBL_EXT_EXEC
static void zapSBL_RxExt(uint8 port, uint8 event);
#else
#error Need File I/O to retrieve the new image for the ZNP.
#endif

/**************************************************************************************************
 * @fn          zapSBL_Init
 *
 * @brief       This function is the application's task initialization.
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
void zapSBL_Init(uint8 id)
{
#if ZAP_SBL_EXT_EXEC
  halUARTCfg_t uartConfig;

  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = HAL_UART_BR_115200;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 16;                // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = 254;               // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = 254;               // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = 1;                 // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = zapSBL_RxExt;
  HalUARTOpen(ZAP_SBL_EXT_PORT, &uartConfig);
#endif

  zapSBL_TaskId = id;
}

/**************************************************************************************************
 * @fn          zapSBL_Evt
 *
 * @brief       This function is called to process the OSAL events for the task.
 *
 * input parameters
 *
 * @param       id - OSAL task Id.
 * @param       evts - OSAL events bit mask of pending events.
 *
 * output parameters
 *
 * None.
 *
 * @return      evts - OSAL events bit mask of unprocessed events.
 **************************************************************************************************
 */
uint16 zapSBL_Evt(uint8 id, uint16 evts)
{
  uint16 mask = 0;
  (void)id;
  
  if (evts & SYS_EVENT_MSG)
  {
    mask = SYS_EVENT_MSG;
    zapSBL_SysEvtMsg();
  }
  else if (evts & ZAP_SBL_EVT_EXIT)
  {
    mask = ZAP_SBL_EVT_EXIT;
    HalReset();
  }
  else
  {
    mask = evts;  // Discard unknown events - should never happen.
  }

  return (evts ^ mask);  // Return unprocessed events.
}

/**************************************************************************************************
 * @fn          zapSBL_SysEvtMsg
 *
 * @brief       This function is called by zapSBL_Evt() to process all pending OSAL messages.
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
static void zapSBL_SysEvtMsg(void)
{
  uint8 *msg;

  while ((msg = osal_msg_receive(zapSBL_TaskId)))
  {
    (void)osal_msg_deallocate(msg);  // Receiving task is responsible for releasing the memory.
  }
}

#if ZAP_PHY_SPI
/**************************************************************************************************
 * @fn          zapSBL_Rx
 *
 * @brief       This function drives an Rx from the ZNP. When the ZNP SBL is in "Rx mode" and has
 *              nothing to transmit, it will be transmitting only zeroes.
 *              So the expected simplistic lock-step sequence:
 *              MRDY Set - SRDY Set
 *              Rx an expected complete SBL message
 *              MRDY Clr - SRDY Clr
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
static void zapSBL_Rx(void)
{
  uint8 len, err = TRUE;

  zapSBL_GetSRDY();
  osal_memset(zapSBL_Buf, 0, 2);

  HalSpiWrite(zapAppPort, zapSBL_Buf, 1);

  if (zapSBL_Buf[0] == SB_SOF)
  {
    HalSpiWrite(zapAppPort, zapSBL_Buf+1, 1);
    if ((len = zapSBL_Buf[1]) <= SB_BUF_SIZE-SB_FCS_STATE)
    {
      HalSpiWrite(zapAppPort, zapSBL_Buf+2, len+3);  // CMD1/2 & FCS.
      err = FALSE;
    }

#if ZAP_SBL_EXT_EXEC
    (void)HalUARTWrite(ZAP_SBL_EXT_PORT, zapSBL_Buf, len+SB_FCS_STATE);  // SOF, LEN, CMD1/2 & FCS.
#else
#error Need to parse the response and act accordingly.
#endif
  }

  HAL_ZNP_MRDY_CLR();
  if (err)
  {
    zapSBL_ResetZNP();
  }
}

/**************************************************************************************************
 * @fn          zapSBL_Tx
 *
 * @brief       This function sends SBL messages to the ZNP across the SPI transport.
 *              Simplistic lock-step sequence:
 *              MRDY Set - SRDY Set
 *              Tx a complete SBL message which must result in an SBL response
 *              MRDY Clr - SRDY Clr
 *              Pause for ZNP processing of the SBL message
 *              Rx the expected SBL response
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
static void zapSBL_Tx(void)
{
  uint8 len = zapSBL_Buf[SB_LEN_STATE];

  zapSBL_GetSRDY();
  HalSpiWrite(zapAppPort, zapSBL_Buf, len+SB_FCS_STATE);  // SOF, LEN, CMD1/2 & FCS.

  if (zapSBL_GetSRDY2())
  {
    zapSBL_Rx();
  }
}

/**************************************************************************************************
 * @fn          zapSBL_GetSRDY
 *
 * @brief       This function sets MRDY and waits for SRDY, resetting the ZNP if necessary.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if got SRDY_SET() on first try (i.e. without having to reset the ZNP).
 **************************************************************************************************
 */
static uint8 zapSBL_GetSRDY(void)
{
  uint8 rtrn = TRUE;

  do {
    HAL_ZNP_MRDY_SET();  // MRDY must be set before talking to the slave.
    HalBoardDelay(ZAP_SBL_DLY_WAIT, FALSE);
    while (!HAL_ZNP_SRDY_SET() && HalBoardDelayed())
    {
      HAL_BOARD_WAIT_MODE();
    }

    if (!HAL_ZNP_SRDY_SET())
    {
      zapSBL_ResetZNP();
      rtrn = FALSE;
    }
  } while (!HAL_ZNP_SRDY_SET());

  return rtrn;
}

/**************************************************************************************************
 * @fn          zapSBL_GetSRDY2
 *
 * @brief       This function clears MRDY and waits for SRDY clear, resetting the ZNP if necessary.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if got SRDY_CLR().
 **************************************************************************************************
 */
static uint8 zapSBL_GetSRDY2(void)
{
  HAL_ZNP_MRDY_CLR();  // MRDY must be set before talking to the slave.

  HalBoardDelay(ZAP_SBL_DLY_WAIT, FALSE);
  while (!HAL_ZNP_SRDY_CLR() && HalBoardDelayed())
  {
    HAL_BOARD_WAIT_MODE();
  }

  return HAL_ZNP_SRDY_CLR();
}

/**************************************************************************************************
 * @fn          zapSBL_ResetZNP
 *
 * @brief       This function resets the ZNP and forces it into its SBL.
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
static void zapSBL_ResetZNP(void)
{
  HAL_ZNP_MRDY_CLR();

  do {
    HAL_ZNP_RST();
    MicroWait(100);
    HAL_ZNP_RUN();

    HalBoardDelay(ZAP_SBL_DLY_WAIT, FALSE);
    while (!HAL_ZNP_SRDY_SET() && HalBoardDelayed())
    {
      HAL_BOARD_WAIT_MODE();
    }
  } while (!HAL_ZNP_SRDY_SET());

  zapSBL_Buf[0] = SB_FORCE_BOOT;
  HAL_ZNP_MRDY_SET();  // MRDY must be set before talking to the slave.
  HalSpiWrite(zapAppPort, zapSBL_Buf, 1);
  HAL_ZNP_MRDY_CLR();
}

#if ZAP_SBL_EXT_EXEC
/**************************************************************************************************
 * @fn          zapSBL_RxExt
 *
 * @brief       This function is the registered callback for the UART to the external application
 *              that is driving the serial boot load to the ZNP.
 *
 * input parameters
 *
 * @param       port - Don't care.
 * @param       event - Don't care.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void zapSBL_RxExt(uint8 port, uint8 event)
{
  uint8 ch;

  (void)port;
  (void)event;

  // Use external UART Rx as the trigger to start an SBL session.
  if (!zapSBL_Active)
  {
    if ((zapSBL_Buf = osal_mem_alloc(SB_BUF_SIZE)) == NULL)
    {
      return;
    }

    zapSBL_Active = TRUE;
    zapSBL_ResetZNP();

    // Stop other tasks access (e.g. zapMonitor()) to SPI when in SBL mode.
    (void)osal_memset(tasksEvents+1, 0, ((tasksCnt-1) * sizeof(uint16)));
    for (ch = 1; ch < tasksCnt; ch++)
    {
      uint16 evt;

      for (evt = 0x0001; evt < 0x8000; evt <<= 1)
      {
        (void)osal_stop_timerEx(ch, evt);
      }
    }

    if (ZSuccess != osal_start_timerEx(zapSBL_TaskId, ZAP_SBL_EVT_EXIT, ZAP_SBL_DLY_EXIT))
    {
      (void)osal_set_event(zapSBL_TaskId, ZAP_SBL_EVT_EXIT);
    }

#if HAL_LCD
    HalLcdWriteString( "TexasInstruments", 1 );
    HalLcdWriteString( "ZAP Proxy to ZNP", 2 );
    HalLcdWriteString( "   BootLoader   ", 3 );
#endif
  }

  while (HalUARTRead(ZAP_SBL_EXT_PORT, &ch, 1))
  {
    sbBuf[sbSte + sbIdx] = ch;
    switch (sbSte)
    {
    case SB_SOF_STATE:
      if (SB_SOF == ch)
      {
        sbSte = SB_LEN_STATE;
      }
      break;
    
    case SB_LEN_STATE:
      sbFcs = 0;
      sbSte = ((sbLen = ch) >= SB_BUF_SIZE) ? SB_SOF_STATE : SB_CMD1_STATE;
      break;

    case SB_CMD1_STATE:
      sbSte = SB_CMD2_STATE;
      break;
    
    case SB_CMD2_STATE:
      sbSte = (sbLen) ? SB_DATA_STATE : SB_FCS_STATE;
      break;

    case SB_DATA_STATE:
      if (++sbIdx == sbLen)
      {
        sbSte = SB_FCS_STATE;
      }
      break;
    
    case SB_FCS_STATE:
      if (sbFcs == ch)
      {
        sbBuf[SB_DATA_STATE + sbIdx] = ch;
        zapSBL_Tx();
      }
      else
      {
        // TODO - RemoTI did not have here or on bad length - adding could cause > 1 SB_INVALID_FCS
        //        for a single data packet which could put out of sync with PC for awhile or
        //        infinte, depending on PC-side?
        // sbResp(SB_INVALID_FCS, 1);
      }
    
      sbSte = sbIdx = 0;
      break;
    
    default:
      break;
    }
    sbFcs ^= ch;

    if (ZSuccess != osal_start_timerEx(zapSBL_TaskId, ZAP_SBL_EVT_EXIT, ZAP_SBL_DLY_EXIT))
    {
      (void)osal_set_event(zapSBL_TaskId, ZAP_SBL_EVT_EXIT);
    }
  }
}
#endif
#endif

#if ZAP_PHY_UART
/**************************************************************************************************
 * @fn          zapSBL_Rx
 *
 * @brief       This function is temporarily set as the registered callback for the UART to the ZNP
 *              in order to complete a serial boot load.
 *
 * input parameters
 *
 * @param       port - Don't care.
 * @param       event - Don't care.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void zapSBL_Rx(uint8 port, uint8 event)
{
  uint8 ch;

  (void)port;
  (void)event;

  while (HalUARTRead(ZAP_SBL_ZNP_PORT, &ch, 1))
  {
    (void)HalUARTWrite(ZAP_SBL_EXT_PORT, &ch, 1);
  }

  /*
   * TODO: Need to design a way to terminate the SBL session and return back to normal ZAP
   *       operations. Among possible other things, need to restore the usurped UART callback.
   */
  //uartRecord.callBackFunc = usurpedCB;
  //zapSBL_Active = FALSE;
}

/**************************************************************************************************
 * @fn          zapSBL_RxExt
 *
 * @brief       This function is the registered callback for the UART to the external application
 *              that is driving the serial boot load to the ZNP.
 *
 * input parameters
 *
 * @param       port - Don't care.
 * @param       event - Don't care.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void zapSBL_RxExt(uint8 port, uint8 event)
{
  uint8 ch;

  (void)port;
  (void)event;

  // Use external UART Rx as the trigger to start an SBL session.
  if (!zapSBL_Active)
  {
    // Usurp and save to restore the currently registered UART transport callback.
    usurpedCB = uartRecord.callBackFunc;
    uartRecord.callBackFunc = zapSBL_Rx;

    znpSystemReset(ZNP_RESET_SOFT);
    HalBoardDelay(1000, TRUE);

    ch = SB_FORCE_BOOT;
    (void)HalUARTWrite(ZAP_SBL_ZNP_PORT, &ch, 1);
    (void)HalUARTWrite(ZAP_SBL_ZNP_PORT, &ch, 1);

    zapSBL_Active = TRUE;

    if (ZSuccess != osal_start_timerEx(zapSBL_TaskId, ZAP_SBL_EVT_EXIT, ZAP_SBL_DLY_EXIT))
    {
      (void)osal_set_event(zapSBL_TaskId, ZAP_SBL_EVT_EXIT);
    }
  }

  while (HalUARTRead(ZAP_SBL_EXT_PORT, &ch, 1))
  {
    (void)HalUARTWrite(ZAP_SBL_ZNP_PORT, &ch, 1);

    if (ZSuccess != osal_start_timerEx(zapSBL_TaskId, ZAP_SBL_EVT_EXIT, ZAP_SBL_DLY_EXIT))
    {
      (void)osal_set_event(zapSBL_TaskId, ZAP_SBL_EVT_EXIT);
    }
  }
}
#endif

/**************************************************************************************************
*/
