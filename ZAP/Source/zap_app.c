/**************************************************************************************************
    Filename:       zap_app.c
    Revised:        $Date: 2013-05-16 17:29:25 -0700 (Thu, 16 May 2013) $
    Revision:       $Revision: 34342 $

    Description:

    This file defines the functionality of the ZNP Application Processor.


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
#include "hal_key.h"
#include "hal_lcd.h"
#include "hal_led.h"
#include "mt.h"
#include "mt_rpc.h"
#include "mt_sys.h"
#include "mt_uart.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "sapi.h"
#include "zap_app.h"
#include "zap_phy.h"
#include "zap_znp.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZComDef.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

#if !defined ZAP_APP_PORT
#define ZAP_APP_PORT  0
#endif

#if !defined ZAP_APP_LED
#define ZAP_APP_LED   FALSE
#endif

#if !defined ZAP_APP_KEYS
#define ZAP_APP_KEYS  TRUE
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

typedef void (*zapProcessFunc_t)(uint8 port, uint8 *pBuf);

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

uint8 zapTaskId;

// Hook for supporting more than 1 ZNP on different ports.
// An example of use would be to set this to the desired port before invoking AF_DataRequest().
uint8 zapAppPort;

// Count of the global ZNP variables that are cached locally on ZAP.
#define ZAP_MON_INFO_CNT  5
// IEEE Address (64-bit Extended Address) of the ZNP device.
uint8 znpIEEE[8];
// ZigBee Network Address of the ZNP device.
uint16 znpAddr;
// ZigBee Network Address of the parent of the ZNP device.
uint16 znpParent;
// ZigBee Network PanId.
uint16 znpPanId;
// ZNP variable indicating the current device state (from Z-Stack ZDO/ZD_App.c).
devStates_t devState;

#if SECURE
// ZNP variable read from NV at powerup by the ZGlobals manager - TODO: how to sync value with ZNP?
uint8 zgSecurityMode = ZG_SECURITY_MODE;
#endif


/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

static const zapProcessFunc_t zapProcessFunc[] =
{
  NULL,

#if defined (ZAP_SYS_FUNC)
  zapSysProcessIncoming,
#else
  NULL,
#endif

#if defined (ZAP_MAC_FUNC)
  zapMacProcessIncoming,
#else
  NULL,
#endif

#if defined (ZAP_NWK_FUNC)
  zapNwkProcessIncoming,
#else
  NULL,
#endif

#if defined (ZAP_AF_FUNC)
  zapAfProcessIncoming,
#else
  NULL,
#endif

#if defined (ZAP_ZDO_FUNC)
  zapZdoProcessIncoming,
#else
  NULL,
#endif

#if defined (ZAP_SAPI_FUNC)
  zapSapiProcessIncoming,
#else
  NULL,
#endif

#if defined (ZAP_UTIL_FUNC)
  zapUtilProcessIncoming,
#else
  NULL,
#endif

#if defined (ZAP_DEBUG_FUNC)
  zapDebugProcessIncoming,
#else
  NULL,
#endif

#if defined (ZAP_APP_FUNC)
  zapApsProcessIncoming,
#else
  NULL,
#endif
};

#ifdef LCD_SUPPORTED
static uint16 zapDisAddr;
static uint16 zapDisPanId;
static uint8  zapDisIEEE[Z_EXTADDR_LEN];
static uint8  zapDisState;
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

#if ZAP_ZNP_MT
#include "zap_phy_uart.c"
#if ZAP_APP_MSG
#define MT_APP_FUNC
#include "MT_App.c"
#endif
#endif

#ifdef LCD_SUPPORTED
static void zapDisInfo(void);
#endif
static void zapMonInfo(void);
#if ZAP_APP_KEYS
static void zapKeys(keyChange_t *msg);
#endif
static void zapMonitor(void);
static void zapSync(void);
static void zapSysEvtMsg(void);

/**************************************************************************************************
 * @fn          zapInit
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
void zapInit(uint8 id)
{
  zapTaskId = id;
  zapAppPort = ZAP_APP_PORT;

  zapPhyInit();
#if ZAP_APP_KEYS
  RegisterForKeys(id);
#endif

  zapLostSync(ZAP_APP_PORT);
  if (ZSuccess != osal_start_timerEx(zapTaskId, ZAP_APP_TMR_EVT, ZAP_APP_TMR_DLY))
  {
    (void)osal_set_event(zapTaskId, ZAP_APP_TMR_EVT);
  }

#if !ZAP_PHY_RESET_ZNP
  uint8 *pBuf;
  if ((pBuf = zap_msg_allocate(0, (uint8)MT_RPC_SYS_UTIL | (uint8)MT_RPC_CMD_AREQ,
                                  (uint8)MT_UTIL_SYNC_REQ)) != NULL)
  {
    zapPhySend(zapAppPort, pBuf);
    zap_msg_deallocate(&pBuf);
  }
#endif

#if ZAP_ZNP_MT
  zapPhyUartInit();
#endif
}

/**************************************************************************************************
 * @fn          zapEvt
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
uint16 zapEvt(uint8 id, uint16 evts)
{
  uint16 mask = 0;
  (void)id;

  if (evts & SYS_EVENT_MSG)
  {
    mask = SYS_EVENT_MSG;
    zapSysEvtMsg();
  }
  else if (evts & (ZAP_PHY_SPI_EVT | ZAP_PHY_UART_EVT))
  {
    mask = evts & (ZAP_PHY_SPI_EVT | ZAP_PHY_UART_EVT);
    zapPhyExec(mask);
  }
  else if (evts & ZAP_APP_TMR_EVT)
  {
    mask = ZAP_APP_TMR_EVT;
    zapMonitor();
  }
  else if (evts & ZAP_APP_SYNC_EVT)
  {
    mask = ZAP_APP_SYNC_EVT;
    zapSync();
  }
  else if (evts & ZAP_APP_ZDO_STATE_CHANGE_EVT)
  {
    mask = ZAP_APP_ZDO_STATE_CHANGE_EVT;

#ifdef LCD_SUPPORTED
    // Loop to get all of the cached, global ZNP variables for the sake of a timely LCD update.
    for (id = 0; id <= ZAP_MON_INFO_CNT; id++)
    {
      zapMonInfo();
    }
#else
    // Otherwise, just get what is needed for the ZDO state change update.
    zb_GetDeviceInfo(ZB_INFO_DEV_STATE, &devState);
#endif

    ZDO_UpdateNwkStatus(devState);
  }
  else
  {
    mask = evts;  // Discard unknown events - should never happen.
  }

  return (evts ^ mask);  // Return unprocessed events.
}

/**************************************************************************************************
 * @fn          zapGotSync
 *
 * @brief       This function is invoked upon receipt of MT_SYS_RESET_IND or MT_UTIL_SYNC_REQ
 *              and would indicate that sync is (re-)gained with the ZNP.
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP that got sync.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zapGotSync(uint8 port)
{
  zapPhySync(port);

  // Especially for UART transport, allow time for multiple got syncs before acting on it.
  if (ZSuccess != osal_start_timerEx(zapTaskId, ZAP_APP_SYNC_EVT, ZAP_APP_SYNC_DLY))
  {
    (void)osal_set_event(zapTaskId, ZAP_APP_SYNC_EVT);
  }
}

/**************************************************************************************************
 * @fn          zapLostSync
 *
 * @brief       This function is invoked by the PHY drivers when sync is lost with the ZNP.
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP that lost sync.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zapLostSync(uint8 port)
{
  devState = DEV_STATE_INVALID;
  znpAddr = znpParent = INVALID_NODE_ADDR;
  znpPanId = 0xFFFF;
}

/**************************************************************************************************
 * @fn          zapProcessIncoming
 *
 * @brief       This function is called by zapSysEvtMsg() to process an incoming message from ZNP.
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP that sent the message.
 * @param       pBuf - Pointer to event message.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zapProcessIncoming(uint8 port, uint8 *pBuf)
{
  if ((pBuf[MT_RPC_POS_CMD0] & MT_RPC_SUBSYSTEM_MASK) < MT_RPC_SYS_MAX)
  {
    zapProcessFunc_t func;

    if (NULL != (func = zapProcessFunc[pBuf[MT_RPC_POS_CMD0] & MT_RPC_SUBSYSTEM_MASK]))
    {
      func(port, pBuf);
    }
  }

#if ZAP_ZNP_MT
#if ZAP_APP_MSG
  if ((MT_RPC_SYS_APP == (pBuf[MT_RPC_POS_CMD0] & MT_RPC_SUBSYSTEM_MASK)) &&
          (MT_APP_MSG ==  pBuf[MT_RPC_POS_CMD1]))
  {
    MT_AppMsg(pBuf);
  }
  else
#endif
  {
    MT_BuildAndSendZToolResponse(pBuf[MT_RPC_POS_CMD0], pBuf[MT_RPC_POS_CMD1],
                                 pBuf[MT_RPC_POS_LEN],  pBuf+MT_RPC_POS_DAT0);
  }
#endif
}

/**************************************************************************************************
 * @fn          zap_msg_allocate
 *
 * @brief       This function allocates and pre-fills a dynamic RPC message buffer.
 *
 * input parameters
 *
 * @param       len - length of the data to send via RPC (i.e. not including RPC overhead.)
 * @param       cmd0 - a valid logical OR of the mtRpcCmdType_t & mtRpcSysType_t.
 * @param       cmd1 - a valid command for the mtRpcSysType_t.
 *
 * output parameters
 *
 * None.
 *
 * @return      NULL on failure to allocate all memory necessary.
 *              A pointer to the data area (size 'len') of a valid dynamic memory buffer on success.
 **************************************************************************************************
 */
uint8 *zap_msg_allocate(uint8 len, uint8 cmd0, uint8 cmd1)
{
  // An SREQ buffer must be big enough to accept an SRSP up to the max allowable size.
  const uint8 sz = (MT_RPC_CMD_SREQ == (cmd0 & MT_RPC_CMD_TYPE_MASK)) ? MT_RPC_DATA_MAX : len;
  // Add space for the RPC frame header and UART transport SOP and FCS.
  uint8 *pBuf = (uint8 *)osal_mem_alloc(sz + MT_RPC_FRAME_HDR_SZ + 2);

  if (NULL != pBuf)
  {
    // Pre-seed the SOP for UART transport.
    *pBuf++ = MT_UART_SOF;
    *pBuf++ = len;
    *pBuf++ = cmd0;
    *pBuf++ = cmd1;
    #if (3 != MT_RPC_FRAME_HDR_SZ)
    #error Need to port RPC frame header changes here.
    #endif
    // Pre-seed the FCS for UART transport.
    *(pBuf + len) = len ^ cmd0 ^ cmd1;
  }

  return pBuf;
}

/**************************************************************************************************
 * @fn          zap_msg_deallocate
 *
 * @brief       This function deallocates an RPC message buffer created with zap_msg_allocate().
 *
 * input parameters
 *
 * @param       ppBuf - A pointer to the buffer pointer returned by zap_msg_allocate().
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zap_msg_deallocate(uint8 **ppBuf)
{
  uint8 *pBuf;

  HAL_ASSERT(ppBuf);

  if (NULL == *ppBuf)
  {
    return;
  }

  pBuf = *ppBuf - (MT_RPC_FRAME_HDR_SZ + 1);
  *ppBuf = NULL;
  osal_mem_free((void *)pBuf);
}

#ifdef LCD_SUPPORTED
/**************************************************************************************************
 * @fn          zapDisInfo
 *
 * @brief       This displays the IEEE (MSB to LSB) and Network State & Address on the LCD whenever
 *              a change in the value is detected.
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
static void zapDisInfo(void)
{
  uint8 i;
  uint8 *xad;
  uint8 lcd_buf[Z_EXTADDR_LEN*2+1];

  if (!osal_memcmp(zapDisIEEE, znpIEEE, Z_EXTADDR_LEN))
  {
    (void)osal_memcpy(zapDisIEEE, znpIEEE, Z_EXTADDR_LEN);
    xad = znpIEEE + Z_EXTADDR_LEN - 1;

    for (i = 0; i < Z_EXTADDR_LEN*2; xad--)
    {
      uint8 ch;
      ch = (*xad >> 4) & 0x0F;
      lcd_buf[i++] = ch + (( ch < 10 ) ? '0' : '7');
      ch = *xad & 0x0F;
      lcd_buf[i++] = ch + (( ch < 10 ) ? '0' : '7');
    }
    lcd_buf[Z_EXTADDR_LEN*2] = '\0';
    HalLcdWriteString((char *)lcd_buf, HAL_LCD_LINE_1);
  }

  if ((zapDisState != devState) || (zapDisAddr != znpAddr))
  {
    zapDisState = devState;
    zapDisAddr = znpAddr;

    switch (devState)
    {
    case DEV_END_DEVICE:
      HalLcdWriteStringValue("End Device", znpAddr, 16, HAL_LCD_LINE_2);
      break;

    case DEV_ROUTER:
      HalLcdWriteStringValue("ZigBee Rtr", znpAddr, 16, HAL_LCD_LINE_2);
      break;

    case DEV_ZB_COORD:
      HalLcdWriteStringValue("ZigBee Coord", znpAddr, 16, HAL_LCD_LINE_2);
      break;

    default:
      HalLcdWriteStringValue("Other Inval", znpAddr, 16, HAL_LCD_LINE_2);
      break;
    }
  }

  if (zapDisPanId != znpPanId)
  {
    zapDisPanId = znpPanId;
    HalLcdWriteStringValue("PanId 0x", znpPanId, 16, HAL_LCD_LINE_3);
  }
}
#endif

/**************************************************************************************************
 * @fn          zapMonInfo
 *
 * @brief       This function is invoked by zapMonitor during steady state in order to maintain
 *              the local copies of ZNP-side global variables.
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
static void zapMonInfo(void)
{
  static uint8 getInfoIdx;

  switch (getInfoIdx++)
  {
  case 0:
    zb_GetDeviceInfo(ZB_INFO_DEV_STATE, &devState);
    break;
  case 1:
    zb_GetDeviceInfo(ZB_INFO_IEEE_ADDR, &znpIEEE);
    break;
  case 2:
    zb_GetDeviceInfo(ZB_INFO_SHORT_ADDR, &znpAddr);
    break;
  case 3:
    zb_GetDeviceInfo(ZB_INFO_PARENT_SHORT_ADDR, &znpParent);
    break;
  case 4:
    zb_GetDeviceInfo(ZB_INFO_PAN_ID, &znpPanId);
    break;
  // Update ZAP_MON_INFO_CNT whenever another case is added.
  default:
#ifdef LCD_SUPPORTED
    zapDisInfo();
#endif
    getInfoIdx = 0;
    break;
  }
}

#if ZAP_APP_KEYS
/**************************************************************************************************
 * @fn          zapKeys
 *
 * @brief       This function is called by zapSysEvtMsg() to process a key(s) event.
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
static void zapKeys(keyChange_t *msg)
{
  const uint8 keys = msg->keys;

  if (msg->state)  // Shift key.
  {
    if (keys & HAL_KEY_SW_1)
    {
    }
    if (keys & HAL_KEY_SW_2)
    {
    }
    if (keys & HAL_KEY_SW_3)
    {
    }
    if (keys & HAL_KEY_SW_4)
    {
    }
  }
  else
  {
    if (keys & HAL_KEY_SW_1)
    {
    }
    if (keys & HAL_KEY_SW_2)
    {
    }
    if (keys & HAL_KEY_SW_3)
    {
    }
    if (keys & HAL_KEY_SW_4)
    {
    }
  }
}
#endif

/**************************************************************************************************
 * @fn          zapMonitor
 *
 * @brief       This function is called by OSAL timer or event to monitor the ZNP for all.
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
static void zapMonitor(void)
{
  /* If the ZNP has not already reported by the first monitor run, only the ZAP has reset and not
   * the ZNP; or the ZAP/ZNP sync has been lost.
   */
  if (DEV_STATE_INVALID == devState)
  {
#if ZAP_APP_LED
    HalLedSet (HAL_LED_2, HAL_LED_MODE_OFF);
#endif
    devState = DEV_STATE_SYNC_LOST;
    zapUtilReq(MT_UTIL_SYNC_REQ, NULL, NULL);
  }
  else if (DEV_STATE_SYNC_LOST == devState)
  {
    devState = DEV_STATE_ZNP_LOST;
    znpSystemReset(ZNP_RESET_SOFT);
  }
  else if (DEV_STATE_ZNP_LOST == devState)
  {
    devState = DEV_STATE_ZAP_LOST;
    zapPhyReset(zapAppPort);
  }
  else if (DEV_STATE_ZAP_LOST == devState)
  {
    HalReset();
  }
  else
  {
#if ZAP_APP_LED
    HalLedSet (HAL_LED_2, HAL_LED_MODE_ON);
#endif
    zapMonInfo();
  }

  if (ZSuccess != osal_start_timerEx(zapTaskId, ZAP_APP_TMR_EVT, ZAP_APP_TMR_DLY))
  {
    (void)osal_set_event(zapTaskId, ZAP_APP_TMR_EVT);
  }

#if ZAP_APP_LED
  HalLedSet (HAL_LED_4, HAL_LED_MODE_TOGGLE);
#endif
}

/**************************************************************************************************
 * @fn          zapSync
 *
 * @brief       This function is invoked upon receipt of ZAP_APP_SYNC_EVT.
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
static void zapSync(void)
{
  uint8 pBuf[Z_EXTADDR_LEN];

#if ZAP_NV_RESTORE
  pBuf[0] = ZCD_STARTOPT_AUTO_START;
  (void)znp_nv_write(ZCD_NV_STARTUP_OPTION, 0, 1, pBuf);
#else
  (void)osal_memset(pBuf, 0, Z_EXTADDR_LEN);
  if (osal_memcmp(pBuf, znpIEEE, Z_EXTADDR_LEN))
  {
    zb_GetDeviceInfo(ZB_INFO_IEEE_ADDR, &znpIEEE);
    pBuf[0] = ZCD_STARTOPT_CLEAR_STATE | ZCD_STARTOPT_CLEAR_CONFIG;
    (void)znp_nv_write(ZCD_NV_STARTUP_OPTION, 0, 1, pBuf);
    zapPhyReset(zapAppPort);
    return;
  }
#endif

  zb_GetDeviceInfo(ZB_INFO_DEV_STATE, &devState);
  zapAfSync();
  zapZdoSync();
#if !ZAP_PHY_RESET_ZNP
  // If ZAP resets while a connected ZNP is left running, the ZNP cannot be expected to issue a
  // ZDO_STATE_CHANGE notification.
  if ((DEV_END_DEVICE == devState) || (DEV_ROUTER == devState) || (DEV_ZB_COORD == devState))
  {
    // Especially for UART transport, allow time for multiple got syncs before acting on it.
    if (ZSuccess != osal_start_timerEx(zapTaskId, ZAP_APP_ZDO_STATE_CHANGE_EVT,
                                                  ZAP_APP_ZDO_STATE_CHANGE_DLY))
    {
      (void)osal_set_event(zapTaskId, ZAP_APP_ZDO_STATE_CHANGE_EVT);
    }
  }
#endif
#if ZAP_AUTO_CFG
  if (devState < DEV_END_DEVICE)
  {
    // Configure the defaults from zap.cfg into the ZNP.
    pBuf[0] = ZAP_DEVICETYPE;
    (void)znp_nv_write(ZCD_NV_LOGICAL_TYPE, 0, 1, pBuf);

    pBuf[0] = LO_UINT16(ZDAPP_CONFIG_PAN_ID);
    pBuf[1] = HI_UINT16(ZDAPP_CONFIG_PAN_ID);
    (void)znp_nv_write(ZCD_NV_PANID, 0, 2, pBuf);

    pBuf[0] = BREAK_UINT32(DEFAULT_CHANLIST, 0);
    pBuf[1] = BREAK_UINT32(DEFAULT_CHANLIST, 1);
    pBuf[2] = BREAK_UINT32(DEFAULT_CHANLIST, 2);
    pBuf[3] = BREAK_UINT32(DEFAULT_CHANLIST, 3);
    (void)znp_nv_write(ZCD_NV_CHANLIST, 0, 4, pBuf);
  }
#endif
#if defined TC_LINKKEY_JOIN
  zapCertSync();
#endif
#if ZAP_AUTO_START
  if ((DEV_HOLD == devState) || (DEV_INIT == devState) || (DEV_NWK_ORPHAN == devState))
  {
    (void)ZDOInitDevice(NWK_START_DELAY);
  }
#endif
  zb_GetDeviceInfo(ZB_INFO_DEV_STATE, &devState);
}

/**************************************************************************************************
 * @fn          zapSysEvtMsg
 *
 * @brief       This function is called by zapEvt() to process all of the pending OSAL messages.
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
static void zapSysEvtMsg(void)
{
  uint8 *msg;

  while ((msg = osal_msg_receive(zapTaskId)))
  {
    switch (*msg)
    {
    case CMD_SERIAL_MSG:
      zapProcessIncoming(((mtOSALSerialData_t *)msg)->hdr.status, ((mtOSALSerialData_t *)msg)->msg);
      break;

#if ZAP_APP_KEYS
    case KEY_CHANGE:
      zapKeys((keyChange_t *)msg);
      break;
#endif

    default:
      break;
    }

    (void)osal_msg_deallocate(msg);  // Receiving task is responsible for releasing the memory.
  }
}

#if ZAP_ZNP_MT
/**************************************************************************************************
 * @fn          MT_BuildAndSendZToolResponse
 *
 * @brief       This function is the ZAP proxy to the ZNP SystemReset() functionality.
 *
 * input parameters
 *
 * @param       cmdType - include type and subsystem
 * @param       cmdId - command ID
 * @param       dataLen
 * @param       *pData
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void MT_BuildAndSendZToolResponse(uint8 cmdType, uint8 cmdId, uint8 dataLen, uint8 *pData)
{
  uint8 *pBuf = zap_msg_allocate(dataLen, cmdType, cmdId);

  if (NULL != pBuf)
  {
    (void)osal_memcpy(pBuf, pData, dataLen);
    zapPhyUartSend(zapAppPort, pBuf);
    zap_msg_deallocate(&pBuf);
  }
}
#endif

/**************************************************************************************************
*/
