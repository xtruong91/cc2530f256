/**************************************************************************************************
  Filename:       hal_xnv.c
  Revised:        $Date: 2008-01-17 12:32:06 -0800 (Thu, 17 Jan 2008) $
  Revision:       $Revision: 16224 $

  Description:    Code to support HAL OAD by implementing external NV storage.
  Notes:          This version targets the Texas Instruments MSP430FG461x family of processors
                  which have to divide the available flash in half and use the upper half for X-NV.
                  This code must be linked with the HAL OAD boot code, not the application code.

  Copyright 2004-2008 Texas Instruments Incorporated. All rights reserved.

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

/*********************************************************************
 * INCLUDES
 */

#include <msp430.h>

#include "hal_board_cfg.h"

#if (defined HAL_OTA_BOOT_CODE)
#include "hal_ota.h"
#else
#include "hal_oad.h"
#endif //(defined HAL_OTA_BOOT_CODE)

#include "hal_types.h"
#include "hal_xnv.h"

#if (HAL_OAD_XNV_IS_INT || HAL_OTA_XNV_IS_INT)

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      HalXNVRead
 *
 * @brief   Read from the internal flash used as the external NV storage.
 *
 * @param   addr - Offset into the external NV.
 * @param   pBuf - Pointer to the buffer in which to copy the bytes read from external NV.
 * @param   len - Number of bytes to read from external NV.
 *
 * @return  None.
 *********************************************************************/
void HalXNVRead(uint32 addr, uint8 *pBuf, uint16 len)
{
  istate_t ss = __get_interrupt_state();
  __disable_interrupt();

#if (HAL_OTA_XNV_IS_INT)  
  addr += HAL_OTA_DL_OSET;
#else
  addr += HAL_OAD_DL_OSET;
#endif

  while (len--)
  {
    *pBuf++ = __data20_read_char(addr++);
  }

  __set_interrupt_state(ss);
}

/*********************************************************************
 * @fn      HalXNVWrite
 *
 * @brief   Write to the internal flash used as the external NV storage.
 *
 *  NOTE:   Destructive write on page boundary! When writing to the first flash word
 *          of a page boundary, the page is erased without saving/restoring the bytes not written.
 *          Writes anywhere else on a page assume that the location written to has been erased.
 *
 * @param   addr - Offset into the external NV.
 * @param   pBuf - Pointer to the buffer in from which to write bytes to external NV.
 * @param   len - Number of bytes to write to external NV.
 *
 * @return  None.
 *********************************************************************/
void HalXNVWrite(uint32 addr, uint8 *pBuf, uint16 len)
{
  // Save interrupt and watchdog settings.
  istate_t ist = __get_interrupt_state();
  uint8 wdt = WDTCTL & 0xFF;
  WDTCTL = WDTPW + WDTHOLD;            // Stop watchdog per data sheet.
  __disable_interrupt();               // Stop interrupts to insure sequential writes in time.

#if (HAL_OTA_XNV_IS_INT)
  addr += HAL_OTA_DL_OSET;
#else
  addr += HAL_OAD_DL_OSET;
#endif

  while (len)
  {
    if ((addr % HAL_FLASH_PAGE_SIZE) == 0)
    {
      FCTL1 = FWKEY + ERASE;           // Set Erase bit.
      FCTL3 = FWKEY;                   // Clear Lock bit.
      __data20_write_char(addr,0);     // Dummy write to erase Flash segment.
      FCTL1 = FWKEY;
      FCTL3 = FWKEY + LOCK;            // Set LOCK bit.
    }

    if (*pBuf != 0xFF)
    {
      FCTL3 = FWKEY;                   // Clear Lock bit.
      FCTL1 = FWKEY + WRT;             // Set WRT bit for write operation.

      if (addr < 0x10000)
      {
        *(uint8 *)(uint16)addr = *pBuf;
      }
      else
      {
        __data20_write_char(addr, *pBuf);
      }

      FCTL1 = FWKEY;
      FCTL3 = FWKEY + LOCK;            // Set LOCK bit.
    }

    pBuf++;
    addr++;
    len--;
  }

  WDTCTL = WDTPW + wdt;                // Restore watchdog setting.
  __set_interrupt_state(ist);          // Restore interrupts setting.
}

/*********************************************************************
*********************************************************************/

#elif (HAL_OAD_XNV_IS_SPI || HAL_OTA_XNV_IS_SPI)
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define XNV_STAT_CMD  0x05
#define XNV_WREN_CMD  0x06
#define XNV_WRPG_CMD  0x0A
#define XNV_READ_CMD  0x0B

#define XNV_STAT_WIP  0x01


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      xnvSPIWrite
 *
 * @brief   SPI write sequence for code size savings.
 *
 * @param   ch - The byte to write to the SPI.
 *
 * @return  None.
 *********************************************************************/
#pragma location="HAL_XNV_SEG"
static void xnvSPIWrite(uint8 ch);
static void xnvSPIWrite(uint8 ch)
{
  XNV_SPI_TX(ch);
  XNV_SPI_WAIT_RXRDY();
}

/*********************************************************************
 * @fn      HalXNVRead
 *
 * @brief   Read from the external NV storage via SPI.
 *
 * @param   addr - Offset into the external NV.
 * @param   pBuf - Pointer to the buffer in which to copy the bytes read from external NV.
 * @param   len - Number of bytes to read from external NV.
 *
 * @return  None.
 *********************************************************************/
void HalXNVRead(uint32 addr, uint8 *pBuf, uint16 len)
{
  XNV_SPI_BEGIN();
  do {
    xnvSPIWrite(XNV_STAT_CMD);
  } while (XNV_SPI_RX() & XNV_STAT_WIP);
  XNV_SPI_END();
  asm("NOP"); asm("NOP");

  XNV_SPI_BEGIN();
  xnvSPIWrite(XNV_READ_CMD);
  xnvSPIWrite(addr >> 16);
  xnvSPIWrite(addr >> 8);
  xnvSPIWrite(addr);
  xnvSPIWrite(0);

  while (len--)
  {
    xnvSPIWrite(0);
    *pBuf++ = XNV_SPI_RX();
  }
  XNV_SPI_END();
}

/*********************************************************************
 * @fn      HaxXNVWrite
 *
 * @brief   Write to the external NV storage via SPI.
 *
 * @param   addr - Offset into the external NV.
 * @param   pBuf - Pointer to the buffer in from which to write bytes to external NV.
 * @param   len - Number of bytes to write to external NV.
 *
 * @return  None.
 *********************************************************************/
void HalXNVWrite(uint32 addr, uint8 *pBuf, uint16 len)
{
  uint8 cnt;

  while (len)
  {
    XNV_SPI_BEGIN();
    do {
      xnvSPIWrite(XNV_STAT_CMD);
    } while (XNV_SPI_RX() & XNV_STAT_WIP);
    XNV_SPI_END();
    asm("NOP"); asm("NOP");

    XNV_SPI_BEGIN();
    xnvSPIWrite(XNV_WREN_CMD);
    XNV_SPI_END();
    asm("NOP"); asm("NOP");

    XNV_SPI_BEGIN();
    xnvSPIWrite(XNV_WRPG_CMD);
    xnvSPIWrite(addr >> 16);
    xnvSPIWrite(addr >> 8);
    xnvSPIWrite(addr);

    // Can only write within any one page boundary, so prepare for next page write if bytes remain.
    cnt = 0 - (uint8)addr;
    if (cnt)
    {
      addr += cnt;
    }
    else
    {
      addr += 256;
    }

    do
    {
      xnvSPIWrite(*pBuf++);
      cnt--;
      len--;
    } while (len && cnt);
    XNV_SPI_END();
  }
}
#else
#error Invalid Xtra-NV for OTA/OAD.
#endif

/**************************************************************************************************
*/
