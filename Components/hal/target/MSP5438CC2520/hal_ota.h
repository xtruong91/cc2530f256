/**************************************************************************************************
  Filename:       hal_ota.h
  Revised:        $Date: 2014-05-20 15:47:48 -0700 (Tue, 20 May 2014) $
  Revision:       $Revision: 38600 $

  Description:    This module defines optionally-compiled Boot Code parameters
                  for the MSP5438 platform.

  Copyright 2010-2014 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
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
******************************************************************************/

#ifndef HAL_OTA_H
#define HAL_OTA_H

/******************************************************************************
 * INCLUDES
 */

#include "hal_board_cfg.h"
#include "hal_types.h"

/******************************************************************************
 * MACROS
 */

#if !defined HAL_OTA_BOOT_CODE
#define HAL_OTA_BOOT_CODE  FALSE
#endif

/* ----------- Boot Code Board Init - OTA Support ---------- */
#define HAL_BOOT_INIT()     \
st(                         \
  WDTCTL = WDTPW | WDTHOLD; \
  asm("MOV #5BFCh, SP");    \
)

/*********************************************************************
 * CONSTANTS
 */

// Placement controlled by ota.xcl.
#define HAL_OTA_RC_START           0x5C00
#define HAL_OTA_CRC_ADDR           0x5C00
#define HAL_OTA_CRC_OSET          (HAL_OTA_CRC_ADDR - HAL_OTA_RC_START)

/* Note: these values must align with _SEG values in 'ota.xcl' */
#define HAL_OTA_CHKDL_ADDR         0xFD08
#define HAL_OTA_INVRC_ADDR         0xFDB8
#define HAL_OTA_AVAIL_ADDR         0xFDF4
#define HAL_OTA_READ_ADDR          0xFDFC
#define HAL_OTA_WRITE_ADDR         0xFE6A

/* Note that corresponding changes must be made to ota.xcl when changing the source of Xtra-NV.
 * When using internal flash for XNV, (HAL_OTA_BOOT_PG_CNT + HAL_NV_PAGE_CNT) must be even.
 */
#define HAL_OTA_XNV_IS_INT         TRUE
#define HAL_OTA_XNV_IS_SPI        !HAL_OTA_XNV_IS_INT

#define HAL_OTA_BOOT_PG_CNT        3

/* To reduce the binary image size due to padding un-used code space, reduce HAL_OTA_DL_SIZE
 * to the minimum required for your Application and make the corresponding changes to ota.xcl.
 * This size must be an even multiple of HAL_FLASH_PAGE_SIZE.
 */
#if HAL_OTA_XNV_IS_SPI
#error Only Internal XNV is supported.
#else
#define HAL_OTA_DL_MAX    0x45C00
#define HAL_OTA_DL_OSET   0x27600
// Floor page bound of: ((0x40000-((HAL_NV_PAGE_CNT+HAL_OTA_BOOT_PG_CNT)*HAL_FLASH_PAGE_SIZE)) / 2).
#define HAL_OTA_DL_SIZE   0x1E400
#endif

#define PREAMBLE_OFFSET            0x8C

#define LO_ROM_BEG                 0x5C00
#define LO_ROM_END                 0xC9FF
#define HI_ROM_BEG                 0x10000

#if HAL_OTA_XNV_IS_INT
#define VDD_MIN_OTA  VDD_MIN_NV
#else
#error OTA with external NV on MSP5438 not supported.
#endif

/******************************************************************************
 * TYPEDEFS
 */

typedef enum {
  HAL_OTA_RC,  /* Run code / active image. */
  HAL_OTA_DL   /* Downloaded code to be activated later. */
} image_t;

_Pragma("pack(1)")
typedef struct {
  uint16 crc;
  uint16 crc_shadow;
} otaCrc_t;
_Pragma("pack()")

_Pragma("pack(1)")
typedef struct {
  uint32 programLength;
  uint16 manufacturerId;
  uint16 imageType;
  uint32 imageVersion;
} preamble_t;
_Pragma("pack()")

/******************************************************************************
 * FUNCTIONS
 */

#if !defined ZPORT
#if HAL_OTA_BOOT_CODE
#pragma location="HAL_OTA_CHKDL_SEG"
uint8 HalOTAChkDL(uint8 dlImagePreambleOffset);
#pragma location="HAL_OTA_INVRC_SEG"
void HalOTAInvRC(void);
#pragma location="HAL_OTA_AVAIL_SEG"
uint32 HalOTAAvail(void);
#pragma location="HAL_OTA_READ_SEG"
void HalOTARead(uint32 oset, uint8 *pBuf, uint16 len, image_t type);
#pragma location="HAL_OTA_WRITE_SEG"
void HalOTAWrite(uint32 oset, uint8 *pBuf, uint16 len, image_t type);
#else
uint8 (*HalOTAChkDL)(uint8 dlImagePreambleOffset) =
 (uint8 (*)(uint8 dlImagePreambleOffset))HAL_OTA_CHKDL_ADDR;
void (*HalOTAInvRC)(void) = (void (*)(void))HAL_OTA_INVRC_ADDR;
uint32 (*HalOTAAvail)(void) = (uint32 (*)(void))HAL_OTA_AVAIL_ADDR;
void (*HalOTARead)(uint32 oset, uint8 *pBuf, uint16 len, image_t type) =
  (void (*)(uint32 oset, uint8 *pBuf, uint16 len, image_t type))HAL_OTA_READ_ADDR;
void (*HalOTAWrite)(uint32 oset, uint8 *pBuf, uint16 len, image_t type) =
  (void (*)(uint32 oset, uint8 *pBuf, uint16 len, image_t type))HAL_OTA_WRITE_ADDR;
#endif // ZPORT
#endif // HAL_OTA_BOOT_CODE
#endif // HAL_OTA_H
