/**************************************************************************************************
  Filename:       mac_radio_defs.c
  Revised:        $Date: 2007-05-14 10:53:49 -0700 (Mon, 14 May 2007) $
  Revision:       $Revision: 14288 $

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

/* ------------------------------------------------------------------------------------------------
 *                                             Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "mac_radio_defs.h"
#include "hal_types.h"
#include "hal_assert.h"




/* ------------------------------------------------------------------------------------------------
 *                                        Global Constants
 * ------------------------------------------------------------------------------------------------
 */

/* MAC_RUNTIME_CC2591 compile flag is not supported but the compile flag is
 * used for future support. */

#if  defined MAC_RUNTIME_CC2590 || defined MAC_RUNTIME_CC2591 || \
(!defined HAL_PA_LNA && !defined HAL_PA_LNA_CC2590)
/* Power table for CC2520 only with no frontend */
const uint8 CODE macRadioDefsTxPwrBare[] =
{
  5,  /* tramsmit power level of the first entry */
  (uint8)(int8)-14, /* Last entry TX power level */
  /*   5 dBm */   0xF7,   /* characterized value from datasheet */
  /*   4 dBm */   0xF7,
  /*   3 dBm */   0xF2,   /* characterized value from datasheet */
  /*   2 dBm */   0xAB,   /* characterized value from datasheet */
  /*   1 dBm */   0x13,   /* characterized value from datasheet */
  /*   0 dBm */   0x32,   /* characterized value from datasheet */
  /*  -1 dBm */   0x32,
  /*  -2 dBm */   0x81,   /* characterized value from datasheet */
  /*  -3 dBm */   0x88,   /* characterized value from datasheet */
  /*  -4 dBm */   0x88,
  /*  -5 dBm */   0x88,
  /*  -6 dBm */   0x88,
  /*  -7 dBm */   0x2C,   /* characterized value from datasheet */
  /*  -8 dBm */   0x2C,
  /*  -9 dBm */   0x2C,
  /* -10 dBm */   0x2C,
  /* -11 dBm */   0x2C,
  /* -12 dBm */   0x2C,
  /* -13 dBm */   0x2C,
  /* -14 dBm */   0x03   /* characterized value from datasheet */
};
#endif /* !defined HAL_PA_LNA || defined MAC_RUNTIME_CC2591 */

#if defined HAL_PA_LNA || defined MAC_RUNTIME_CC2591
/* Power table for CC2520 + CC2591 */
const uint8 CODE macRadioDefsTxPwrCC2591[] =
{
  17, /* First entry TX power level */
  9, /* Last entry TX power level */
  /*  17 dBm */   0xF9,   /* characterized value at 17 dBm */
  /*  16 dBm */   0xE0,   /* characterized value at 15 dBm */
  /*  15 dBm */   0xE0,   /* characterized value at 13 dBm */
  /*  14 dBm */   0x6C,
  /*  13 dBm */   0x6C,
  /*  12 dBm */   0x79,   /* characterized value at 10 dBm */
  /*  11 dBm */   0x79,
  /*  10 dBm */   0x79,
  /*   9 dBm */   0x49    /* characterized value at 5 dBm  */
};
#endif /* defined HAL_PA_LNA || defined MAC_RUNTIME_CC2591 */


#if defined HAL_PA_LNA_CC2590 || defined MAC_RUNTIME_CC2590
/* Power table for CC2520 + CC2590*/
const uint8 CODE macRadioDefsTxPwrCC2590[] =
{
  11,         /* First entry TX power level */
  5,          /* Last entry TX power level */
  /*  11 dBm */ 0xFA,      /* characterized value at 11.4 dBm */
  /*  10 dBm */ 0xF9,      /* characterized value at 9.7 dBm  */ 
  /*  9 dBm */  0xE9,      /* characterized value at 9.1 dBm  */
  /*  8 dBm */  0xF8,      /* characterized value at 8.4 dBm  */
  /*  7 dBm */  0x81,      /* characterized value at  7.5 dBm */
  /*  6 dBm */  0x88,      /* characterized value at 5.8 dBm, A0 characterized value at 6.5 dBm */
  /*  5 dBm */  0x80,      /* characterized value at 5.3 dBm  */
};
#endif 

#if defined MAC_RUNTIME_CC2591 || defined MAC_RUNTIME_CC2590 || \
  defined HAL_PA_LNA || defined HAL_PA_LNA_CC2590

/* TX power table array */
const uint8 CODE *const CODE macRadioDefsTxPwrTables[] =
{
#if defined MAC_RUNTIME_CC2591 || defined MAC_RUNTIME_CC2590 || \
(!defined HAL_PA_LNA && !defined HAL_PA_LNA_CC2590)
  macRadioDefsTxPwrBare, 
#endif 

#if defined HAL_PA_LNA || defined MAC_RUNTIME_CC2591
  macRadioDefsTxPwrCC2591, 
#endif

#if defined HAL_PA_LNA_CC2590 || defined MAC_RUNTIME_CC2590
/* TX power table array for CC2590*/
   macRadioDefsTxPwrCC2590,
#endif
};
#endif /* if defined MAC_RUNTIME_CC2591 || defined HAL_PA_LNA 
  defined HAL_PA_LNA || defined HAL_PA_LNA_CC2590 */


/**************************************************************************************************
 *                                  Compile Time Integrity Checks
 **************************************************************************************************
 */

#if (MAC_RADIO_RECEIVER_SENSITIVITY_DBM > MAC_SPEC_MIN_RECEIVER_SENSITIVITY)
#error "ERROR: Radio sensitivity does not meet specification."
#endif

#if defined (MAC_RUNTIME_CC2591)
#error "ERROR: MAC_RUNTIME_CC2591 is not supported at this time."
#endif

/**************************************************************************************************
 */
