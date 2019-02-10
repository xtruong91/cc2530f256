/**************************************************************************************************
  Filename:       hal_adc.c
  Revised:        $Date: 2011-07-01 16:27:08 -0700 (Fri, 01 Jul 2011) $
  Revision:       $Revision: 26552 $

  Description:    This file contains the interface to the HAL ADC.


  Copyright 2007-2011 Texas Instruments Incorporated. All rights reserved.

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

/**************************************************************************************************
 *                                           INCLUDES
 **************************************************************************************************/

#include "hal_types.h"
#include "hal_adc.h"
#include "hal_mcu.h"

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/* TEMP SENSOR defines */
#define HAL_ADC_TS_CHANNEL              10
#define HAL_ADC_TS_CH_BV                BV(10)
#define HAL_ADC_TS_RESULT               ADC12MEM10
#define HAL_ADC_TS_MEM_CONTROL          ADC12MCTL10

/* Vcc defines */
#define HAL_ADC_VCC_CHANNEL             11
#define HAL_ADC_VCC_CH_BV               BV(11)
#define HAL_ADC_VCC_RESULT              ADC12MEM11
#define HAL_ADC_VCC_MEM_CONTROL         ADC12MCTL11

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/
/* Enables/disables the ADC macros */
#define HAL_ADC_ENABLE_12()   { ADC12CTL0 |= ADC12ENC; }
#define HAL_ADC_DISABLE_12()  { ADC12CTL0 &= (ADC12ENC ^ 0xFFFF); }

#define HAL_ADC_START_12()    { ADC12CTL0 |= ADC12SC; }
#define HAL_ADC_STOP_12()     { ADC12CTL0 &= (ADC12SC ^ 0xFFFF); }

/**************************************************************************************************
 *                                      LOCAL FUNCTIONS
 **************************************************************************************************/
uint16 HalAdcReadAccX(void);
void HalAdcReadVccTS(uint16 *temp, uint16 *vcc);
uint16 HalAdcReadVccLevel(void);

/**************************************************************************************************
 * @fn      HalAdcInit
 *
 * @brief   Initialize ADC Service
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalAdcInit (void)
{
  /* Sequence of channels, once, ACLK */
  ADC12CTL0 = ADC12ON + ADC12SHT02 + ADC12MSC + ADC12REFON + ADC12REF2_5V;
  ADC12CTL1 = ADC12SHP + ADC12CONSEQ_1 + ADC12SSEL_1;

  /* Default Resolution is 12 */
  ADC12CTL2 = ADC12RES_2;

  /* Setup memory controls */
  HAL_ADC_TS_MEM_CONTROL = HAL_ADC_TS_CHANNEL;
  HAL_ADC_VCC_MEM_CONTROL = HAL_ADC_VCC_CHANNEL + ADC12EOS;

  /* Delay to stabilize */
  for (uint16 i=0;i<0xFFFF;i++);
}

/**************************************************************************************************
 * @fn      HalAdcRead
 *
 * @brief   Read the ADC based on given channel and resolution
 *
 * @param   channel - channel where ADC will be read
 * @param   resolution - the resolution of the value
 *
 * @return  16 bit value of the ADC in offset binary format.
 **************************************************************************************************/
uint16 HalAdcRead (uint8 channel, uint8 resolution)
{
  uint16 reading = 0;
  uint16 temp=0, vcc=0;

  switch (channel)
  {
    case HAL_ADC_CHANNEL_TEMP:
      HalAdcReadVccTS(&temp, &vcc);
      reading = temp;
      break;

    case HAL_ADC_CHANNEL_VDD:
      HalAdcReadVccTS(&temp, &vcc);
      reading = vcc;
      break;

    default:
      break;
  }

  /* Get Sample */
  switch (resolution)
  {
    case HAL_ADC_RESOLUTION_12:
      break;
    case HAL_ADC_RESOLUTION_8:
      reading >>= 4;
      break;
    case HAL_ADC_RESOLUTION_10:
      reading >>= 2;
      break;
    case HAL_ADC_RESOLUTION_14:
      reading <<= 2;
    default:
      break;
  }

  return (reading);
}

/*********************************************************************
 * @fn       HalAdcCheckVdd
 *
 * @brief    Check for minimum Vdd specified.
 *
 * @param   vdd - The board-specific Vdd reading to check for.
 *
 * @return  TRUE if the Vdd measured is greater than the 'vdd' minimum parameter;
 *          FALSE if not.
 *
 *********************************************************************/
bool HalAdcCheckVdd(uint8 vdd)
{
  ADC12CTL0 = ADC12REFON | ADC12ON;
  ADC12CTL1 = ADC12SHP;
  ADC12CTL2 = 0;
  ADC12MCTL0 = ADC12SREF_1 | ADC12EOS | ADC12INCH_11;

  __delay_cycles(6000);                    // Settling time for reference.

  ADC12CTL0 |= ADC12ENC + ADC12SC;         // Sampling and conversion start.
  while (ADC12IFG == 0);
  ADC12CTL0 = 0;                           // Disable the ADC
  ADC12CTL0 = 0;                           // Turn off reference (must be done AFTER clearing ENC).

  return ((uint8)ADC12MEM0 > vdd);
}

/**************************************************************************************************
 * @fn      HalAdcReadVccTS()
 *
 * @brief   Read the Temperature sensor value and the voltage for the temperature sensor
 *
 * @param   temp - temperature value from temperature sensor
 *          vcc - voltage
 *
 * @return  none.
 *
 **************************************************************************************************/
void HalAdcReadVccTS(uint16 *temp, uint16 *vcc)
{
  ADC12CTL0 = ADC12REFON | ADC12ON | ADC12MSC | ADC12SHT0_8;  // Reference On, ADC On, Multi sample, hold time
  ADC12CTL1 = ADC12SHP + ADC12CONSEQ_1;   // enable sample timer, sequence of channels
  ADC12CTL2 = ADC12RES_2;    // 12 bit resolution
  ADC12MCTL0 = ADC12SREF_1 | ADC12INCH_10;    // ADC input ch A10 => temp sense
  ADC12MCTL1 = ADC12SREF_1 | ADC12INCH_11 | ADC12EOS;   // ADC input ch A11 => Vcc, end of sequence

  ADC12CTL0 |= ADC12ENC + ADC12SC;    // enable and start

  *temp = ADC12MEM0;
  *vcc = ADC12MEM1;

  ADC12CTL0 &= ~ADC12ENC;   // disable ADC
  ADC12CTL0 = 0;    // clear reference
}

/**************************************************************************************************
**************************************************************************************************/
