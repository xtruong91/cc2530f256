/**************************************************************************************************
  Filename:       hal_board_cfg.h
  Revised:        $Date: 2012-09-30 16:36:36 -0700 (Sun, 30 Sep 2012) $
  Revision:       $Revision: 31658 $

  Description:    Declarations for the MSP430F5438.


  Copyright 2006-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
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
**************************************************************************************************/

#ifndef HAL_BOARD_CFG_H
#define HAL_BOARD_CFG_H


/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_assert.h"
#include "hal_dco.h"

/* ------------------------------------------------------------------------------------------------
 *                                       Board Indentifier
 * ------------------------------------------------------------------------------------------------
 */

#define HAL_BOARD_F5438

/* ------------------------------------------------------------------------------------------------
 *                                          Clock Speed
 *
 *   Note that when the HAL_CPU_CLOCK_MHZ is changed, the halBoardSetSystemClock() in
 *   HAL_CLOCK_INIT() macro must also be changed. Also note that the maximum Clock Speed
 *   is 18MHz. 25MHz should not be used until MSP430F5438A part is available.
 * ------------------------------------------------------------------------------------------------
 */
#define HAL_CPU_CLOCK_MHZ     12.0000

/* ------------------------------------------------------------------------------------------------
 *                                          Clock Type
 * ------------------------------------------------------------------------------------------------
 */
#define HAL_CLOCK_TYPE_DCO         0
#define HAL_CLOCK_TYPE_CRYSTAL     1

#if defined (HAL_CLOCK_CRYSTAL)
#define HAL_CLOCK_TYPE             HAL_CLOCK_TYPE_CRYSTAL
#else
#define HAL_CLOCK_TYPE             HAL_CLOCK_TYPE_DCO
#endif


/* ------------------------------------------------------------------------------------------------
 *                                       LED Configuration
 * ------------------------------------------------------------------------------------------------
 */
#define HAL_NUM_LEDS            2
#define HAL_LED_BLINK_DELAY()   st( { volatile uint32 i; for (i=0; i<0x34000; i++) { }; } )

/* LED1 - GREEN */
#define LED1_BV           BV(0)
#define LED1_PORT         P1OUT
#define LED1_DDR          P1DIR

/* LED2 - RED */
#define LED2_BV           BV(1)
#define LED2_PORT         P1OUT
#define LED2_DDR          P1DIR

/* LED3 - not supported - set to LED 1 for now */
#define LED3_BV           LED1_BV
#define LED3_PORT         LED1_PORT
#define LED3_DDR          LED1_DDR

/* LED4 - not supported - set to LED 2 for now */
#define LED4_BV           LED2_BV
#define LED4_PORT         LED2_PORT
#define LED4_DDR          LED2_DDR


/* ------------------------------------------------------------------------------------------------
 *                                    Push Button Configuration
 * ------------------------------------------------------------------------------------------------
 */

#define ACTIVE_LOW        !
#define ACTIVE_HIGH       !!    /* double negation forces result to be '1' */

/* UP */
#define PUSH1_BV          BV(4)
#define PUSH1_PORT        P2IN
#define PUSH1_POLARITY    ACTIVE_LOW

/* RIGHT */
#define PUSH2_BV          BV(2)
#define PUSH2_PORT        P2IN
#define PUSH2_POLARITY    ACTIVE_LOW

/* DOWN */
#define PUSH3_BV          BV(5)
#define PUSH3_PORT        P2IN
#define PUSH3_POLARITY    ACTIVE_LOW

/* LEFT */
#define PUSH4_BV          BV(1)
#define PUSH4_PORT        P2IN
#define PUSH4_POLARITY    ACTIVE_LOW

/* PUSH */
#define PUSH5_BV          BV(3)
#define PUSH5_PORT        P2IN
#define PUSH5_POLARITY    ACTIVE_LOW

/* BUTTON 1 */
#define PUSH6_BV          BV(6)
#define PUSH6_PORT        P2IN
#define PUSH6_POLARITY    ACTIVE_LOW

/* BUTTON 2 */
#define PUSH7_BV          BV(7)
#define PUSH7_PORT        P2IN
#define PUSH7_POLARITY    ACTIVE_LOW


/* ------------------------------------------------------------------------------------------------
 *                                    LCD Configuration
 * ------------------------------------------------------------------------------------------------
 */

/* LCD Max Chars and Buffer */
#define HAL_LCD_MAX_CHARS   16
#define HAL_LCD_MAX_BUFF    25


/* ------------------------------------------------------------------------------------------------
 *                                     Board REV identification
 * ------------------------------------------------------------------------------------------------
 */

#define BOARD_ID_PIN                          BV(7)
#define BOARD_ID_PORT                         P8IN
#define HAL_BOARD_ID_CONFIG()             st( P8DIR &= ~BV(7); P8REN |= BV(7); P8OUT |= BV(7); )
#define HAL_BOARD_ID_DISABLE_PULLUP()     st( P8REN &= ~BV(7); )
#define HAL_MSP_EXP430F5438_REV_02()        ((BOARD_ID_PORT & BOARD_ID_PIN) != 0)
#define HAL_MSP_EXP430F5438_REV_03()        ((BOARD_ID_PORT & BOARD_ID_PIN) == 0)


/* ------------------------------------------------------------------------------------------------
 *                         OSAL NV implemented by internal flash pages.
 * ------------------------------------------------------------------------------------------------
 */

#define HAL_FLASH_PAGE_SIZE        512
#define HAL_FLASH_WORD_SIZE        1

// NV page definitions must coincide with segment declaration in "MSP430F5438.xcl" file
#define HAL_NV_PAGE_END            124
#if defined HAL_OAD_BL21
#define HAL_NV_PAGE_CNT            8
#else
#define HAL_NV_PAGE_CNT            24
#endif
#define HAL_NV_PAGE_BEG           (HAL_NV_PAGE_END-HAL_NV_PAGE_CNT+1)

// The INFOA page:
// RESERVED for future Z-Stack use:    0x1980 - 0x199c
#define HAL_NV_DEVICE_PRIVATE_KEY_ADDR     0x199D
#define HAL_NV_CA_PUBLIC_KEY_ADDR          0x19B2
#define HAL_NV_IMPLICIT_CERTIFICATE_ADDR   0x19C8

#define HAL_NV_IEEE_ADDR                   0x19F8

/* ------------------------------------------------------------------------------------------------
 *                                            Macros
 * ------------------------------------------------------------------------------------------------
 */

#define HAL_CLOCK_INIT(x)                            \
{                                                    \
  /* The DCO library does not handle the stack */    \
  /* frame well. As a result, this macro must */     \
  /* be the first thing that runs after main. */     \
  /* Disable watchdog timer */                       \
  WDTCTL = WDTPW | WDTHOLD;                          \
                                                     \
  if (x == HAL_CLOCK_TYPE_CRYSTAL)                   \
  {                                                  \
    /* Configure external crystal TBD */             \
  }                                                  \
  else if (x == HAL_CLOCK_TYPE_DCO)                  \
  {                                                  \
    /* Note that when halBoardSetSystemClock() */    \
    /* is changed, the HAL_CPU_CLOCK_MHZ must */     \
    /* also be changed. */                           \
    halBoardSetSystemClock(SYSCLK_12MHZ);            \
  }                                                  \
  else                                               \
  {                                                  \
    /* Unknown clock type */                         \
    HAL_ASSERT(0);                                   \
  }                                                  \
                                                     \
  /* CPU15 Errata workaround */                      \
  __bic_SR_register(SCG0);                           \
                                                     \
}

/* ----------- Board Initialization ---------- */
#define HAL_BOARD_INIT()                                         \
{                                                                \
  /* initialize MCU clocks */                                    \
  HAL_CLOCK_INIT(HAL_CLOCK_TYPE);                                \
                                                                 \
  /* reset does not affect GPIO state */                         \
  HAL_TURN_OFF_LED1();                                           \
  HAL_TURN_OFF_LED2();                                           \
  HAL_TURN_OFF_LED3();                                           \
  HAL_TURN_OFF_LED4();                                           \
                                                                 \
  /* set direction for GPIO outputs  */                          \
  LED1_DDR |= LED1_BV;                                           \
  LED2_DDR |= LED2_BV;                                           \
  LED3_DDR |= LED3_BV;                                           \
  LED4_DDR |= LED4_BV;                                           \
                                                                 \
  /* configure board ID GPIO */                                  \
  HAL_BOARD_ID_CONFIG();                                         \
}

/* ----------- Debounce ---------- */
#define HAL_DEBOUNCE(expr)    { int i; for (i=0; i<500; i++) { if (!(expr)) i = 0; } }

/* ----------- Push Buttons ---------- */
#define HAL_PUSH_BUTTON1()        (PUSH1_POLARITY (PUSH1_PORT & PUSH1_BV))
#define HAL_PUSH_BUTTON2()        (PUSH2_POLARITY (PUSH2_PORT & PUSH2_BV))
#define HAL_PUSH_BUTTON3()        (PUSH3_POLARITY (PUSH3_PORT & PUSH3_BV))
#define HAL_PUSH_BUTTON4()        (PUSH4_POLARITY (PUSH4_PORT & PUSH4_BV))
#define HAL_PUSH_BUTTON5()        (PUSH5_POLARITY (PUSH5_PORT & PUSH5_BV))
#define HAL_PUSH_BUTTON6()        (PUSH6_POLARITY (PUSH6_PORT & PUSH6_BV))
#define HAL_PUSH_BUTTON7()        (PUSH7_POLARITY (PUSH7_PORT & PUSH7_BV))

/* ----------- LED's ---------- */
#define HAL_TURN_OFF_LED1()       st( LED1_PORT &= ~LED1_BV; )
#define HAL_TURN_OFF_LED2()       st( LED2_PORT &= ~LED2_BV; )
#define HAL_TURN_OFF_LED3()       st( LED3_PORT &= ~LED3_BV; )
#define HAL_TURN_OFF_LED4()       st( LED4_PORT &= ~LED4_BV; )

#define HAL_TURN_ON_LED1()        st( LED1_PORT |=  LED1_BV; )
#define HAL_TURN_ON_LED2()        st( LED2_PORT |=  LED2_BV; )
#define HAL_TURN_ON_LED3()        st( LED3_PORT |=  LED3_BV; )
#define HAL_TURN_ON_LED4()        st( LED4_PORT |=  LED4_BV; )

#define HAL_TOGGLE_LED1()         st( LED1_PORT ^=  LED1_BV; )
#define HAL_TOGGLE_LED2()         st( LED2_PORT ^=  LED2_BV; )
#define HAL_TOGGLE_LED3()         st( LED3_PORT ^=  LED3_BV; )
#define HAL_TOGGLE_LED4()         st( LED4_PORT ^=  LED4_BV; )

#define HAL_STATE_LED1()          (LED1_PORT & LED1_BV)
#define HAL_STATE_LED2()          (LED2_PORT & LED2_BV)

/* ----------- Minimum safe bus voltage ---------- */

// Vdd/2 / Internal Reference X ENOB --> (Vdd / 2) / 1.5 X 255
#define VDD_2_0  170  // 2.0 V required to safely read/write internal flash.

#define VDD_MIN_RUN  VDD_2_0
#define VDD_MIN_NV  (VDD_2_0+9)  // 5% margin over minimum to survive a page erase and compaction.

/* ------------------------------------------------------------------------------------------------
 *                                     Driver Configuration
 * ------------------------------------------------------------------------------------------------
 */

/* Set to TRUE enable H/W TIMER usage, FALSE disable it */
#ifndef HAL_TIMER
#define HAL_TIMER FALSE
#endif

/* Set to TRUE enable ADC usage, FALSE disable it */
#ifndef HAL_ADC
#define HAL_ADC TRUE
#endif

/* Set to TRUE enable LCD usage, FALSE disable it */
#ifndef HAL_LCD
#define HAL_LCD TRUE
#endif

/* Set to TRUE enable LED usage, FALSE disable it */
#ifndef HAL_LED
#define HAL_LED TRUE
#endif
#if (!defined BLINK_LEDS) && (HAL_LED == TRUE)
#define BLINK_LEDS
#endif

/* Set to TRUE enable KEY usage, FALSE disable it */
#ifndef HAL_KEY
#define HAL_KEY TRUE
#endif

/* Set to TRUE enable UART usage, FALSE disable it */
#ifndef HAL_UART
#if (defined ZAPP_P1) || (defined ZAPP_P2) || (defined ZTOOL_P1) || (defined ZTOOL_P2)
#define HAL_UART TRUE
#else
#define HAL_UART FALSE
#endif
#endif


/* ------------------------------------------------------------------------------------------------
 *                                    Interrupt Configuration
 * ------------------------------------------------------------------------------------------------
 */

/* ----------- timer interrupts ---------- */
#define INTERRUPT_TIMERB_OC_CC0()     HAL_ISR_FUNCTION( haBoardTimerB0Isr, TIMERB0_VECTOR )
#define INTERRUPT_TIMERB_OC_CC1_6()   HAL_ISR_FUNCTION( haBoardTimerB1Isr, TIMERB1_VECTOR )

/* ----------- UART interrupts ---------- */
#define INTERRUPT_UART()    HAL_ISR_FUNCTION( halBoardUart1Isr, USCI_A1_VECTOR /*USCIA1_VECTOR*/ )

/* ----------- key interrupts ---------- */
#define INTERRUPT_KEYBD()             HAL_ISR_FUNCTION( halBoardPort1Isr, PORT2_VECTOR )


#endif
/*******************************************************************************************************
*/
