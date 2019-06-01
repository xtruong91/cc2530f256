#ifndef __DHT11_H__
#define __DHT11_H__

#include "OnBoard.h"

#define MAXTIMING 85

#define HIGH        1
#define LOW         0
#define HAL_ADC_DEC_064     0x00    /* Decimate by 64 : 8-bit resolution */
#define HAL_ADC_DEC_128     0x10    /* Decimate by 128 : 10-bit resolution */
#define HAL_ADC_DEC_256     0x20    /* Decimate by 256 : 12-bit resolution */
#define HAL_ADC_DEC_512     0x30    /* Decimate by 512 : 14-bit resolution */
#define HAL_ADC_DEC_BITS    0x30    /* Bits [5:4] */

#define H_PIN               P0_4
#define G_PIN               P0_5 

typedef unsigned char BOOL;
typedef unsigned char uint8_t;
typedef unsigned int uint16_t;

extern void DHT11_ReadData(uint8_t* buffer, unsigned int length);
extern void DelayMs(unsigned int number);
extern uint16_t ReadADC(void);

#endif