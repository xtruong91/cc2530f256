#include "DHT11.h"
#include <ioCC2530.h>
#include "OnBoard.h"
#include "hal_adc.h"

//#define DATA_PIN 0x07

uint8_t flag;
uint8_t DATA_PIN;
uint8_t dht11_data[5] = {0,0,0,0,0};
uint8_t dht11_data_temp[5] ={0,0,0,0,0}; 
uint8_t temp_data;
uint8_t data;

/*prototype function */
void RequestRead(void);
void Delay_10us(void);
void Delay_us(unsigned int k);

void Delay_us(unsigned int k)
{ 

    T1CC0L = 0x10;
    T1CC0H = 0x00;
    T1CCTL0 |= (1 << 2);  
    T1CTL = 0x02; 
    //??16M = 16000K = 16000000 ?
    while(k)
    {
        while( ( T1STAT & (1 << 0) ) != 1);
        T1STAT &= ~(1 << 0);
        k--;
    }
    T1CTL = 0x00; //?????
}

void Delay_10us(void)
{
    MicroWait(10);
}
void DelayMs(unsigned int number)
{
    unsigned char i;
    while(number--)
    {
        for(i = 0; i < 100; i++)
            MicroWait(10);
    }    
}

void DHT11_ReadData(uint8_t* buffer, unsigned int length)
{
    DATA_PIN = 0;
    DelayMs(19); // > 18 Ms;
    DATA_PIN = 1;

    P0DIR &=~0x80;    
    Delay_10us();
    Delay_10us();
    Delay_10us();
    Delay_10us();

    dht11_data_temp[0] = dht11_data_temp[1] = dht11_data_temp[2] = dht11_data_temp[3] = dht11_data_temp[4] = 0;
    if(!DATA_PIN)
    {
        flag = 2;
        while((!DATA_PIN) && flag++);
        flag = 2;
        while((DATA_PIN) && flag++);
        int index;
        for(index = 0; index < 5; index++)
        {
            RequestRead();
            dht11_data_temp[index] = data;
        }
        DATA_PIN = 1;
        temp_data = dht11_data_temp[0] + dht11_data_temp[1] + dht11_data_temp[2] + dht11_data_temp[3];
        if(temp_data == dht11_data[4])
        {
            int index;
            for(index = 0; index < 5; index++)
            {
                dht11_data[index] = dht11_data_temp[index];
            }
        }
        buffer[0] = dht11_data[0] / 10;
        buffer[1] = dht11_data[0] % 10;
        buffer[2] = dht11_data[2] / 10;
        buffer[2] = dht11_data[2] % 10;
    }else
    {
        buffer[0] = buffer[1] = buffer[2] = buffer[3];
    }
    P0DIR |= 0x80;
}

void RequestRead()
{
    uint8_t i;
    for(i = 0; i < 8; i++)
    {
        flag = 2;
        while((!DATA_PIN) && flag++);
        Delay_10us();
        Delay_10us();
        Delay_10us();
        temp_data = 0;
        if(DATA_PIN) 
        {
            temp_data = 1;
        }
        flag = 2;
        while((DATA_PIN) && flag++);
        if(flag == 1) break;
        data <<=1;
        data |= temp_data;
    }
}

uint16_t ReadADC()
{
  uint16_t data = 0;
      P0DIR &= ~0x20;  
  asm("NOP");asm("NOP");
  
  /* Clear ADC interrupt flag */
  ADCIF = 0;
  
  ADCCON3 = (0x80 | HAL_ADC_DEC_064 | HAL_ADC_CHANNEL_5);
  
  /* Wait for the conversion to finish */
  while ( !ADCIF );
  
  asm("NOP");asm("NOP");
  
  /* Read the result */
  data = ADCL;
  data |= (int16) (data << 8);
  data >>= 8;
  
  return data;
}
