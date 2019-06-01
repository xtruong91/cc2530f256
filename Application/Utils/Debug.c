#include "Debug.h"
#include "OSAL.h"
#include "OSAL_Memory.h"

halUARTCfg_t halUARTConfig;
static void HalUARTCback(uint8 port, uint8 event)
{
}

bool DebugInit()
{
    halUARTConfig.configured = TRUE;
    halUARTConfig.baudRate = HAL_UART_BR_115200;
    halUARTConfig.flowControl = FALSE;
    halUARTConfig.flowControlThreshold = 10; // this parameter indicates number of bytes left before Rx Buffer
                                             // reaches maxRxBufSize
    halUARTConfig.idleTimeout = 6;           // this parameter indicates rx timeout period in millisecond
    halUARTConfig.rx.maxBufSize = BUFFLEN;
    halUARTConfig.tx.maxBufSize = BUFFLEN;
    halUARTConfig.intEnable = TRUE;
    halUARTConfig.callBackFunc = HalUARTCback;
    HalUARTInit();

    if (HalUARTOpen(HAL_UART_PORT_0, &halUARTConfig) == HAL_UART_SUCCESS)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if (0 < vsprintf(string, fmt, argp)) // build string
    {
        HalUARTWrite(HAL_UART_PORT_0, string, strlen(string));
    }
}

void LREPMaster(const char *data)
{
    if (data == NULL)
        return;
    uint16 length = osal_strlen((char *)data) + 1;
    HalUARTWrite(HAL_UART_PORT_0, (uint8 *)data, length);
}

void LREP(char *format, ...)
{
    va_list argp;
    va_start(argp, format);
    vprint(format, argp);
    va_end(argp);
}
