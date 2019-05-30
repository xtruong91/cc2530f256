/* HAL */
#include "hal_uart.h"
#include "OSAL.h"
#include "SH_App.h"
#include "Debug.h"


EndDeviceInfo_t EndDeviceInfos[16];

//@fn rt_str_reverse
//@brief message received is reserved, so we need to reserve again using this function
//@para *tr the first byte in a string that needs reserved
//@para * length the total length that needs to be reserved, 0 included
// e.g a[]={1,2,3,4} then rt_str_reserve(&a[0], 3) result in {4,3,2,1}
void str_reserve(char *str, int length)
{
    char temp, *end_ptr;
    end_ptr = str + length;

    while(end_ptr > str)
    {
        temp = *str;
        *str = *end_ptr;
        *end_ptr = temp;
        str++;
        end_ptr--;
    }
}

void osal_buffer_uint16(unsigned char* buffer, unsigned int data)
{

}

void SHApp_SendTheMessage(unsigned char dest_endID, unsigned char cmd, unsigned int data)
{
    unsigned char theMessageData[6] = "";
    theMessageData[0] = 0xCC; // beginning check byte
    theMessageData[1] = EndDeviceInfos[dest_endID].compressed_addr;
    theMessageData[2] = cmd;

    osal_buffer_uint16(&theMessageData[3], data);
    theMessageData[5] = 0x33; // end check byte;
    // set the destination data;

    GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr64Bit;
    GenericApp_DstAddr.endPoint = EndDeviceInfos[dest_endID].endPoint;
    osal_memcpy(GenericApp_DstAddr.addr.extAddr, EndDeviceInfos[dest_endID].extAddr,8);
    byte  transID = 0;
    if(AF_DataRequest(&GenericApp_DstAddr, &GenericApp_epDesc,
                        SHAPP_CLUSTERID,
                        7, // send one more char or the last char might be missing.
                        theMessageData,
                        &transID,
                        AF_DISCV_ROUTE, AF_DEFAULT_RADIUS) == afStatus_SUCCESS)
    {     
        LREPMaster("\r\n AF_DataRequest: Request to send!. \r\n");
    }
    else
    {
        LREPMaster("\r\n AF_DataRequest:Send failure!. \r\n");
    }
    
}
/*
@fn: SHApp_SerialMSGCB
@brief: process data received by UART, used to execute cmds from slave machine
@param: none
@return: none
*/
void SHApp_SerialMSGCB(void)
{
    unsigned char dest_endID = 0;
    unsigned int data = 0;
    unsigned char buff[7] = "";
    HalUARTRead(0, buff, 6);
    if((buff[1] & SHAPP_ENDPOINT) && (buff[0] == 0xCC) && (buff[5] == 0x33))
    {
        dest_endID = buff[1] & 0x1F; // get destination endPoint from UART message;
        data = osal_build_uint16(&buff[3]);
        HalUARTWrite(0, buff, 6);

        if(buff[2] & 0x80) // cmd for coordinator
        {
            // add cmds here, e.g case 0x81 ...
            switch(buff[2])
            {
                // add cmds here e.g case 0x81
                default:
                    break;
            }
        }
        else
        {
            SHApp_SendTheMessage(dest_endID, buff[2], data);
        }       
    }
    
}