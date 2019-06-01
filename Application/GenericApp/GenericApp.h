#ifndef GENERIC_APP_H
#define GENERIC_APP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"

/*********************************************************************
 * CONSTANTS
 */

// These constants are only for example and should be changed to the
// device's needs

#define GENERICAPP_ENDPOINT           10    
#define GENERICAPP_PROFID             0x0F04
#define GENERICAPP_DEVICEID           0x0001
#define GENERICAPP_DEVICE_VERSION     0
#define GENERICAPP_FLAGS              0

#define GENERICAPP_MAX_CLUSTERS       1
#define GENERICAPP_CLUSTERID          1

// Send Message Timeout
#define GENERICAPP_SEND_MSG_TIMEOUT   5000     // Every 5 seconds
#define SEND_MSG_TIMEOUT_DEFAUTL      5000

// Application Events (OSAL) - These are bit weighted definitions.
#define GENERICAPP_SEND_MSG_EVT       0x0001
#define GENERICAPP_UART_RX_EVT        0x0002
   
#if defined( IAR_ARMCM3_LM )
#define GENERICAPP_RTOS_MSG_EVT       0x0002
#endif  

#define GENERICAPP_CTRL_ENDPOINT        0x55
#define GENERICAPP_MAX_CTRL_CLUSTERS    1
#define GENERICAPP_TIMEOUT_CLUSTER      2

/*********************************************************************
 * MACROS
 */

extern const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS];
extern const SimpleDescriptionFormat_t GenericApp_SimpleDesc; 

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
extern endPointDesc_t GenericApp_epDesc;
extern afAddrType_t GenericApp_DstAddr;
extern byte GenericApp_TransID; // This is the unique message ID (counter)
/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void GenericApp_Init( byte task_id );
extern void PCApp_SerialMSGCB(void);
extern void PCApp_SendTheMessage(unsigned char dest_endID, unsigned char cmd, unsigned int temp_set);
extern void PCApp_SendTemperature(void);
extern void str_reserve(char *str, int length);

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 GenericApp_ProcessEvent( byte task_id, UINT16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif