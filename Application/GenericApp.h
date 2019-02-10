#ifndef GENERICAPP_H
#define GENERICAPP_H

#include "Config.h"

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
#if defined (ROUTER)
#define GENERICAPP_ENDPOINT           ROUTER_ENDPOINT
#elif defined (END_DEVICE)
#define GENERICAPP_ENDPOINT           DEVICE_ENDPOINT
#else 
//#define GENERICAPP_ENDPOINT           10
#endif



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

#if defined( IAR_ARMCM3_LM )
#define GENERICAPP_RTOS_MSG_EVT       0x0002
#endif  

#define GENERICAPP_CTRL_ENDPOINT        0x55
#define GENERICAPP_MAX_CTRL_CLUSTERS    1
#define GENERICAPP_TIMEOUT_CLUSTER      2

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void GenericApp_Init( byte task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 GenericApp_ProcessEvent( byte task_id, UINT16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GENERICAPP_H */
