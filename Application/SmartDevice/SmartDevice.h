#ifndef SMART_DEVICE_H
#define SMART_DEVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "zcl.h"

/*********************************************************************
 * CONSTANTS
 */
#define ENDPOINT            14

#define LIGHT_OFF                       0x00
#define LIGHT_ON                        0x01

/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */
extern SimpleDescriptionFormat_t zclSampleLight_SimpleDesc;

extern CONST zclAttrRec_t smartDeviceAttrs[];

extern uint16 zclSampleLight_IdentifyTime;

/*********************************************************************
 * FUNCTIONS
 */

 /*
  * Initialization for the task
  */
extern void SmartDevice_Init( byte task_id );

/*
 *  Event Process for the task
 */
extern UINT16 SmartDeviceEventLoop( byte task_id, UINT16 events );
/*********************************************************************
*********************************************************************/


#ifdef __cplusplus
}
#endif

#endif