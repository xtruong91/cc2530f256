#ifndef POWER_METER_H
#define POWER_METER_H

#ifdef _cplusplus
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
#define POWERENDPOINT                    13

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
extern SimpleDescriptionFormat_t powerMeter_SimpleDesc;

extern CONST zclAttrRec_t powerMeterAttrs[];

/*********************************************************************
 * FUNCTIONS
 */

 /*
  * Initialization for the task
  */
extern void powerMeter_Init( byte task_id );

/*
 *  Event Process for the task
 */
extern UINT16 powerMeterEventLoop( byte task_id, UINT16 events );


/*********************************************************************
*********************************************************************/


#ifdef _cplusplus
}
#endif 
#endif