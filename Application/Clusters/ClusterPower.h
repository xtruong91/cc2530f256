#ifndef CLUSTER_POWER_H
#define CLUSTER_POWER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "zcl_general.h"
#include "zcl.h"
#include "ClusterOSALEvent.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */
#define POWER_ATTRIBUTES  \
	{ ZCL_CLUSTER_ID_GEN_POWER_CFG, {  ATTRID_POWER_CFG_MAINS_VOLTAGE,  ZCL_DATATYPE_UINT16, ACCESS_CONTROL_READ, (void *)&mainVoltage, NULL  }  },  \
	{ ZCL_CLUSTER_ID_GEN_POWER_CFG, {  ATTRID_POWER_CFG_BATTERY_VOLTAGE,  ZCL_DATATYPE_UINT8, ACCESS_CONTROL_READ, (void *)&batteryVoltage, NULL  }  },  \
	{ ZCL_CLUSTER_ID_GEN_POWER_CFG, {  ATTRID_POWER_CFG_BAT_ALARM_MASK,  ZCL_DATATYPE_BITMAP8, ACCESS_CONTROL_READ, (void *)&batteryAlarmMask, NULL  }  },
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */
extern uint16 mainVoltage;
extern uint16 batteryVoltage;
extern uint8  batteryAlarmMask;

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
*********************************************************************/


#ifdef __cplusplus
}
#endif

#endif