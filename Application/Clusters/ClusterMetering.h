#ifndef CLUSTER_METERING_H
#define CLUSTER_METERING_H

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
#include "GenericType.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */
#define ATTRID_METERING_CURRENT_SUMMATION_DELIVERED 0
#define ATTRID_METERING_STATUS_ATTRIBUTE 0x0200
#define ATTRID_METERING_UNIT_OF_MEASURE 0x0300
#define ATTRID_METERING_MULTIPLIER 0x0301
#define ATTRID_METERING_DIVISOR 0x0302
#define ATTRID_METERING_SUMMATION_FORMATTING 0x0303
#define ATTRID_METERING_DEMAND_FORMATTING 0x0304
#define ATTRID_METERING_HISTORICAL_CONSUMPTION_FORMATTING 0x0305
#define ATTRID_METERING_METERING_DEVICE_TYPE 0x0306
#define METERING_ATTRIBUTES  \
	{ ZCL_CLUSTER_ID_SE_SIMPLE_METERING, {  ATTRID_METERING_CURRENT_SUMMATION_DELIVERED,  ZCL_DATATYPE_UINT48, ACCESS_CONTROL_READ, (void *)&currentSummationDelivered, NULL  }  },  \
	{ ZCL_CLUSTER_ID_SE_SIMPLE_METERING, {  ATTRID_METERING_STATUS_ATTRIBUTE,  ZCL_DATATYPE_BITMAP8, ACCESS_CONTROL_READ, (void *)&status, NULL  }  },  \
	{ ZCL_CLUSTER_ID_SE_SIMPLE_METERING, {  ATTRID_METERING_UNIT_OF_MEASURE,  ZCL_DATATYPE_ENUM8, ACCESS_CONTROL_READ, (void *)&unitOfMeasure, NULL  }  },  \
	{ ZCL_CLUSTER_ID_SE_SIMPLE_METERING, {  ATTRID_METERING_SUMMATION_FORMATTING,  ZCL_DATATYPE_BITMAP8, ACCESS_CONTROL_READ, (void *)&summationFormatting, NULL  }  },  \
	{ ZCL_CLUSTER_ID_SE_SIMPLE_METERING, {  ATTRID_METERING_METERING_DEVICE_TYPE,  ZCL_DATATYPE_BITMAP8, ACCESS_CONTROL_READ, (void *)&metteringDeviceType, NULL  }  }, 
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */
extern uint48 currentSummationDelivered;
extern uint8  status;
extern uint8  unitOfMeasure;
extern uint8  summationFormatting;
extern uint8  metteringDeviceType;
/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
*********************************************************************/


#ifdef __cplusplus
}
#endif

#endif