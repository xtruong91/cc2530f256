#ifndef CLUSTER_BASIC_H
#define CLUSTER_BASIC_H

#ifdef __cplusplus
extern "C"
{
#endif


/*********************************************************************
 * INCLUDES
 */
#include "zcl_general.h"
#include "zcl.h"

/*********************************************************************
 * CONSTANTS
 */
extern const uint8 HWRevision;
extern const uint8 ZCLVersion;
extern const uint8 manufacturerName[];
extern const uint8 modelId[];
extern const uint8 dateCode[];
extern const uint8 powerSource;
extern uint8 locationDescription[];
extern uint8 physicalEnvironment;
extern uint8 deviceEnable;

/*********************************************************************
 * MACROS
 */
#define BASIC_ATTRIBUTE \
	{ ZCL_CLUSTER_ID_GEN_BASIC, { ATTRID_BASIC_HW_VERSION       , ZCL_DATATYPE_UINT8   , ACCESS_CONTROL_READ, (void *)&HWRevision     , NULL }  }, \
	{ ZCL_CLUSTER_ID_GEN_BASIC, { ATTRID_BASIC_ZCL_VERSION      , ZCL_DATATYPE_UINT8   , ACCESS_CONTROL_READ, (void *)&ZCLVersion     , NULL }  }, \
	{ ZCL_CLUSTER_ID_GEN_BASIC, { ATTRID_BASIC_MANUFACTURER_NAME, ZCL_DATATYPE_CHAR_STR, ACCESS_CONTROL_READ, (void *)manufacturerName, NULL }  }, \
	{ ZCL_CLUSTER_ID_GEN_BASIC, { ATTRID_BASIC_MODEL_ID         , ZCL_DATATYPE_CHAR_STR, ACCESS_CONTROL_READ, (void *)modelId         , NULL }  }, \
	{ ZCL_CLUSTER_ID_GEN_BASIC, { ATTRID_BASIC_DATE_CODE        , ZCL_DATATYPE_CHAR_STR, ACCESS_CONTROL_READ, (void *)dateCode        , NULL     }  }, \
	{ ZCL_CLUSTER_ID_GEN_BASIC, { ATTRID_BASIC_POWER_SOURCE     , ZCL_DATATYPE_ENUM8,    ACCESS_CONTROL_READ, (void *)&powerSource    , NULL     }  }, \
	{ ZCL_CLUSTER_ID_GEN_BASIC, { ATTRID_BASIC_LOCATION_DESC    , ZCL_DATATYPE_CHAR_STR, ACCESS_CONTROL_R_W , (void *)locationDescription, NULL  } }, \
        { ZCL_CLUSTER_ID_GEN_BASIC, { ATTRID_BASIC_PHYSICAL_ENV     , ZCL_DATATYPE_ENUM8   , ACCESS_CONTROL_R_W , (void *)&physicalEnvironment , NULL} }, \
	{ ZCL_CLUSTER_ID_GEN_BASIC, { ATTRID_BASIC_DEVICE_ENABLED   , ZCL_DATATYPE_BOOLEAN , ACCESS_CONTROL_R_W , (void *)&deviceEnable   , NULL     } },
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * FUNCTIONS
 */
void basicResetCB( void );
ZStatus_t processBasicClusterCommands( zclIncoming_t *pInMsg );


/*********************************************************************
*********************************************************************/


#ifdef __cplusplus
}
#endif

#endif