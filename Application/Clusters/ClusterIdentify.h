#ifndef CLUSTER_IDENTIFY_H
#define CLUSTER_IDENTIFY_H

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
#define IDENTIFY_ATTRIBUTES  \
  { ZCL_CLUSTER_ID_GEN_IDENTIFY, {  ATTRID_IDENTIFY_TIME,  ZCL_DATATYPE_UINT16, ACCESS_CONTROL_R_W, (void *)&identifyTime, &processIdentifyTimeChange  }  },
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */

extern uint16 identifyTime;

/*********************************************************************
 * FUNCTIONS
 */
void identifyInit(void);
void processIdentifyTimeChange( void );
uint16 identifyLoop(uint16 events);

ZStatus_t processIdentifyClusterServerCommands( zclIncoming_t *pInMsg );
ZStatus_t processIdentifyClusterClientCommands( zclIncoming_t *pInMsg );

/*********************************************************************
*********************************************************************/


#ifdef __cplusplus
}
#endif


#endif