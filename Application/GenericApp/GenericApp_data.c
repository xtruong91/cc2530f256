/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDConfig.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"

#include  "GenericApp.h"

/*********************************************************************
 * CONSTANTS
 */

#define DEVICE_VERSION     0
#define FLAGS              0


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */



// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
    {
        GENERICAPP_CLUSTERID
    };

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
    {
        GENERICAPP_ENDPOINT,             //  int Endpoint;
        GENERICAPP_PROFID,               //  uint16 AppProfId[2];
        GENERICAPP_DEVICEID,             //  uint16 AppDeviceId[2];
        GENERICAPP_DEVICE_VERSION,       //  int   AppDevVer:4;
        GENERICAPP_FLAGS,                //  int   AppFlags:4;
        GENERICAPP_MAX_CLUSTERS,         //  byte  AppNumInClusters;
        (cId_t *)GenericApp_ClusterList, //  byte *pAppInClusterList;
        GENERICAPP_MAX_CLUSTERS,         //  byte  AppNumInClusters;
        (cId_t *)GenericApp_ClusterList  //  byte *pAppInClusterList;
};

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/****************************************************************************
****************************************************************************/