#ifndef CLUSTER_OSAL_EVENT_H
#define CLUSTER_OSAL_EVENT_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */
#define CLUSTER_EVENT
#define IDENTIFY_TIMEOUT_EVT     0x0001
#define LEVEL_MOVEMENT           0x0002
#define READ_TEMP_EVT	         0x0004
#define END_READ_TEMP_EVT        0x0008
#define READ_TEMP_MASK	         0x000C

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * FUNCTIONS
 */
/*********************************************************************
*********************************************************************/


#ifdef __cplusplus
}
#endif

#endif