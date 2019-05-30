#ifndef SH_APP_H
#define SH_APP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "GenericApp.h"


 //#define NV_RESTORE
#define SHAPP_ENDPOINT           0x20 //coordinator: 0x20, 0x40, 0x80
#define SHAPP_MAX_CLUSTERS       1
#define SHAPP_CLUSTERID         1          
  
typedef struct 
{
  unsigned char endPoint;
  unsigned char extAddr[8];
  unsigned char compressed_addr;
} EndDeviceInfo_t; /* saved the end device information */


extern endPointDesc_t GenericApp_epDesc; 
extern afAddrType_t GenericApp_DstAddr;

extern void SHApp_SerialMSGCB(void);
extern void SHApp_SendTheMessage(unsigned char dest_endID, unsigned char cmd, unsigned int temp_set);
extern void str_reserve(char *str, int length);
extern void osal_buffer_uint16(unsigned char* buffer, unsigned int data);

#ifdef __cplusplus
}
#endif

#endif /*SH_APP_H*/