#ifndef PC_App_H
#define PC_APP_H

#include "AF.h"
#include "aps_groups.h"
#define SAMPLEAPP_ENDPOINT 20
// Group ID for Flash Command
#define SAMPLEAPP_FLASH_GROUP                  0x0001
  
// Flash Command Duration - in milliseconds
#define SAMPLEAPP_FLASH_DURATION 1000


extern aps_Group_t SampleApp_Group;

extern void SampleApp_SendPeriodicMessage( void );
extern void SampleApp_SendFlashMessage( uint16 flashTime );

#endif