#ifndef COMPILE_OPTION_H
#define COMPILE_OPTION_H

// select a board for building
#define BOARD_VERSION_GENERIC           1
#define BOARD_VERSION_POWERMETER        2
#define BOARD_VERSION_SMARTDEVICE       3

// ----------------------------------- Board selection --------------------------------------------//
#define BOARD_VERSION           BOARD_VERSION_GENERIC
#if (BOARD_VERSION == BOARD_VERSION_GENERIC)
    #define GENERIC_APP     
#elif (BOARD_VERSION == BOARD_VERSION_POWERMETER)
    #define POWERMETER_APP    
#elif(BOARD_VERSION == BOARD_VERSION_SMARTDEVICE)
    #define SMARTDEVICE_APP
#else
    #error Select complied board invalid
#endif

#define UART_EN     1 // enable UART module

#endif