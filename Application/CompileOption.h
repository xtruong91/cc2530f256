#ifndef _COMPILE_OPTION_H
#define _COMPILE_OPTION_H

// select a board for building
#define BOARD_VERSION_COORDINATOR     1
#define BOARD_VERSION_ROUTER          2
#define BOARD_VERSION_ENDDEVICE       3
#define BOARD_VERSION_NONE         4

// ----------------------------------- Board selection --------------------------------------------//
#define BOARD_VERSION           BOARD_VERSION_COORDINATOR
#if (BOARD_VERSION == BOARD_VERSION_COORDINATOR)
    #define COORDINATOR_VERSION      1
#elif (BOARD_VERSION == BOARD_VERSION_ROUTER)
    #define ROUTER_VERSION      1
#elif(BOARD_VERSION == BOARD_VERSION_ENDDEVICE)
    #define ENDDEVICE_VERSION      1
#elif (BOARD_VERSION == BOARD_VERSION_NONE)
    #define NONE_VERSION     1
#else
    #error Select complied board invalid
#endif

#define UART_EN     1 // enable UART module

#endif