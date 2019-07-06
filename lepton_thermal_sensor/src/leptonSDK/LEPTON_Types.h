#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

    //  #define AVR_IAR_BUILD
/******************************************************************************/
/** INCLUDE FILES                                                            **/
/******************************************************************************/

    #ifdef AVR_IAR_BUILD
        #include "avr_compiler.h"
    #endif      

    #if defined(_WIN32) || defined (_WIN64) 
       #include <windows.h> 
       #define WINDOWSS 1;
    #else
       #include <stdlib.h>
       #include <stdbool.h>
    #endif
    
/******************************************************************************/
/** EXPORTED DEFINES                                                         **/
/******************************************************************************/
    #define	LEP_FAILURE		-1
    #define	LEP_SUCCESS		0
    #define	LEP_TRUE		1
    #define	LEP_FALSE		0
    #define LEP_NULL		0

/******************************************************************************/
/** EXPORTED TYPE DEFINITIONS                                                **/
/******************************************************************************/

    #ifdef _STDINT_H
    typedef uint8_t             LEP_UINT8;
    typedef uint8_t             LEP_UCHAR;
    typedef int8_t              LEP_INT8;
    typedef char                LEP_CHAR8;

    typedef uint16_t            LEP_UINT16;
    typedef uint16_t            LEP_USHORT;
    typedef int16_t             LEP_INT16;
    typedef short               LEP_SHORT;

    typedef uint32_t            LEP_UINT32;
    typedef uint32_t            LEP_UINT;
    typedef int32_t             LEP_INT32;
    typedef int                 LEP_INT;

    typedef uint64_t            LEP_UINT64;
    typedef uint64_t            LEP_ULONG64;
    typedef uint32_t            LEP_ULONG32;
    typedef uint32_t            LEP_ULONG;
    typedef int64_t             LEP_LONG32;
    typedef long                LEP_LONG;

    typedef float               LEP_FLOAT32;
    typedef double              LEP_FLOAT64;
    #else    
    typedef unsigned char       LEP_UINT8;
    typedef unsigned char       LEP_UCHAR;
    typedef signed char         LEP_INT8;
    typedef char                LEP_CHAR8;

    typedef unsigned short      LEP_UINT16;
    typedef unsigned short      LEP_USHORT;
    typedef signed short        LEP_INT16;
    typedef short               LEP_SHORT;

    typedef unsigned int        LEP_UINT32;
    typedef unsigned int        LEP_UINT;
    typedef signed int          LEP_INT32;
    typedef int                 LEP_INT;

    typedef unsigned long long  LEP_UINT64;
    typedef unsigned long long  LEP_ULONG64;
    typedef unsigned long       LEP_ULONG32;
    typedef unsigned long       LEP_ULONG;
    typedef signed long         LEP_LONG32;
    typedef long                LEP_LONG;

    typedef float               LEP_FLOAT32;
    typedef double              LEP_FLOAT64;
    #endif        


    #ifdef _STDBOOL_H
    typedef bool                LEP_BOOL, *LEP_BOOL_PTR;
    #else    
    typedef unsigned char       LEP_BOOL, *LEP_BOOL_PTR;
    #endif        

    /* NULL
    */ 
    #ifndef NULL
        #define NULL '\0'
    #endif


    typedef LEP_UINT16          LEP_COMMAND_ID;
    typedef LEP_UINT16          LEP_ATTRIBUTE_T,*LEP_ATTRIBUTE_T_PTR;

    #define LEP_GET_TYPE        0x0000
    #define LEP_SET_TYPE        0x0001
    #define LEP_RUN_TYPE        0x0002

    typedef enum
    {
        LEP_LSB_FIRST=0,
        LEP_MSB_FIRST

    }LEP_BYTE_ORDER_T, *LEP_BYTE_ORDER_T_PTR;

    typedef enum
    {
        LEP_READY = 0,
        LEP_BUSY,
        LEP_WAITING

    }LEP_OPERATION_STATE;

    typedef enum
    {
        LEP_DISABLED = 0,
        LEP_ENABLED

    }LEP_ENABLE_STATE;

    typedef enum
    {
        LEP_OFF = 0,
        LEP_ON

    }LEP_ON_STATE;


    /* Lepton physical tranport interfaces
    */ 
    typedef enum LEP_CAMERA_PORT_E_TAG
    {
        LEP_CCI_TWI=0,
        LEP_CCI_SPI,
        LEP_END_CCI_PORTS
    }LEP_CAMERA_PORT_E, *LEP_CAMERA_PORT_E_PTR;

    /* Lepton supported TWI  clock rates
    */ 
    typedef enum LEP_TWI_CLOCK_RATE_T_TAG
    {
        LEP_TWI_CLOCK_100KHZ=0,
        LEP_TWI_CLOCK_400KHZ,
        LEP_TWI_CLOCK_1MHZ,
        LEP_END_TWI_CLOCK_RATE

    }LEP_TWI_CLOCK_RATE_T, *LEP_TWI_CLOCK_RATE_T_PTR;

    /* Lepton supported SPI  clock rates
    */ 
    typedef enum LEP_SPI_CLOCK_RATE_T_TAG
    {
        LEP_SPI_CLOCK_2MHZ=0,
        LEP_SPI_CLOCK_10MHZ,
        LEP_SPI_CLOCK_20MHZ,
        LEP_END_SPI_CLOCK_RATE

    }LEP_SPI_CLOCK_RATE_T, *LEP_SPI_CLOCK_RATE_T_PTR;

    /* Communications Port Descriptor Type
    */ 
    typedef struct  LEP_CAMERA_PORT_DESC_T_TAG
    {
        LEP_UINT16  portID;
        LEP_CAMERA_PORT_E   portType;
        LEP_UINT16  portBaudRate;
        LEP_UINT8 deviceAddress;
    }LEP_CAMERA_PORT_DESC_T, *LEP_CAMERA_PORT_DESC_T_PTR;


/******************************************************************************/
/** EXPORTED PUBLIC DATA                                                     **/
/******************************************************************************/

/******************************************************************************/
/** EXPORTED PUBLIC FUNCTIONS                                                **/
/******************************************************************************/


/******************************************************************************/
#ifdef __cplusplus
}
#endif


