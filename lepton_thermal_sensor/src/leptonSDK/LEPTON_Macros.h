#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************/
/** INCLUDE FILES                                                            **/
/******************************************************************************/

/******************************************************************************/
/** EXPORTED DEFINES                                                         **/
/******************************************************************************/

/****************************************************************** 
***    USEFUL MACROS                                            ***
******************************************************************/

    #ifndef MIN
        #define MIN(a, b) ((a) < (b)? (a): (b))
    #endif
    #ifndef MAX
        #define MAX(a, b) ((a) > (b)? (a): (b))
    #endif

    #ifndef LOW_WORD
        #define LOW_WORD(longVariable) ((LEP_UINT16)longVariable)
    #endif
    #ifndef HIGH_WORD
        #define HIGH_WORD(longVariable) ((LEP_UINT16)(longVariable>>16))
    #endif
    #ifndef LOW_BYTE
        #define LOW_BYTE(w)           ((LEP_UINT8)(w))
    #endif
    #ifndef HIGH_BYTE
        #define HIGH_BYTE(w)           ((LEP_UINT8)(((w) >> 8) & 0xFF))
    #endif

    #ifndef LOW_NIBBLE
        #define LOW_NIBBLE(w)           ((LEP_UINT8)(w) & 0x0F)
    #endif
    #ifndef HIGH_NIBBLE
        #define HIGH_NIBBLE(w)           ((LEP_UINT8)(((w) >> 4) & 0x0F))
    #endif

    #define CLR_BIT(_port,_bit)         ((_port) & ~(_bit))


    #define REVERSE_ENDIENESS_UINT16(uint16Var) \
           ( ( ((LEP_UINT16)LOW_BYTE(uint16Var))<<8) + (LEP_UINT16)HIGH_BYTE(uint16Var))

    #define REVERSE_ENDIENESS_UINT32(uint32Var) \
           ( ((LEP_UINT32)REVERSE_ENDIENESS_UINT16(LOW_WORD(uint32Var)) << 16) + \
             (LEP_UINT32)REVERSE_ENDIENESS_UINT16(HIGH_WORD(uint32Var) ) )

    #define REVERSE_NIBBLE_UINT8(uint8Var) \
           ( ( ((LEP_UINT8)LOW_NIBBLE(uint8Var))<<4) + (LEP_UINT8)HIGH_NIBBLE(uint8Var))

    #define REVERSE_BYTEORDER_UINT32(uint32Var) \
           ( (((LEP_UINT32)LOW_BYTE(uint32Var))<<24) + (((LEP_UINT32)HIGH_BYTE(uint32Var))<<16) + \
             (((LEP_UINT32)LOW_BYTE(HIGH_WORD(uint32Var)))<<8) + (LEP_UINT32)HIGH_BYTE(HIGH_WORD(uint32Var)) )

    #define WORD_SWAP_16(uint32Var)  \
            ( ((LEP_UINT16)LOW_WORD(uint32Var) << 16) + ((LEP_UINT16)HIGH_WORD(uint32Var)) )


    #ifndef NUM_ELEMENTS_IN_ARRAY
        #define NUM_ELEMENTS_IN_ARRAY(array) (sizeof (array) / sizeof ((array) [0]))
    #endif /* NUM_ELEMENTS_IN_ARRAY */

    #ifndef NELEMENTS  
        #define NELEMENTS(array)      /* number of elements in an array */ \
              (sizeof (array) / sizeof ((array) [0]))  
    #endif /* NELEMENTS */


/******************************************************************************/
/** EXPORTED TYPE DEFINITIONS                                                **/
/******************************************************************************/

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
