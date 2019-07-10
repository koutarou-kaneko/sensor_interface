#pragma once

#ifdef __cplusplus
extern "C"
{
    #endif
/******************************************************************************/
/** INCLUDE FILES                                                            **/
/******************************************************************************/
    #include "LEPTON_Types.h"
    #include "LEPTON_ErrorCodes.h"

/******************************************************************************/
/** EXPORTED DEFINES                                                         **/
/******************************************************************************/

/******************************************************************************/
/** EXPORTED TYPE DEFINITIONS                                                **/
/******************************************************************************/

/******************************************************************************/
/** EXPORTED PUBLIC DATA                                                     **/
/******************************************************************************/

/******************************************************************************/
/** EXPORTED PUBLIC FUNCTIONS                                                **/
/******************************************************************************/

    extern LEP_RESULT DEV_I2C_MasterInit(char* portID,
                                         LEP_UINT16 *BaudRate);

    extern LEP_RESULT DEV_I2C_MasterClose();

    extern LEP_RESULT DEV_I2C_MasterReset(void );

    extern LEP_RESULT DEV_I2C_MasterReadData(LEP_UINT8   deviceAddress,
                                             LEP_UINT16  regAddress,            // Lepton Register Address
                                             LEP_UINT16 *readDataPtr,
                                             LEP_UINT16  wordsToRead,          // Number of 16-bit words to Read
                                             LEP_UINT16 *numWordsRead,         // Number of 16-bit words actually Read
                                             LEP_UINT16 *status
                                            );

    extern LEP_RESULT DEV_I2C_MasterWriteData(LEP_UINT8   deviceAddress,
                                              LEP_UINT16  regAddress,            // Lepton Register Address
                                              LEP_UINT16 *writeDataPtr,
                                              LEP_UINT16  wordsToWrite,        // Number of 16-bit words to Write
                                              LEP_UINT16 *numWordsWritten,     // Number of 16-bit words actually written
                                              LEP_UINT16 *status
                                             );

    extern LEP_RESULT DEV_I2C_MasterReadRegister( LEP_UINT8  deviceAddress, 
                                                  LEP_UINT16 regAddress,
                                                  LEP_UINT16 *regValue,     // Number of 16-bit words actually written
                                                  LEP_UINT16 *status
                                                );

    extern LEP_RESULT DEV_I2C_MasterWriteRegister( LEP_UINT8  deviceAddress, 
                                                   LEP_UINT16 regAddress,
                                                   LEP_UINT16 regValue,     // Number of 16-bit words actually written
                                                   LEP_UINT16 *status
                                                 );

    extern LEP_RESULT DEV_I2C_MasterStatus(void );

/******************************************************************************/
    #ifdef __cplusplus
}
    #endif

