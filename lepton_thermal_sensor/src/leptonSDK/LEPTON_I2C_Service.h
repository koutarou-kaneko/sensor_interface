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

    extern LEP_RESULT LEP_I2C_MasterOpen(LEP_UINT16 portID, 
                                         LEP_UINT16 *portBaudRate);

    extern LEP_RESULT LEP_I2C_MasterClose(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr );

    extern LEP_RESULT LEP_I2C_MasterReset(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr );

    extern LEP_RESULT LEP_I2C_MasterReadData(LEP_UINT16 portID,
                                             LEP_UINT8  deviceAddress, 
                                             LEP_UINT16 subAddress, 
                                             LEP_UINT16 *dataPtr,
                                             LEP_UINT16 dataLength);

    extern LEP_RESULT LEP_I2C_MasterWriteData(LEP_UINT16 portID,
                                              LEP_UINT8  deviceAddress, 
                                              LEP_UINT16 subAddress, 
                                              LEP_UINT16 *dataPtr,
                                              LEP_UINT16 dataLength);

    extern LEP_RESULT LEP_I2C_MasterReadRegister(LEP_UINT16 portID,
                                                 LEP_UINT8  deviceAddress, 
                                                 LEP_UINT16 regAddress,
                                                 LEP_UINT16 *regValue);


    extern LEP_RESULT LEP_I2C_MasterWriteRegister(LEP_UINT16 portID,
                                                  LEP_UINT8  deviceAddress, 
                                                  LEP_UINT16 regAddress,
                                                  LEP_UINT16 regValue);

    extern LEP_RESULT LEP_I2C_MasterStatus(LEP_UINT16 portID,
                                           LEP_UINT16 *portStatus );

/******************************************************************************/
#ifdef __cplusplus
}
#endif
