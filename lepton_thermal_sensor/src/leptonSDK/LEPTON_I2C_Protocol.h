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

    #include "LEPTON_I2C_Service.h"
/******************************************************************************/
/** EXPORTED DEFINES                                                         **/
/******************************************************************************/

    /* Timeout count to wait for I2C command to complete
    */ 
    #define LEPTON_I2C_COMMAND_BUSY_WAIT_COUNT              1000

/******************************************************************************/
/** EXPORTED TYPE DEFINITIONS                                                **/
/******************************************************************************/


    typedef enum LEP_I2C_COMMAND_STATUS_TAG
    {
        LEP_I2C_COMMAND_NOT_BUSY = 0,
        LEP_I2C_COMMAND_IS_BUSY,
        LEP_I2C_END_COMMAND_STATUS

    }LEP_I2C_COMMAND_STATUS_E, *LEP_I2C_COMMAND_STATUS_E_PTR;

/******************************************************************************/
/** EXPORTED PUBLIC DATA                                                     **/
/******************************************************************************/
    
/******************************************************************************/
/** EXPORTED PUBLIC FUNCTIONS                                                **/
/******************************************************************************/

    LEP_RESULT LEP_I2C_GetCommandBusyStatus(LEP_I2C_COMMAND_STATUS_E_PTR commandStatus);

    LEP_RESULT LEP_I2C_SetCommandRegister(LEP_COMMAND_ID commandID, 
                                          LEP_UINT16 *transactionStatus);

    extern LEP_RESULT LEP_I2C_OpenPort(char* portID,
                                       LEP_UINT16 *baudRateInkHz,
                                       LEP_UINT8 *deviceAddress);

    extern LEP_RESULT LEP_I2C_ClosePort(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr);

    extern LEP_RESULT LEP_I2C_ResetPort(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr);

    extern LEP_RESULT LEP_I2C_GetAttribute(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                           LEP_COMMAND_ID commandID, 
                                           LEP_ATTRIBUTE_T_PTR attributePtr,
                                           LEP_UINT16 attributeWordLength);

    extern LEP_RESULT LEP_I2C_SetAttribute(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                           LEP_COMMAND_ID commandID, 
                                           LEP_ATTRIBUTE_T_PTR attributePtr,
                                           LEP_UINT16 attributeWordLength);

    extern LEP_RESULT LEP_I2C_RunCommand(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                         LEP_COMMAND_ID commandID);

    extern LEP_RESULT LEP_I2C_ReadData(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr);

    extern LEP_RESULT LEP_I2C_WriteData(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr);

    extern LEP_RESULT LEP_I2C_GetPortStatus(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr);

    extern LEP_RESULT LEP_I2C_GetDeviceAddress(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                               LEP_UINT8* deviceAddress);

    extern LEP_RESULT LEP_I2C_DirectWriteRegister(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                                  LEP_UINT16 regAddress,
                                                  LEP_UINT16 regValue);
    extern LEP_RESULT LEP_I2C_DirectWriteBuffer(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                                LEP_ATTRIBUTE_T_PTR attributePtr,
                                                LEP_UINT16 attributeWordLength);
    extern LEP_RESULT LEP_I2C_DirectReadRegister(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                                 LEP_UINT16 regAddress,
                                                 LEP_UINT16 *regValue);

/******************************************************************************/
#ifdef __cplusplus
}
#endif

