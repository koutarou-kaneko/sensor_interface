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
    #include "LEPTON_SDKConfig.h"
    #include "LEPTON_I2C_Protocol.h"
	
/******************************************************************************/
    /**
     * If defined then the CCI and SPI port drivers require opening
     * and closing with each access; otherwise the ports are opened 
     * once and left open for future accesses 
     */
//    #define LEP_USE_DYNAMIC_ALLOCATION

     #define LEP_JOVA_I2C
     #define LEP_SDK_VERSION_MAJOR         0
     #define LEP_SDK_VERSION_MINOR         3
     #define LEP_SDK_VERSION_BUILD         53

    /* SDK Module Command IDs
    */ 
    #define LEP_SDK_MODULE_BASE            0x0000

    #define LEP_SDK_ENABLE_STATE           (LEP_SDK_MODULE_BASE + 0x0000 )


/******************************************************************************/
/** EXPORTED TYPEDEFS                                                        **/
/******************************************************************************/
   typedef struct LEP_SDK_VERSION_TAG
   {
      LEP_UINT8   major;
      LEP_UINT8   minor;
      LEP_UINT8   build;
      LEP_UINT8   reserved;

   }LEP_SDK_VERSION_T, *LEP_SDK_VERSION_T_PTR;

   typedef enum LEP_SDK_BOOT_STATUS_E_TAG
   {
      LEP_BOOT_STATUS_NOT_BOOTED = 0,
      LEP_BOOT_STATUS_BOOTED = 1,

      LEP_END_BOOT_STATUS,
   }LEP_SDK_BOOT_STATUS_E, *LEP_SDK_BOOT_STATUS_E_PTR;
/******************************************************************************/
/** EXPORTED DEFINES                                                         **/ 
/******************************************************************************/

/******************************************************************************/
/** EXPORTED PUBLIC DATA                                                     **/
/******************************************************************************/
   extern LEP_SDK_VERSION_T sdkVersion;
/******************************************************************************/
/** EXPORTED PUBLIC FUNCTIONS                                                **/
/******************************************************************************/

    extern LEP_RESULT LEP_OpenPort(char* portID,
                                   LEP_CAMERA_PORT_E portType,
                                   LEP_UINT16   portBaudRate,
                                   LEP_CAMERA_PORT_DESC_T_PTR portDescPtr);

    extern LEP_RESULT LEP_ClosePort(LEP_CAMERA_PORT_DESC_T_PTR pd);

    extern LEP_RESULT LEP_ResetPort(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr);

    extern LEP_RESULT LEP_GetPortStatus(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr, 
                                        LEP_UINT16 *status);

    extern LEP_RESULT LEP_GetDeviceAddress(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                           LEP_UINT8* deviceAddress);

    extern LEP_RESULT LEP_GetAttribute(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                       LEP_COMMAND_ID commandID, 
                                       LEP_ATTRIBUTE_T_PTR attributePtr,
                                       LEP_UINT16 attributeWordLength);

    extern LEP_RESULT LEP_SetAttribute(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                       LEP_COMMAND_ID commandID, 
                                       LEP_ATTRIBUTE_T_PTR attributePtr,
                                       LEP_UINT16 attributeWordLength);

    extern LEP_RESULT LEP_RunCommand(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                     LEP_COMMAND_ID commandID);

    extern LEP_RESULT LEP_DirectWriteBuffer(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                            LEP_ATTRIBUTE_T_PTR attributePtr,
                                            LEP_UINT16 attributeWordLength);

    extern LEP_RESULT LEP_DirectWriteRegister(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                              LEP_UINT16 registerAddress,
                                              LEP_UINT16 regValue);

    extern LEP_RESULT LEP_DirectReadRegister(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                             LEP_UINT16 registerAddress,
                                             LEP_UINT16* regValue);

    extern LEP_RESULT LEP_GetSDKVersion(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                        LEP_SDK_VERSION_T_PTR sdkVersionPtr);

    extern LEP_RESULT LEP_GetCameraBootStatus(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                              LEP_SDK_BOOT_STATUS_E_PTR bootStatusPtr);

/******************************************************************************/
	
#ifdef __cplusplus
}
#endif
