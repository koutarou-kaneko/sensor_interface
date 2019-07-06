#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************/
/** INCLUDE FILES                                                            **/
/******************************************************************************/
   #include "LEPTON_Types.h"

/******************************************************************************/
/** EXPORTED DEFINES                                                         **/
/******************************************************************************/

   /* VID Module Command IDs
   */ 
   #define LEP_VID_MODULE_BASE                  0x0300

   #define LEP_CID_VID_POLARITY_SELECT         (LEP_VID_MODULE_BASE + 0x0000 )
   #define LEP_CID_VID_LUT_SELECT              (LEP_VID_MODULE_BASE + 0x0004 )
   #define LEP_CID_VID_LUT_TRANSFER            (LEP_VID_MODULE_BASE + 0x0008 )
   #define LEP_CID_VID_FOCUS_CALC_ENABLE       (LEP_VID_MODULE_BASE + 0x000C )
   #define LEP_CID_VID_FOCUS_ROI               (LEP_VID_MODULE_BASE + 0x0010 )
   #define LEP_CID_VID_FOCUS_THRESHOLD         (LEP_VID_MODULE_BASE + 0x0014 )
   #define LEP_CID_VID_FOCUS_METRIC            (LEP_VID_MODULE_BASE + 0x0018 )
   #define LEP_CID_VID_SBNUC_ENABLE            (LEP_VID_MODULE_BASE + 0x001C )
   #define LEP_CID_VID_GAMMA_SELECT            (LEP_VID_MODULE_BASE + 0x0020 )
   #define LEP_CID_VID_FREEZE_ENABLE           (LEP_VID_MODULE_BASE + 0x0024 )


   /* VID Module Attribute Limits
   */ 


   /* VID Module Object Sizes
   */ 
   #define LEPTON_COLOR_LUT_SIZE               256     /* 8-bit LUT as 256 x 8-bits */

/******************************************************************************/
/** EXPORTED TYPE DEFINITIONS                                                **/
/******************************************************************************/

   typedef LEP_UINT32 LEP_VID_FOCUS_METRIC_T, *LEP_VID_FOCUS_METRIC_T_PTR;
   typedef LEP_UINT32 LEP_VID_FOCUS_METRIC_THRESHOLD_T, *LEP_VID_FOCUS_METRIC_THRESHOLD_T_PTR;


   typedef enum LEP_POLARITY_E_TAG
   {
      LEP_VID_WHITE_HOT=0,
      LEP_VID_BLACK_HOT,
      LEP_VID_END_POLARITY

   }LEP_POLARITY_E, *LEP_POLARITY_E_PTR;


   /* Video Pseudo color LUT Enum
   */
   typedef enum LEP_PCOLOR_LUT_E_TAG
   {
      LEP_VID_WHEEL6_LUT=0,
      LEP_VID_FUSION_LUT,
      LEP_VID_RAINBOW_LUT,
      LEP_VID_GLOBOW_LUT,
      LEP_VID_SEPIA_LUT,
      LEP_VID_COLOR_LUT,
      LEP_VID_ICE_FIRE_LUT,
      LEP_VID_RAIN_LUT,
      LEP_VID_USER_LUT,
      LEP_VID_END_PCOLOR_LUT

   }LEP_PCOLOR_LUT_E, *LEP_PCOLOR_LUT_E_PTR;

   /* User-Defined color look-up table (LUT)
   */ 
   typedef struct LEP_VID_LUT_PIXEL_T_TAG
   {
      LEP_UINT8 reserved;
      LEP_UINT8 red;
      LEP_UINT8 green;
      LEP_UINT8 blue;
   } LEP_VID_LUT_PIXEL_T, *LEP_VID_LUT_PIXEL_T_PTR;
   typedef struct LEP_VID_LUT_BUFFER_T_TAG
   {
      LEP_VID_LUT_PIXEL_T bin[256];
   } LEP_VID_LUT_BUFFER_T, *LEP_VID_LUT_BUFFER_T_PTR;

#if 0
   typedef struct
   {
      LEP_UINT8 bin[LEPTON_COLOR_LUT_SIZE];

   }LEP_VID_LUT_BUFFER_T, *LEP_VID_LUT_BUFFER_T_PTR;
#endif

   /* Video Focus Metric Calculation Enable Enum
   */ 
   typedef enum LEP_VID_ENABLE_TAG
   {
      LEP_VID_FOCUS_CALC_DISABLE=0,
      LEP_VID_FOCUS_CALC_ENABLE,
      LEP_VID_END_FOCUS_CALC_ENABLE

   }LEP_VID_FOCUS_CALC_ENABLE_E, *LEP_VID_FOCUS_CALC_ENABLE_E_PTR;


   /* Video Focus ROI Structure
   */ 
   typedef struct LEP_VID_FOCUS_ROI_TAG
   {
      LEP_UINT16 startCol;
      LEP_UINT16 startRow;
      LEP_UINT16 endCol;
      LEP_UINT16 endRow;

   }LEP_VID_FOCUS_ROI_T, *LEP_VID_FOCUS_ROI_T_PTR;


   /* Video Scene-Based NUC Enable Enum
   */ 
   typedef enum LEP_VID_SBNUC_ENABLE_TAG
   {
      LEP_VID_SBNUC_DISABLE = 0,
      LEP_VID_SBNUC_ENABLE,
      LEP_VID_END_SBNUC_ENABLE

   }LEP_VID_SBNUC_ENABLE_E, *LEP_VID_SBNUC_ENABLE_E_PTR ;


   /* Video Freeze Output Enable Enum
   */ 
   typedef enum LEP_VID_FREEZE_ENABLE_TAG
   {
      LEP_VID_FREEZE_DISABLE = 0,
      LEP_VID_FREEZE_ENABLE,
      LEP_VID_END_FREEZE_ENABLE

   }LEP_VID_FREEZE_ENABLE_E, *LEP_VID_FREEZE_ENABLE_E_PTR ;


/******************************************************************************/
/** EXPORTED PUBLIC DATA                                                     **/
/******************************************************************************/

/******************************************************************************/
/** EXPORTED PUBLIC FUNCTIONS                                                **/
/******************************************************************************/
   extern LEP_RESULT LEP_GetVidPolarity(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                        LEP_POLARITY_E_PTR vidPolarityPtr);
   extern LEP_RESULT LEP_SetVidPolarity(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                        LEP_POLARITY_E vidPolarity);

   extern LEP_RESULT LEP_GetVidPcolorLut(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                         LEP_PCOLOR_LUT_E_PTR vidPcolorLutPtr);
   extern LEP_RESULT LEP_SetVidPcolorLut(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                         LEP_PCOLOR_LUT_E vidPcolorLut);

   extern LEP_RESULT LEP_GetVidUserLut(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                       LEP_VID_LUT_BUFFER_T_PTR vidUserLutBufPtr);

   extern LEP_RESULT LEP_SetVidUserLut(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                       LEP_VID_LUT_BUFFER_T_PTR vidUserLutBufPtr);

   extern LEP_RESULT LEP_GetVidFocusCalcEnableState(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                                    LEP_VID_FOCUS_CALC_ENABLE_E_PTR vidEnableFocusCalcStatePtr);
   extern LEP_RESULT LEP_SetVidFocusCalcEnableState(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                                    LEP_VID_FOCUS_CALC_ENABLE_E vidFocusCalcEnableState);

   extern LEP_RESULT LEP_GetVidROI(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                   LEP_VID_FOCUS_ROI_T_PTR vidFocusROIPtr);
   extern LEP_RESULT LEP_SetVidROI(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                   LEP_VID_FOCUS_ROI_T vidFocusROI);

   extern LEP_RESULT LEP_GetVidFocusMetric(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                           LEP_VID_FOCUS_METRIC_T_PTR vidFocusMetricPtr);

   extern LEP_RESULT LEP_GetVidFocusMetricThreshold(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                                    LEP_VID_FOCUS_METRIC_THRESHOLD_T_PTR vidFocusMetricThresholdPtr);
   extern LEP_RESULT LEP_SetVidFocusMetricThreshold(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                                    LEP_VID_FOCUS_METRIC_THRESHOLD_T vidFocusMetricThreshold);

   extern LEP_RESULT LEP_GetVidSbNucEnableState(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                                LEP_VID_SBNUC_ENABLE_E_PTR vidSbNucEnableStatePtr);
   extern LEP_RESULT LEP_SetVidSbNucEnableState(LEP_CAMERA_PORT_DESC_T_PTR portDescPtr,
                                                LEP_VID_SBNUC_ENABLE_E vidSbNucEnableState);

  /******************************************************************************/
#ifdef __cplusplus
}
#endif

