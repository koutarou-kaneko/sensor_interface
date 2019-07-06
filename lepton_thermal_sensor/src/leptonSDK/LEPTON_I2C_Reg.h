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

/* DEVICE ADDRESSES
*/
/* The Lepton camera's device address
*/
    #define LEP_I2C_DEVICE_ADDRESS              0x2A          

/* Block Data Buffers
*/ 
    #define LEP_DATA_BUFFER_0_BASE_ADDR         0xF800
    #define LEP_DATA_BUFFER_1_BASE_ADDR         0xFC00


/* The Lepton I2C Registers Sub-Addresses
*/
    #define LEP_I2C_REG_BASE_ADDR               0x0000

    /* Host On Switch when camera is in stand by of off
    */ 
    #define LEP_I2C_POWER_REG                  (LEP_I2C_REG_BASE_ADDR + 0x0000 )

    /* Host Command Interface over I2C
    */ 
    #define LEP_I2C_STATUS_REG                 (LEP_I2C_REG_BASE_ADDR + 0x0002 )
    #define LEP_I2C_COMMAND_REG                (LEP_I2C_REG_BASE_ADDR + 0x0004 )
    #define LEP_I2C_DATA_LENGTH_REG            (LEP_I2C_REG_BASE_ADDR + 0x0006 )
    #define LEP_I2C_DATA_0_REG                 (LEP_I2C_REG_BASE_ADDR + 0x0008 )
    #define LEP_I2C_DATA_1_REG                 (LEP_I2C_REG_BASE_ADDR + 0x000A )
    #define LEP_I2C_DATA_2_REG                 (LEP_I2C_REG_BASE_ADDR + 0x000C )
    #define LEP_I2C_DATA_3_REG                 (LEP_I2C_REG_BASE_ADDR + 0x000E )
    #define LEP_I2C_DATA_4_REG                 (LEP_I2C_REG_BASE_ADDR + 0x0010 )
    #define LEP_I2C_DATA_5_REG                 (LEP_I2C_REG_BASE_ADDR + 0x0012 )
    #define LEP_I2C_DATA_6_REG                 (LEP_I2C_REG_BASE_ADDR + 0x0014 )
    #define LEP_I2C_DATA_7_REG                 (LEP_I2C_REG_BASE_ADDR + 0x0016 )
    #define LEP_I2C_DATA_8_REG                 (LEP_I2C_REG_BASE_ADDR + 0x0018 )
    #define LEP_I2C_DATA_9_REG                 (LEP_I2C_REG_BASE_ADDR + 0x001A )
    #define LEP_I2C_DATA_10_REG                (LEP_I2C_REG_BASE_ADDR + 0x001C )
    #define LEP_I2C_DATA_11_REG                (LEP_I2C_REG_BASE_ADDR + 0x001E )
    #define LEP_I2C_DATA_12_REG                (LEP_I2C_REG_BASE_ADDR + 0x0020 )
    #define LEP_I2C_DATA_13_REG                (LEP_I2C_REG_BASE_ADDR + 0x0022 )
    #define LEP_I2C_DATA_14_REG                (LEP_I2C_REG_BASE_ADDR + 0x0024 )
    #define LEP_I2C_DATA_15_REG                (LEP_I2C_REG_BASE_ADDR + 0x0026 )

    #define LEP_I2C_DATA_CRC_REG               (LEP_I2C_REG_BASE_ADDR + 0x0028 )

    #define LEP_I2C_DATA_BUFFER_0              (LEP_DATA_BUFFER_0_BASE_ADDR )
    #define LEP_I2C_DATA_BUFFER_0_END          (LEP_DATA_BUFFER_0_BASE_ADDR + 0x03FF )
    #define LEP_I2C_DATA_BUFFER_0_LENGTH        0x400

    #define LEP_I2C_DATA_BUFFER_1              (LEP_DATA_BUFFER_1_BASE_ADDR )
    #define LEP_I2C_DATA_BUFFER_1_END          (LEP_DATA_BUFFER_1_BASE_ADDR + 0x03FF )
    #define LEP_I2C_DATA_BUFFER_1_LENGTH        0x400

    #define LEP_I2C_STATUS_BUSY_BIT_MASK        0x0001   /* Bit 0 is the Busy Bit */


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
