cmake_minimum_required(VERSION 3.3)
project(leptonSDK)

add_library(${PROJECT_NAME}
  LEPTON_AGC.c
  LEPTON_VID.c
  LEPTON_I2C_Protocol.c
  LEPTON_I2C_Service.c
  LEPTON_SDK.c
  LEPTON_SYS.c
  LEPTON_OEM.c
  i2c_handler.c
  crc16fast.c
  )
