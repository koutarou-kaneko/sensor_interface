#pragma once

#include <stdint.h>
#include <iostream>

#include "LEPTON_SDK.h"
#include "LEPTON_SYS.h"
#include "LEPTON_OEM.h"
#include "LEPTON_Types.h"

namespace Lepton
{
  namespace I2C
  {
    bool openPort(std::string i2c_dev);
    bool closePort(void);
    void performFfc();
    void reboot();
    int  fpaTemperature();
  };
};

