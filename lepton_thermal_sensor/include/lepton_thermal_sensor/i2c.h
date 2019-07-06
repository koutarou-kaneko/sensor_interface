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
    bool openPort(uint8_t id = 1);
    bool closePort(void);
    void performFfc();
    void reboot();
    int  fpaTemperature();
  };
};

