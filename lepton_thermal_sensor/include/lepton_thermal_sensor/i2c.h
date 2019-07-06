#pragma once

#include "LEPTON_SDK.h"
#include "LEPTON_SYS.h"
#include "LEPTON_OEM.h"
#include "LEPTON_Types.h"

namespace Lepton
{
  namespace I2C
  {
    int connect();
    void performFfc();
    void reboot();
    int  fpaTemperature();
  };
};

