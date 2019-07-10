#include "lepton_thermal_sensor/i2c.h"

namespace
{
  LEP_CAMERA_PORT_DESC_T port;
  LEP_SYS_FPA_TEMPERATURE_KELVIN_T fpa_temp_kelvin;
  LEP_SYS_AUX_TEMPERATURE_KELVIN_T aux_temp_kelvin;
  LEP_RESULT result;

  bool connected;
};

namespace Lepton
{
  namespace I2C
  {
    bool openPort(std::string i2c_dev) {
      if(LEP_OpenPort(const_cast<char*>(i2c_dev.c_str()), LEP_CCI_TWI, 400, &port) != LEP_OK)
        return false;

      connected = true;
      return true;
    }

    bool closePort()
    {
      if(LEP_ClosePort(&port) != LEP_OK)
        return false;
      return true;
    }

    void performFfc() {
      if(!connected) return;

      LEP_RunSysFFCNormalization(&port);
    }

    //LEP_SYS_FPA_TEMPERATURE_KELVIN_T lepton_fpa_temperature() {
    int fpaTemperature() {
      if(!connected)
        {
          std::cout << "I2C is not connected" << std::endl;
          return 0;
        }

      result = LEP_GetSysFpaTemperatureKelvin(&port, &fpa_temp_kelvin);
      return int(fpa_temp_kelvin);
    }

    void reboot()
    {
      if(!connected)
        {
          std::cout << "I2C is not connected" << std::endl;
          return;
        }

      LEP_RunOemReboot(&port);
    }
  };
};
