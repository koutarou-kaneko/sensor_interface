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
    int connect() {
      LEP_OpenPort(1, LEP_CCI_TWI, 400, &port);
      connected = true;
      return 0;
    }

    void performFfc() {
      if(!connected) connect();

      LEP_RunSysFFCNormalization(&port);
    }

    //LEP_SYS_FPA_TEMPERATURE_KELVIN_T lepton_fpa_temperature() {
    int fpaTemperature() {
      if(!connected)  connect();

      result = LEP_GetSysFpaTemperatureKelvin(&port, &fpa_temp_kelvin);
      return int(fpa_temp_kelvin);
    }

    //presumably more commands could go here if desired
    void reboot() {
      if(!connected) {
        connect();
      }
      LEP_RunOemReboot(&port);
    }
  };
};
