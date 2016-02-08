#ifndef _ROS_hrpsys_ros_bridge_OpenHRP_RobotHardwareService_BatteryState_h
#define _ROS_hrpsys_ros_bridge_OpenHRP_RobotHardwareService_BatteryState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

  class OpenHRP_RobotHardwareService_BatteryState : public ros::Msg
  {
    public:
      float voltage;
      float current;
      float soc;

    OpenHRP_RobotHardwareService_BatteryState():
      voltage(0),
      current(0),
      soc(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->voltage);
      offset += serializeAvrFloat64(outbuffer + offset, this->current);
      offset += serializeAvrFloat64(outbuffer + offset, this->soc);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->voltage));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->current));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->soc));
     return offset;
    }

    const char * getType(){ return "hrpsys_ros_bridge/OpenHRP_RobotHardwareService_BatteryState"; };
    const char * getMD5(){ return "67871635fa0e3aafb978a788d456ea1f"; };

  };

}
#endif