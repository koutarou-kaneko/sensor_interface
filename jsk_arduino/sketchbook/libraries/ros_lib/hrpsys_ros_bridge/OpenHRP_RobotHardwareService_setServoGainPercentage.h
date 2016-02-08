#ifndef _ROS_SERVICE_OpenHRP_RobotHardwareService_setServoGainPercentage_h
#define _ROS_SERVICE_OpenHRP_RobotHardwareService_setServoGainPercentage_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

static const char OPENHRP_ROBOTHARDWARESERVICE_SETSERVOGAINPERCENTAGE[] = "hrpsys_ros_bridge/OpenHRP_RobotHardwareService_setServoGainPercentage";

  class OpenHRP_RobotHardwareService_setServoGainPercentageRequest : public ros::Msg
  {
    public:
      const char* name;
      float percentage;

    OpenHRP_RobotHardwareService_setServoGainPercentageRequest():
      name(""),
      percentage(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      offset += serializeAvrFloat64(outbuffer + offset, this->percentage);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->percentage));
     return offset;
    }

    const char * getType(){ return OPENHRP_ROBOTHARDWARESERVICE_SETSERVOGAINPERCENTAGE; };
    const char * getMD5(){ return "9792b77124fee8e72b69a034d8c9dd49"; };

  };

  class OpenHRP_RobotHardwareService_setServoGainPercentageResponse : public ros::Msg
  {
    public:

    OpenHRP_RobotHardwareService_setServoGainPercentageResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return OPENHRP_ROBOTHARDWARESERVICE_SETSERVOGAINPERCENTAGE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class OpenHRP_RobotHardwareService_setServoGainPercentage {
    public:
    typedef OpenHRP_RobotHardwareService_setServoGainPercentageRequest Request;
    typedef OpenHRP_RobotHardwareService_setServoGainPercentageResponse Response;
  };

}
#endif
