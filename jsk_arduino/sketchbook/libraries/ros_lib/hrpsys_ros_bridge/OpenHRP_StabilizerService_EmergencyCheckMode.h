#ifndef _ROS_hrpsys_ros_bridge_OpenHRP_StabilizerService_EmergencyCheckMode_h
#define _ROS_hrpsys_ros_bridge_OpenHRP_StabilizerService_EmergencyCheckMode_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

  class OpenHRP_StabilizerService_EmergencyCheckMode : public ros::Msg
  {
    public:
      enum { NO_CHECK = 0 };
      enum { COP = 1 };
      enum { CP = 2 };

    OpenHRP_StabilizerService_EmergencyCheckMode()
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

    const char * getType(){ return "hrpsys_ros_bridge/OpenHRP_StabilizerService_EmergencyCheckMode"; };
    const char * getMD5(){ return "66e0b16f8f6a24c5380e6dc5f7efaf7a"; };

  };

}
#endif