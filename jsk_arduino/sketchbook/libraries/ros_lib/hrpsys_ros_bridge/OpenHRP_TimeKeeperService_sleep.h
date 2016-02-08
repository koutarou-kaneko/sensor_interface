#ifndef _ROS_SERVICE_OpenHRP_TimeKeeperService_sleep_h
#define _ROS_SERVICE_OpenHRP_TimeKeeperService_sleep_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

static const char OPENHRP_TIMEKEEPERSERVICE_SLEEP[] = "hrpsys_ros_bridge/OpenHRP_TimeKeeperService_sleep";

  class OpenHRP_TimeKeeperService_sleepRequest : public ros::Msg
  {
    public:
      float tm;

    OpenHRP_TimeKeeperService_sleepRequest():
      tm(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->tm);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tm));
     return offset;
    }

    const char * getType(){ return OPENHRP_TIMEKEEPERSERVICE_SLEEP; };
    const char * getMD5(){ return "965b531338cd158d383ca8bc28ad9e18"; };

  };

  class OpenHRP_TimeKeeperService_sleepResponse : public ros::Msg
  {
    public:

    OpenHRP_TimeKeeperService_sleepResponse()
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

    const char * getType(){ return OPENHRP_TIMEKEEPERSERVICE_SLEEP; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class OpenHRP_TimeKeeperService_sleep {
    public:
    typedef OpenHRP_TimeKeeperService_sleepRequest Request;
    typedef OpenHRP_TimeKeeperService_sleepResponse Response;
  };

}
#endif
