#ifndef _ROS_SERVICE_OpenHRP_AutoBalancerService_goPos_h
#define _ROS_SERVICE_OpenHRP_AutoBalancerService_goPos_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

static const char OPENHRP_AUTOBALANCERSERVICE_GOPOS[] = "hrpsys_ros_bridge/OpenHRP_AutoBalancerService_goPos";

  class OpenHRP_AutoBalancerService_goPosRequest : public ros::Msg
  {
    public:
      float x;
      float y;
      float th;

    OpenHRP_AutoBalancerService_goPosRequest():
      x(0),
      y(0),
      th(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->th);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->th));
     return offset;
    }

    const char * getType(){ return OPENHRP_AUTOBALANCERSERVICE_GOPOS; };
    const char * getMD5(){ return "000e435776f4fd6ba555d25d7a61ed8f"; };

  };

  class OpenHRP_AutoBalancerService_goPosResponse : public ros::Msg
  {
    public:
      bool operation_return;

    OpenHRP_AutoBalancerService_goPosResponse():
      operation_return(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_operation_return;
      u_operation_return.real = this->operation_return;
      *(outbuffer + offset + 0) = (u_operation_return.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->operation_return);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_operation_return;
      u_operation_return.base = 0;
      u_operation_return.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->operation_return = u_operation_return.real;
      offset += sizeof(this->operation_return);
     return offset;
    }

    const char * getType(){ return OPENHRP_AUTOBALANCERSERVICE_GOPOS; };
    const char * getMD5(){ return "8dd59ee39c15084c92106411b8c3e8fc"; };

  };

  class OpenHRP_AutoBalancerService_goPos {
    public:
    typedef OpenHRP_AutoBalancerService_goPosRequest Request;
    typedef OpenHRP_AutoBalancerService_goPosResponse Response;
  };

}
#endif
