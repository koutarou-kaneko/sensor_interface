#ifndef _ROS_SERVICE_OpenHRP_AutoBalancerService_goVelocity_h
#define _ROS_SERVICE_OpenHRP_AutoBalancerService_goVelocity_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

static const char OPENHRP_AUTOBALANCERSERVICE_GOVELOCITY[] = "hrpsys_ros_bridge/OpenHRP_AutoBalancerService_goVelocity";

  class OpenHRP_AutoBalancerService_goVelocityRequest : public ros::Msg
  {
    public:
      float vx;
      float vy;
      float vth;

    OpenHRP_AutoBalancerService_goVelocityRequest():
      vx(0),
      vy(0),
      vth(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->vx);
      offset += serializeAvrFloat64(outbuffer + offset, this->vy);
      offset += serializeAvrFloat64(outbuffer + offset, this->vth);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vx));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vy));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vth));
     return offset;
    }

    const char * getType(){ return OPENHRP_AUTOBALANCERSERVICE_GOVELOCITY; };
    const char * getMD5(){ return "14e82c5776916d6989ea54aa7b04632c"; };

  };

  class OpenHRP_AutoBalancerService_goVelocityResponse : public ros::Msg
  {
    public:
      bool operation_return;

    OpenHRP_AutoBalancerService_goVelocityResponse():
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

    const char * getType(){ return OPENHRP_AUTOBALANCERSERVICE_GOVELOCITY; };
    const char * getMD5(){ return "8dd59ee39c15084c92106411b8c3e8fc"; };

  };

  class OpenHRP_AutoBalancerService_goVelocity {
    public:
    typedef OpenHRP_AutoBalancerService_goVelocityRequest Request;
    typedef OpenHRP_AutoBalancerService_goVelocityResponse Response;
  };

}
#endif
