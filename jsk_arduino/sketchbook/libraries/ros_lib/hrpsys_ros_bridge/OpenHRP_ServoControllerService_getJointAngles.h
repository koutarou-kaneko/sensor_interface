#ifndef _ROS_SERVICE_OpenHRP_ServoControllerService_getJointAngles_h
#define _ROS_SERVICE_OpenHRP_ServoControllerService_getJointAngles_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

static const char OPENHRP_SERVOCONTROLLERSERVICE_GETJOINTANGLES[] = "hrpsys_ros_bridge/OpenHRP_ServoControllerService_getJointAngles";

  class OpenHRP_ServoControllerService_getJointAnglesRequest : public ros::Msg
  {
    public:

    OpenHRP_ServoControllerService_getJointAnglesRequest()
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

    const char * getType(){ return OPENHRP_SERVOCONTROLLERSERVICE_GETJOINTANGLES; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class OpenHRP_ServoControllerService_getJointAnglesResponse : public ros::Msg
  {
    public:
      bool operation_return;
      uint8_t jvs_length;
      float st_jvs;
      float * jvs;

    OpenHRP_ServoControllerService_getJointAnglesResponse():
      operation_return(0),
      jvs_length(0), jvs(NULL)
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
      *(outbuffer + offset++) = jvs_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < jvs_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->jvs[i]);
      }
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
      uint8_t jvs_lengthT = *(inbuffer + offset++);
      if(jvs_lengthT > jvs_length)
        this->jvs = (float*)realloc(this->jvs, jvs_lengthT * sizeof(float));
      offset += 3;
      jvs_length = jvs_lengthT;
      for( uint8_t i = 0; i < jvs_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_jvs));
        memcpy( &(this->jvs[i]), &(this->st_jvs), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return OPENHRP_SERVOCONTROLLERSERVICE_GETJOINTANGLES; };
    const char * getMD5(){ return "4fdfe079ef20ee48e9fab4471d91dd87"; };

  };

  class OpenHRP_ServoControllerService_getJointAngles {
    public:
    typedef OpenHRP_ServoControllerService_getJointAnglesRequest Request;
    typedef OpenHRP_ServoControllerService_getJointAnglesResponse Response;
  };

}
#endif
