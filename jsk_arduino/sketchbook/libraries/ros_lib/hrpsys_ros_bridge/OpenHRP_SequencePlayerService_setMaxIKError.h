#ifndef _ROS_SERVICE_OpenHRP_SequencePlayerService_setMaxIKError_h
#define _ROS_SERVICE_OpenHRP_SequencePlayerService_setMaxIKError_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

static const char OPENHRP_SEQUENCEPLAYERSERVICE_SETMAXIKERROR[] = "hrpsys_ros_bridge/OpenHRP_SequencePlayerService_setMaxIKError";

  class OpenHRP_SequencePlayerService_setMaxIKErrorRequest : public ros::Msg
  {
    public:
      float pos;
      float rot;

    OpenHRP_SequencePlayerService_setMaxIKErrorRequest():
      pos(0),
      rot(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->pos);
      offset += serializeAvrFloat64(outbuffer + offset, this->rot);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pos));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->rot));
     return offset;
    }

    const char * getType(){ return OPENHRP_SEQUENCEPLAYERSERVICE_SETMAXIKERROR; };
    const char * getMD5(){ return "3c6bfe875826b548627fa31630e69ef4"; };

  };

  class OpenHRP_SequencePlayerService_setMaxIKErrorResponse : public ros::Msg
  {
    public:

    OpenHRP_SequencePlayerService_setMaxIKErrorResponse()
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

    const char * getType(){ return OPENHRP_SEQUENCEPLAYERSERVICE_SETMAXIKERROR; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class OpenHRP_SequencePlayerService_setMaxIKError {
    public:
    typedef OpenHRP_SequencePlayerService_setMaxIKErrorRequest Request;
    typedef OpenHRP_SequencePlayerService_setMaxIKErrorResponse Response;
  };

}
#endif
