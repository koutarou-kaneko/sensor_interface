#ifndef _ROS_SERVICE_OpenHRP_SequencePlayerService_loadPattern_h
#define _ROS_SERVICE_OpenHRP_SequencePlayerService_loadPattern_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

static const char OPENHRP_SEQUENCEPLAYERSERVICE_LOADPATTERN[] = "hrpsys_ros_bridge/OpenHRP_SequencePlayerService_loadPattern";

  class OpenHRP_SequencePlayerService_loadPatternRequest : public ros::Msg
  {
    public:
      const char* basename;
      float tm;

    OpenHRP_SequencePlayerService_loadPatternRequest():
      basename(""),
      tm(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_basename = strlen(this->basename);
      memcpy(outbuffer + offset, &length_basename, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->basename, length_basename);
      offset += length_basename;
      offset += serializeAvrFloat64(outbuffer + offset, this->tm);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_basename;
      memcpy(&length_basename, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_basename; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_basename-1]=0;
      this->basename = (char *)(inbuffer + offset-1);
      offset += length_basename;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tm));
     return offset;
    }

    const char * getType(){ return OPENHRP_SEQUENCEPLAYERSERVICE_LOADPATTERN; };
    const char * getMD5(){ return "5b6071c1a3707612b9adf6c3e53d310c"; };

  };

  class OpenHRP_SequencePlayerService_loadPatternResponse : public ros::Msg
  {
    public:

    OpenHRP_SequencePlayerService_loadPatternResponse()
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

    const char * getType(){ return OPENHRP_SEQUENCEPLAYERSERVICE_LOADPATTERN; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class OpenHRP_SequencePlayerService_loadPattern {
    public:
    typedef OpenHRP_SequencePlayerService_loadPatternRequest Request;
    typedef OpenHRP_SequencePlayerService_loadPatternResponse Response;
  };

}
#endif
