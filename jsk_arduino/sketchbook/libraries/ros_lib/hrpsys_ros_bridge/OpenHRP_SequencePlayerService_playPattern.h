#ifndef _ROS_SERVICE_OpenHRP_SequencePlayerService_playPattern_h
#define _ROS_SERVICE_OpenHRP_SequencePlayerService_playPattern_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Float64MultiArray.h"

namespace hrpsys_ros_bridge
{

static const char OPENHRP_SEQUENCEPLAYERSERVICE_PLAYPATTERN[] = "hrpsys_ros_bridge/OpenHRP_SequencePlayerService_playPattern";

  class OpenHRP_SequencePlayerService_playPatternRequest : public ros::Msg
  {
    public:
      std_msgs::Float64MultiArray pos;
      std_msgs::Float64MultiArray rpy;
      std_msgs::Float64MultiArray zmp;
      uint8_t tm_length;
      float st_tm;
      float * tm;

    OpenHRP_SequencePlayerService_playPatternRequest():
      pos(),
      rpy(),
      zmp(),
      tm_length(0), tm(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pos.serialize(outbuffer + offset);
      offset += this->rpy.serialize(outbuffer + offset);
      offset += this->zmp.serialize(outbuffer + offset);
      *(outbuffer + offset++) = tm_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < tm_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->tm[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pos.deserialize(inbuffer + offset);
      offset += this->rpy.deserialize(inbuffer + offset);
      offset += this->zmp.deserialize(inbuffer + offset);
      uint8_t tm_lengthT = *(inbuffer + offset++);
      if(tm_lengthT > tm_length)
        this->tm = (float*)realloc(this->tm, tm_lengthT * sizeof(float));
      offset += 3;
      tm_length = tm_lengthT;
      for( uint8_t i = 0; i < tm_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_tm));
        memcpy( &(this->tm[i]), &(this->st_tm), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return OPENHRP_SEQUENCEPLAYERSERVICE_PLAYPATTERN; };
    const char * getMD5(){ return "337f00a6e3b068d4dfdc1cc7b8dd965c"; };

  };

  class OpenHRP_SequencePlayerService_playPatternResponse : public ros::Msg
  {
    public:

    OpenHRP_SequencePlayerService_playPatternResponse()
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

    const char * getType(){ return OPENHRP_SEQUENCEPLAYERSERVICE_PLAYPATTERN; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class OpenHRP_SequencePlayerService_playPattern {
    public:
    typedef OpenHRP_SequencePlayerService_playPatternRequest Request;
    typedef OpenHRP_SequencePlayerService_playPatternResponse Response;
  };

}
#endif
