#ifndef _ROS_SERVICE_OpenHRP_SequencePlayerService_playPatternOfGroup_h
#define _ROS_SERVICE_OpenHRP_SequencePlayerService_playPatternOfGroup_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Float64MultiArray.h"

namespace hrpsys_ros_bridge
{

static const char OPENHRP_SEQUENCEPLAYERSERVICE_PLAYPATTERNOFGROUP[] = "hrpsys_ros_bridge/OpenHRP_SequencePlayerService_playPatternOfGroup";

  class OpenHRP_SequencePlayerService_playPatternOfGroupRequest : public ros::Msg
  {
    public:
      const char* gname;
      std_msgs::Float64MultiArray pos;
      uint8_t tm_length;
      float st_tm;
      float * tm;

    OpenHRP_SequencePlayerService_playPatternOfGroupRequest():
      gname(""),
      pos(),
      tm_length(0), tm(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_gname = strlen(this->gname);
      memcpy(outbuffer + offset, &length_gname, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->gname, length_gname);
      offset += length_gname;
      offset += this->pos.serialize(outbuffer + offset);
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
      uint32_t length_gname;
      memcpy(&length_gname, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_gname; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_gname-1]=0;
      this->gname = (char *)(inbuffer + offset-1);
      offset += length_gname;
      offset += this->pos.deserialize(inbuffer + offset);
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

    const char * getType(){ return OPENHRP_SEQUENCEPLAYERSERVICE_PLAYPATTERNOFGROUP; };
    const char * getMD5(){ return "4dacd7c1e4da1aeb5f3269e58af6ff17"; };

  };

  class OpenHRP_SequencePlayerService_playPatternOfGroupResponse : public ros::Msg
  {
    public:
      bool operation_return;

    OpenHRP_SequencePlayerService_playPatternOfGroupResponse():
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

    const char * getType(){ return OPENHRP_SEQUENCEPLAYERSERVICE_PLAYPATTERNOFGROUP; };
    const char * getMD5(){ return "8dd59ee39c15084c92106411b8c3e8fc"; };

  };

  class OpenHRP_SequencePlayerService_playPatternOfGroup {
    public:
    typedef OpenHRP_SequencePlayerService_playPatternOfGroupRequest Request;
    typedef OpenHRP_SequencePlayerService_playPatternOfGroupResponse Response;
  };

}
#endif
