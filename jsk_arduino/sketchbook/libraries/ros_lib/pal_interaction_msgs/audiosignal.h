#ifndef _ROS_pal_interaction_msgs_audiosignal_h
#define _ROS_pal_interaction_msgs_audiosignal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pal_interaction_msgs
{

  class audiosignal : public ros::Msg
  {
    public:
      uint8_t channel0_length;
      float st_channel0;
      float * channel0;
      uint8_t channel1_length;
      float st_channel1;
      float * channel1;

    audiosignal():
      channel0_length(0), channel0(NULL),
      channel1_length(0), channel1(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = channel0_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < channel0_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->channel0[i]);
      }
      *(outbuffer + offset++) = channel1_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < channel1_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->channel1[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t channel0_lengthT = *(inbuffer + offset++);
      if(channel0_lengthT > channel0_length)
        this->channel0 = (float*)realloc(this->channel0, channel0_lengthT * sizeof(float));
      offset += 3;
      channel0_length = channel0_lengthT;
      for( uint8_t i = 0; i < channel0_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_channel0));
        memcpy( &(this->channel0[i]), &(this->st_channel0), sizeof(float));
      }
      uint8_t channel1_lengthT = *(inbuffer + offset++);
      if(channel1_lengthT > channel1_length)
        this->channel1 = (float*)realloc(this->channel1, channel1_lengthT * sizeof(float));
      offset += 3;
      channel1_length = channel1_lengthT;
      for( uint8_t i = 0; i < channel1_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_channel1));
        memcpy( &(this->channel1[i]), &(this->st_channel1), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "pal_interaction_msgs/audiosignal"; };
    const char * getMD5(){ return "9406f94b261ed2ebad808e7af5b61907"; };

  };

}
#endif