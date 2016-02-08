#ifndef _ROS_leap_motion_leap_h
#define _ROS_leap_motion_leap_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace leap_motion
{

  class leap : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float hand_direction[3];
      float hand_normal[3];
      float hand_palm_pos[3];
      float hand_pitch;
      float hand_roll;
      float hand_yaw;

    leap():
      header(),
      hand_direction(),
      hand_normal(),
      hand_palm_pos(),
      hand_pitch(0),
      hand_roll(0),
      hand_yaw(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint8_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->hand_direction[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->hand_normal[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->hand_palm_pos[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->hand_pitch);
      offset += serializeAvrFloat64(outbuffer + offset, this->hand_roll);
      offset += serializeAvrFloat64(outbuffer + offset, this->hand_yaw);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint8_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->hand_direction[i]));
      }
      for( uint8_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->hand_normal[i]));
      }
      for( uint8_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->hand_palm_pos[i]));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->hand_pitch));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->hand_roll));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->hand_yaw));
     return offset;
    }

    const char * getType(){ return "leap_motion/leap"; };
    const char * getMD5(){ return "3e9a0dc7fd1a98f1d7489e9011c78807"; };

  };

}
#endif