#ifndef _ROS_jsk_footstep_controller_GroundContactState_h
#define _ROS_jsk_footstep_controller_GroundContactState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace jsk_footstep_controller
{

  class GroundContactState : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t contact_state;
      float error_pitch_angle;
      float error_roll_angle;
      float error_yaw_angle;
      float error_z;
      enum { CONTACT_BOTH_GROUND = 1 };
      enum { CONTACT_AIR = 2 };
      enum { CONTACT_LLEG_GROUND = 3 };
      enum { CONTACT_RLEG_GROUND = 4 };
      enum { CONTACT_UNSTABLE = 5 };

    GroundContactState():
      header(),
      contact_state(0),
      error_pitch_angle(0),
      error_roll_angle(0),
      error_yaw_angle(0),
      error_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->contact_state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->contact_state);
      offset += serializeAvrFloat64(outbuffer + offset, this->error_pitch_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->error_roll_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->error_yaw_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->error_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->contact_state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->contact_state);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->error_pitch_angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->error_roll_angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->error_yaw_angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->error_z));
     return offset;
    }

    const char * getType(){ return "jsk_footstep_controller/GroundContactState"; };
    const char * getMD5(){ return "da0f3906e0a6eafe324ba582440493ea"; };

  };

}
#endif