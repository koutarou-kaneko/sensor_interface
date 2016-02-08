#ifndef _ROS_hrpsys_ros_bridge_ContactState_h
#define _ROS_hrpsys_ros_bridge_ContactState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

  class ContactState : public ros::Msg
  {
    public:
      uint8_t state;
      float remaining_time;
      enum { OFF = 0 };
      enum { ON = 1 };

    ContactState():
      state(0),
      remaining_time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      offset += serializeAvrFloat64(outbuffer + offset, this->remaining_time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->remaining_time));
     return offset;
    }

    const char * getType(){ return "hrpsys_ros_bridge/ContactState"; };
    const char * getMD5(){ return "fb79aca4abc1c93c62c57fb11cb443a0"; };

  };

}
#endif