#ifndef _ROS_jsk_footstep_controller_FootCoordsLowLevelInfo_h
#define _ROS_jsk_footstep_controller_FootCoordsLowLevelInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace jsk_footstep_controller
{

  class FootCoordsLowLevelInfo : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float raw_lleg_force;
      float raw_rleg_force;
      float filtered_lleg_force;
      float filtered_rleg_force;
      float threshold;

    FootCoordsLowLevelInfo():
      header(),
      raw_lleg_force(0),
      raw_rleg_force(0),
      filtered_lleg_force(0),
      filtered_rleg_force(0),
      threshold(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->raw_lleg_force);
      offset += serializeAvrFloat64(outbuffer + offset, this->raw_rleg_force);
      offset += serializeAvrFloat64(outbuffer + offset, this->filtered_lleg_force);
      offset += serializeAvrFloat64(outbuffer + offset, this->filtered_rleg_force);
      offset += serializeAvrFloat64(outbuffer + offset, this->threshold);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->raw_lleg_force));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->raw_rleg_force));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->filtered_lleg_force));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->filtered_rleg_force));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->threshold));
     return offset;
    }

    const char * getType(){ return "jsk_footstep_controller/FootCoordsLowLevelInfo"; };
    const char * getMD5(){ return "f03f7aaafde613e7d2247f1ee6314403"; };

  };

}
#endif