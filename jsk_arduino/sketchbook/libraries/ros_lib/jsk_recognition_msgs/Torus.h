#ifndef _ROS_jsk_recognition_msgs_Torus_h
#define _ROS_jsk_recognition_msgs_Torus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"

namespace jsk_recognition_msgs
{

  class Torus : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Pose pose;
      float large_radius;
      float small_radius;

    Torus():
      header(),
      pose(),
      large_radius(0),
      small_radius(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->large_radius);
      offset += serializeAvrFloat64(outbuffer + offset, this->small_radius);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->large_radius));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->small_radius));
     return offset;
    }

    const char * getType(){ return "jsk_recognition_msgs/Torus"; };
    const char * getMD5(){ return "641f1779f70127023d391e4b09c3628b"; };

  };

}
#endif