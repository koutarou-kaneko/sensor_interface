#ifndef _ROS_leap_motion_leapros_h
#define _ROS_leap_motion_leapros_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"

namespace leap_motion
{

  class leapros : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Vector3 direction;
      geometry_msgs::Vector3 normal;
      geometry_msgs::Point palmpos;
      geometry_msgs::Vector3 ypr;

    leapros():
      header(),
      direction(),
      normal(),
      palmpos(),
      ypr()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->direction.serialize(outbuffer + offset);
      offset += this->normal.serialize(outbuffer + offset);
      offset += this->palmpos.serialize(outbuffer + offset);
      offset += this->ypr.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->direction.deserialize(inbuffer + offset);
      offset += this->normal.deserialize(inbuffer + offset);
      offset += this->palmpos.deserialize(inbuffer + offset);
      offset += this->ypr.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "leap_motion/leapros"; };
    const char * getMD5(){ return "25ef29a0ceca587154a48677ba8488b6"; };

  };

}
#endif