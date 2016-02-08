#ifndef _ROS_arm_navigation_msgs_VisibilityConstraint_h
#define _ROS_arm_navigation_msgs_VisibilityConstraint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"

namespace arm_navigation_msgs
{

  class VisibilityConstraint : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::PointStamped target;
      geometry_msgs::PoseStamped sensor_pose;
      float absolute_tolerance;

    VisibilityConstraint():
      header(),
      target(),
      sensor_pose(),
      absolute_tolerance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->target.serialize(outbuffer + offset);
      offset += this->sensor_pose.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->absolute_tolerance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->target.deserialize(inbuffer + offset);
      offset += this->sensor_pose.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->absolute_tolerance));
     return offset;
    }

    const char * getType(){ return "arm_navigation_msgs/VisibilityConstraint"; };
    const char * getMD5(){ return "ab297b6588ea21c1a862067d8447cb08"; };

  };

}
#endif