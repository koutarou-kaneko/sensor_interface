#ifndef _ROS_arm_navigation_msgs_SimplePoseConstraint_h
#define _ROS_arm_navigation_msgs_SimplePoseConstraint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

namespace arm_navigation_msgs
{

  class SimplePoseConstraint : public ros::Msg
  {
    public:
      std_msgs::Header header;
      const char* link_name;
      geometry_msgs::Pose pose;
      geometry_msgs::Point absolute_position_tolerance;
      float absolute_roll_tolerance;
      float absolute_yaw_tolerance;
      float absolute_pitch_tolerance;
      int32_t orientation_constraint_type;
      enum { HEADER_FRAME = 0 };
      enum { LINK_FRAME = 1 };

    SimplePoseConstraint():
      header(),
      link_name(""),
      pose(),
      absolute_position_tolerance(),
      absolute_roll_tolerance(0),
      absolute_yaw_tolerance(0),
      absolute_pitch_tolerance(0),
      orientation_constraint_type(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_link_name = strlen(this->link_name);
      memcpy(outbuffer + offset, &length_link_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->link_name, length_link_name);
      offset += length_link_name;
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->absolute_position_tolerance.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->absolute_roll_tolerance);
      offset += serializeAvrFloat64(outbuffer + offset, this->absolute_yaw_tolerance);
      offset += serializeAvrFloat64(outbuffer + offset, this->absolute_pitch_tolerance);
      union {
        int32_t real;
        uint32_t base;
      } u_orientation_constraint_type;
      u_orientation_constraint_type.real = this->orientation_constraint_type;
      *(outbuffer + offset + 0) = (u_orientation_constraint_type.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_orientation_constraint_type.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_orientation_constraint_type.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_orientation_constraint_type.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->orientation_constraint_type);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_link_name;
      memcpy(&length_link_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_link_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_link_name-1]=0;
      this->link_name = (char *)(inbuffer + offset-1);
      offset += length_link_name;
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->absolute_position_tolerance.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->absolute_roll_tolerance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->absolute_yaw_tolerance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->absolute_pitch_tolerance));
      union {
        int32_t real;
        uint32_t base;
      } u_orientation_constraint_type;
      u_orientation_constraint_type.base = 0;
      u_orientation_constraint_type.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_orientation_constraint_type.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_orientation_constraint_type.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_orientation_constraint_type.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->orientation_constraint_type = u_orientation_constraint_type.real;
      offset += sizeof(this->orientation_constraint_type);
     return offset;
    }

    const char * getType(){ return "arm_navigation_msgs/SimplePoseConstraint"; };
    const char * getMD5(){ return "3483d830eb84ecd3059741fd417b30da"; };

  };

}
#endif