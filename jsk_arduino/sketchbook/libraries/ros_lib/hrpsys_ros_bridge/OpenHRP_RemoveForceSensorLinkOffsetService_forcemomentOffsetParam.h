#ifndef _ROS_hrpsys_ros_bridge_OpenHRP_RemoveForceSensorLinkOffsetService_forcemomentOffsetParam_h
#define _ROS_hrpsys_ros_bridge_OpenHRP_RemoveForceSensorLinkOffsetService_forcemomentOffsetParam_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

  class OpenHRP_RemoveForceSensorLinkOffsetService_forcemomentOffsetParam : public ros::Msg
  {
    public:
      float force_offset[3];
      float moment_offset[3];
      float link_offset_centroid[3];
      float link_offset_mass;

    OpenHRP_RemoveForceSensorLinkOffsetService_forcemomentOffsetParam():
      force_offset(),
      moment_offset(),
      link_offset_centroid(),
      link_offset_mass(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint8_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->force_offset[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->moment_offset[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->link_offset_centroid[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->link_offset_mass);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint8_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->force_offset[i]));
      }
      for( uint8_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->moment_offset[i]));
      }
      for( uint8_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->link_offset_centroid[i]));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->link_offset_mass));
     return offset;
    }

    const char * getType(){ return "hrpsys_ros_bridge/OpenHRP_RemoveForceSensorLinkOffsetService_forcemomentOffsetParam"; };
    const char * getMD5(){ return "65a8bdda0c275a081765814539fb2401"; };

  };

}
#endif