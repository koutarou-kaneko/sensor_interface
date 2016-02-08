#ifndef _ROS_hrpsys_ros_bridge_OpenHRP_StabilizerService_TwoDimensionVertex_h
#define _ROS_hrpsys_ros_bridge_OpenHRP_StabilizerService_TwoDimensionVertex_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

  class OpenHRP_StabilizerService_TwoDimensionVertex : public ros::Msg
  {
    public:
      float pos[2];

    OpenHRP_StabilizerService_TwoDimensionVertex():
      pos()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint8_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->pos[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint8_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pos[i]));
      }
     return offset;
    }

    const char * getType(){ return "hrpsys_ros_bridge/OpenHRP_StabilizerService_TwoDimensionVertex"; };
    const char * getMD5(){ return "8d9688bdfa4b2786a249fae029b876f1"; };

  };

}
#endif