#ifndef _ROS_hrpsys_ros_bridge_OpenHRP_AutoBalancerService_Footstep_h
#define _ROS_hrpsys_ros_bridge_OpenHRP_AutoBalancerService_Footstep_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

  class OpenHRP_AutoBalancerService_Footstep : public ros::Msg
  {
    public:
      float pos[3];
      float rot[4];
      const char* leg;

    OpenHRP_AutoBalancerService_Footstep():
      pos(),
      rot(),
      leg("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint8_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->pos[i]);
      }
      for( uint8_t i = 0; i < 4; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->rot[i]);
      }
      uint32_t length_leg = strlen(this->leg);
      memcpy(outbuffer + offset, &length_leg, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->leg, length_leg);
      offset += length_leg;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint8_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pos[i]));
      }
      for( uint8_t i = 0; i < 4; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->rot[i]));
      }
      uint32_t length_leg;
      memcpy(&length_leg, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_leg; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_leg-1]=0;
      this->leg = (char *)(inbuffer + offset-1);
      offset += length_leg;
     return offset;
    }

    const char * getType(){ return "hrpsys_ros_bridge/OpenHRP_AutoBalancerService_Footstep"; };
    const char * getMD5(){ return "f65a0e5bbf029cf10add63f7ae123997"; };

  };

}
#endif