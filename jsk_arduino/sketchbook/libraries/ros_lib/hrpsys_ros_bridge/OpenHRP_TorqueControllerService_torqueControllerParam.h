#ifndef _ROS_hrpsys_ros_bridge_OpenHRP_TorqueControllerService_torqueControllerParam_h
#define _ROS_hrpsys_ros_bridge_OpenHRP_TorqueControllerService_torqueControllerParam_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

  class OpenHRP_TorqueControllerService_torqueControllerParam : public ros::Msg
  {
    public:
      const char* name;
      float tc;
      float ke;
      float kd;
      float ki;
      float alpha;
      float beta;

    OpenHRP_TorqueControllerService_torqueControllerParam():
      name(""),
      tc(0),
      ke(0),
      kd(0),
      ki(0),
      alpha(0),
      beta(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      offset += serializeAvrFloat64(outbuffer + offset, this->tc);
      offset += serializeAvrFloat64(outbuffer + offset, this->ke);
      offset += serializeAvrFloat64(outbuffer + offset, this->kd);
      offset += serializeAvrFloat64(outbuffer + offset, this->ki);
      offset += serializeAvrFloat64(outbuffer + offset, this->alpha);
      offset += serializeAvrFloat64(outbuffer + offset, this->beta);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tc));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ke));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->kd));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ki));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->alpha));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->beta));
     return offset;
    }

    const char * getType(){ return "hrpsys_ros_bridge/OpenHRP_TorqueControllerService_torqueControllerParam"; };
    const char * getMD5(){ return "c2c93ee8385c97a0f222304c89369c30"; };

  };

}
#endif