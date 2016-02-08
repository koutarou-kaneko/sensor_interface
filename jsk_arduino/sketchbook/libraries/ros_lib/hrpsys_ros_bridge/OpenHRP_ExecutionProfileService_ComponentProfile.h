#ifndef _ROS_hrpsys_ros_bridge_OpenHRP_ExecutionProfileService_ComponentProfile_h
#define _ROS_hrpsys_ros_bridge_OpenHRP_ExecutionProfileService_ComponentProfile_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

  class OpenHRP_ExecutionProfileService_ComponentProfile : public ros::Msg
  {
    public:
      int32_t count;
      float max_process;
      float avg_process;

    OpenHRP_ExecutionProfileService_ComponentProfile():
      count(0),
      max_process(0),
      avg_process(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_count;
      u_count.real = this->count;
      *(outbuffer + offset + 0) = (u_count.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_count.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_count.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_count.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->count);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_process);
      offset += serializeAvrFloat64(outbuffer + offset, this->avg_process);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_count;
      u_count.base = 0;
      u_count.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_count.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_count.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_count.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->count = u_count.real;
      offset += sizeof(this->count);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_process));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->avg_process));
     return offset;
    }

    const char * getType(){ return "hrpsys_ros_bridge/OpenHRP_ExecutionProfileService_ComponentProfile"; };
    const char * getMD5(){ return "2756437d1ed9f4a80e8165d05e369c5a"; };

  };

}
#endif