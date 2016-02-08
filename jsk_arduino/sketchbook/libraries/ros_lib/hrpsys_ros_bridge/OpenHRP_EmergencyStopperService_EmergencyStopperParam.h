#ifndef _ROS_hrpsys_ros_bridge_OpenHRP_EmergencyStopperService_EmergencyStopperParam_h
#define _ROS_hrpsys_ros_bridge_OpenHRP_EmergencyStopperService_EmergencyStopperParam_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

  class OpenHRP_EmergencyStopperService_EmergencyStopperParam : public ros::Msg
  {
    public:
      float default_recover_time;
      float default_retrieve_time;
      bool is_stop_mode;

    OpenHRP_EmergencyStopperService_EmergencyStopperParam():
      default_recover_time(0),
      default_retrieve_time(0),
      is_stop_mode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->default_recover_time);
      offset += serializeAvrFloat64(outbuffer + offset, this->default_retrieve_time);
      union {
        bool real;
        uint8_t base;
      } u_is_stop_mode;
      u_is_stop_mode.real = this->is_stop_mode;
      *(outbuffer + offset + 0) = (u_is_stop_mode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_stop_mode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->default_recover_time));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->default_retrieve_time));
      union {
        bool real;
        uint8_t base;
      } u_is_stop_mode;
      u_is_stop_mode.base = 0;
      u_is_stop_mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_stop_mode = u_is_stop_mode.real;
      offset += sizeof(this->is_stop_mode);
     return offset;
    }

    const char * getType(){ return "hrpsys_ros_bridge/OpenHRP_EmergencyStopperService_EmergencyStopperParam"; };
    const char * getMD5(){ return "9e7993b5ea68e78c2c5246f4a2a5f844"; };

  };

}
#endif