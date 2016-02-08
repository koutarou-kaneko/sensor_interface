#ifndef _ROS_SERVICE_OpenHRP_TorqueControllerService_setTorqueControllerParam_h
#define _ROS_SERVICE_OpenHRP_TorqueControllerService_setTorqueControllerParam_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "hrpsys_ros_bridge/OpenHRP_TorqueControllerService_torqueControllerParam.h"

namespace hrpsys_ros_bridge
{

static const char OPENHRP_TORQUECONTROLLERSERVICE_SETTORQUECONTROLLERPARAM[] = "hrpsys_ros_bridge/OpenHRP_TorqueControllerService_setTorqueControllerParam";

  class OpenHRP_TorqueControllerService_setTorqueControllerParamRequest : public ros::Msg
  {
    public:
      hrpsys_ros_bridge::OpenHRP_TorqueControllerService_torqueControllerParam t_param;

    OpenHRP_TorqueControllerService_setTorqueControllerParamRequest():
      t_param()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->t_param.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->t_param.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return OPENHRP_TORQUECONTROLLERSERVICE_SETTORQUECONTROLLERPARAM; };
    const char * getMD5(){ return "a0c1ff6b736d20763d71cd1030c8b233"; };

  };

  class OpenHRP_TorqueControllerService_setTorqueControllerParamResponse : public ros::Msg
  {
    public:
      bool operation_return;

    OpenHRP_TorqueControllerService_setTorqueControllerParamResponse():
      operation_return(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_operation_return;
      u_operation_return.real = this->operation_return;
      *(outbuffer + offset + 0) = (u_operation_return.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->operation_return);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_operation_return;
      u_operation_return.base = 0;
      u_operation_return.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->operation_return = u_operation_return.real;
      offset += sizeof(this->operation_return);
     return offset;
    }

    const char * getType(){ return OPENHRP_TORQUECONTROLLERSERVICE_SETTORQUECONTROLLERPARAM; };
    const char * getMD5(){ return "8dd59ee39c15084c92106411b8c3e8fc"; };

  };

  class OpenHRP_TorqueControllerService_setTorqueControllerParam {
    public:
    typedef OpenHRP_TorqueControllerService_setTorqueControllerParamRequest Request;
    typedef OpenHRP_TorqueControllerService_setTorqueControllerParamResponse Response;
  };

}
#endif
