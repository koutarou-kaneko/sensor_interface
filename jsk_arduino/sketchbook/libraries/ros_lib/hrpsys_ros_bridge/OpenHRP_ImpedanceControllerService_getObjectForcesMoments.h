#ifndef _ROS_SERVICE_OpenHRP_ImpedanceControllerService_getObjectForcesMoments_h
#define _ROS_SERVICE_OpenHRP_ImpedanceControllerService_getObjectForcesMoments_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Float64MultiArray.h"

namespace hrpsys_ros_bridge
{

static const char OPENHRP_IMPEDANCECONTROLLERSERVICE_GETOBJECTFORCESMOMENTS[] = "hrpsys_ros_bridge/OpenHRP_ImpedanceControllerService_getObjectForcesMoments";

  class OpenHRP_ImpedanceControllerService_getObjectForcesMomentsRequest : public ros::Msg
  {
    public:

    OpenHRP_ImpedanceControllerService_getObjectForcesMomentsRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return OPENHRP_IMPEDANCECONTROLLERSERVICE_GETOBJECTFORCESMOMENTS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class OpenHRP_ImpedanceControllerService_getObjectForcesMomentsResponse : public ros::Msg
  {
    public:
      bool operation_return;
      std_msgs::Float64MultiArray o_forces;
      std_msgs::Float64MultiArray o_moments;

    OpenHRP_ImpedanceControllerService_getObjectForcesMomentsResponse():
      operation_return(0),
      o_forces(),
      o_moments()
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
      offset += this->o_forces.serialize(outbuffer + offset);
      offset += this->o_moments.serialize(outbuffer + offset);
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
      offset += this->o_forces.deserialize(inbuffer + offset);
      offset += this->o_moments.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return OPENHRP_IMPEDANCECONTROLLERSERVICE_GETOBJECTFORCESMOMENTS; };
    const char * getMD5(){ return "75949020c4f47d94d5dc7cd6425eec88"; };

  };

  class OpenHRP_ImpedanceControllerService_getObjectForcesMoments {
    public:
    typedef OpenHRP_ImpedanceControllerService_getObjectForcesMomentsRequest Request;
    typedef OpenHRP_ImpedanceControllerService_getObjectForcesMomentsResponse Response;
  };

}
#endif
