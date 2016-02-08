#ifndef _ROS_hrpsys_ros_bridge_OpenHRP_ImpedanceControllerService_objectTurnaroundDetectorParam_h
#define _ROS_hrpsys_ros_bridge_OpenHRP_ImpedanceControllerService_objectTurnaroundDetectorParam_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

  class OpenHRP_ImpedanceControllerService_objectTurnaroundDetectorParam : public ros::Msg
  {
    public:
      float wrench_cutoff_freq;
      float dwrench_cutoff_freq;
      float detect_ratio_thre;
      float start_ratio_thre;
      float axis[3];

    OpenHRP_ImpedanceControllerService_objectTurnaroundDetectorParam():
      wrench_cutoff_freq(0),
      dwrench_cutoff_freq(0),
      detect_ratio_thre(0),
      start_ratio_thre(0),
      axis()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->wrench_cutoff_freq);
      offset += serializeAvrFloat64(outbuffer + offset, this->dwrench_cutoff_freq);
      offset += serializeAvrFloat64(outbuffer + offset, this->detect_ratio_thre);
      offset += serializeAvrFloat64(outbuffer + offset, this->start_ratio_thre);
      for( uint8_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->axis[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->wrench_cutoff_freq));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->dwrench_cutoff_freq));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->detect_ratio_thre));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->start_ratio_thre));
      for( uint8_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->axis[i]));
      }
     return offset;
    }

    const char * getType(){ return "hrpsys_ros_bridge/OpenHRP_ImpedanceControllerService_objectTurnaroundDetectorParam"; };
    const char * getMD5(){ return "5ce71643dadefc08a0ef5a6debef4f18"; };

  };

}
#endif