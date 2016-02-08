#ifndef _ROS_hrpsys_ros_bridge_RTC_TimedDoubleSeq_h
#define _ROS_hrpsys_ros_bridge_RTC_TimedDoubleSeq_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "hrpsys_ros_bridge/RTC_Time.h"

namespace hrpsys_ros_bridge
{

  class RTC_TimedDoubleSeq : public ros::Msg
  {
    public:
      hrpsys_ros_bridge::RTC_Time tm;
      uint8_t data_length;
      float st_data;
      float * data;

    RTC_TimedDoubleSeq():
      tm(),
      data_length(0), data(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->tm.serialize(outbuffer + offset);
      *(outbuffer + offset++) = data_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < data_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->tm.deserialize(inbuffer + offset);
      uint8_t data_lengthT = *(inbuffer + offset++);
      if(data_lengthT > data_length)
        this->data = (float*)realloc(this->data, data_lengthT * sizeof(float));
      offset += 3;
      data_length = data_lengthT;
      for( uint8_t i = 0; i < data_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_data));
        memcpy( &(this->data[i]), &(this->st_data), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "hrpsys_ros_bridge/RTC_TimedDoubleSeq"; };
    const char * getMD5(){ return "d9e01689e17707d99ff596edf800755c"; };

  };

}
#endif