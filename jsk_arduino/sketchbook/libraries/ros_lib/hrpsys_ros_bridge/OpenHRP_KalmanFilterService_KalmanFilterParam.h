#ifndef _ROS_hrpsys_ros_bridge_OpenHRP_KalmanFilterService_KalmanFilterParam_h
#define _ROS_hrpsys_ros_bridge_OpenHRP_KalmanFilterService_KalmanFilterParam_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

  class OpenHRP_KalmanFilterService_KalmanFilterParam : public ros::Msg
  {
    public:
      float Q_angle;
      float Q_rate;
      float R_angle;
      int64_t kf_algorithm;
      float acc_offset[3];
      float sensorRPY_offset[3];

    OpenHRP_KalmanFilterService_KalmanFilterParam():
      Q_angle(0),
      Q_rate(0),
      R_angle(0),
      kf_algorithm(0),
      acc_offset(),
      sensorRPY_offset()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->Q_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->Q_rate);
      offset += serializeAvrFloat64(outbuffer + offset, this->R_angle);
      union {
        int64_t real;
        uint64_t base;
      } u_kf_algorithm;
      u_kf_algorithm.real = this->kf_algorithm;
      *(outbuffer + offset + 0) = (u_kf_algorithm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kf_algorithm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kf_algorithm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kf_algorithm.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_kf_algorithm.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_kf_algorithm.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_kf_algorithm.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_kf_algorithm.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->kf_algorithm);
      for( uint8_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->acc_offset[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->sensorRPY_offset[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Q_angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Q_rate));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->R_angle));
      union {
        int64_t real;
        uint64_t base;
      } u_kf_algorithm;
      u_kf_algorithm.base = 0;
      u_kf_algorithm.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kf_algorithm.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kf_algorithm.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kf_algorithm.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_kf_algorithm.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_kf_algorithm.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_kf_algorithm.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_kf_algorithm.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->kf_algorithm = u_kf_algorithm.real;
      offset += sizeof(this->kf_algorithm);
      for( uint8_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->acc_offset[i]));
      }
      for( uint8_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->sensorRPY_offset[i]));
      }
     return offset;
    }

    const char * getType(){ return "hrpsys_ros_bridge/OpenHRP_KalmanFilterService_KalmanFilterParam"; };
    const char * getMD5(){ return "9e2ec644e20575e0461c6b3cde1053fe"; };

  };

}
#endif