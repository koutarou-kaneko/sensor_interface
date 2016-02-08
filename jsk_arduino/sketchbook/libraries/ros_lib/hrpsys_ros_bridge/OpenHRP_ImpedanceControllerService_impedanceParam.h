#ifndef _ROS_hrpsys_ros_bridge_OpenHRP_ImpedanceControllerService_impedanceParam_h
#define _ROS_hrpsys_ros_bridge_OpenHRP_ImpedanceControllerService_impedanceParam_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

  class OpenHRP_ImpedanceControllerService_impedanceParam : public ros::Msg
  {
    public:
      float M_p;
      float D_p;
      float K_p;
      float M_r;
      float D_r;
      float K_r;
      float force_gain[3];
      float moment_gain[3];
      float sr_gain;
      float avoid_gain;
      float reference_gain;
      float manipulability_limit;
      int64_t controller_mode;
      uint8_t ik_optional_weight_vector_length;
      float st_ik_optional_weight_vector;
      float * ik_optional_weight_vector;
      bool use_sh_base_pos_rpy;

    OpenHRP_ImpedanceControllerService_impedanceParam():
      M_p(0),
      D_p(0),
      K_p(0),
      M_r(0),
      D_r(0),
      K_r(0),
      force_gain(),
      moment_gain(),
      sr_gain(0),
      avoid_gain(0),
      reference_gain(0),
      manipulability_limit(0),
      controller_mode(0),
      ik_optional_weight_vector_length(0), ik_optional_weight_vector(NULL),
      use_sh_base_pos_rpy(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->M_p);
      offset += serializeAvrFloat64(outbuffer + offset, this->D_p);
      offset += serializeAvrFloat64(outbuffer + offset, this->K_p);
      offset += serializeAvrFloat64(outbuffer + offset, this->M_r);
      offset += serializeAvrFloat64(outbuffer + offset, this->D_r);
      offset += serializeAvrFloat64(outbuffer + offset, this->K_r);
      for( uint8_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->force_gain[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->moment_gain[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->sr_gain);
      offset += serializeAvrFloat64(outbuffer + offset, this->avoid_gain);
      offset += serializeAvrFloat64(outbuffer + offset, this->reference_gain);
      offset += serializeAvrFloat64(outbuffer + offset, this->manipulability_limit);
      union {
        int64_t real;
        uint64_t base;
      } u_controller_mode;
      u_controller_mode.real = this->controller_mode;
      *(outbuffer + offset + 0) = (u_controller_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_controller_mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_controller_mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_controller_mode.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_controller_mode.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_controller_mode.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_controller_mode.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_controller_mode.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->controller_mode);
      *(outbuffer + offset++) = ik_optional_weight_vector_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < ik_optional_weight_vector_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->ik_optional_weight_vector[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_use_sh_base_pos_rpy;
      u_use_sh_base_pos_rpy.real = this->use_sh_base_pos_rpy;
      *(outbuffer + offset + 0) = (u_use_sh_base_pos_rpy.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->use_sh_base_pos_rpy);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->M_p));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->D_p));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->K_p));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->M_r));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->D_r));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->K_r));
      for( uint8_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->force_gain[i]));
      }
      for( uint8_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->moment_gain[i]));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->sr_gain));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->avoid_gain));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->reference_gain));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->manipulability_limit));
      union {
        int64_t real;
        uint64_t base;
      } u_controller_mode;
      u_controller_mode.base = 0;
      u_controller_mode.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_controller_mode.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_controller_mode.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_controller_mode.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_controller_mode.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_controller_mode.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_controller_mode.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_controller_mode.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->controller_mode = u_controller_mode.real;
      offset += sizeof(this->controller_mode);
      uint8_t ik_optional_weight_vector_lengthT = *(inbuffer + offset++);
      if(ik_optional_weight_vector_lengthT > ik_optional_weight_vector_length)
        this->ik_optional_weight_vector = (float*)realloc(this->ik_optional_weight_vector, ik_optional_weight_vector_lengthT * sizeof(float));
      offset += 3;
      ik_optional_weight_vector_length = ik_optional_weight_vector_lengthT;
      for( uint8_t i = 0; i < ik_optional_weight_vector_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_ik_optional_weight_vector));
        memcpy( &(this->ik_optional_weight_vector[i]), &(this->st_ik_optional_weight_vector), sizeof(float));
      }
      union {
        bool real;
        uint8_t base;
      } u_use_sh_base_pos_rpy;
      u_use_sh_base_pos_rpy.base = 0;
      u_use_sh_base_pos_rpy.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->use_sh_base_pos_rpy = u_use_sh_base_pos_rpy.real;
      offset += sizeof(this->use_sh_base_pos_rpy);
     return offset;
    }

    const char * getType(){ return "hrpsys_ros_bridge/OpenHRP_ImpedanceControllerService_impedanceParam"; };
    const char * getMD5(){ return "93910157f4c229ccb0ff5677c4006630"; };

  };

}
#endif