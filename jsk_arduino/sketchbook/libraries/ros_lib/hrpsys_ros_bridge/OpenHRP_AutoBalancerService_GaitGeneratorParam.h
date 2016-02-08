#ifndef _ROS_hrpsys_ros_bridge_OpenHRP_AutoBalancerService_GaitGeneratorParam_h
#define _ROS_hrpsys_ros_bridge_OpenHRP_AutoBalancerService_GaitGeneratorParam_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Float64MultiArray.h"

namespace hrpsys_ros_bridge
{

  class OpenHRP_AutoBalancerService_GaitGeneratorParam : public ros::Msg
  {
    public:
      float default_step_time;
      float default_step_height;
      float default_double_support_ratio;
      float default_double_support_ratio_before;
      float default_double_support_ratio_after;
      float default_double_support_static_ratio;
      float default_double_support_static_ratio_before;
      float default_double_support_static_ratio_after;
      float default_double_support_ratio_swing_before;
      float default_double_support_ratio_swing_after;
      float stride_parameter[4];
      int64_t default_orbit_type;
      float swing_trajectory_delay_time_offset;
      float swing_trajectory_final_distance_weight;
      float stair_trajectory_way_point_offset[3];
      float cycloid_delay_kick_point_offset[3];
      float gravitational_acceleration;
      float toe_pos_offset_x;
      float heel_pos_offset_x;
      float toe_zmp_offset_x;
      float heel_zmp_offset_x;
      float toe_angle;
      float heel_angle;
      uint8_t toe_heel_phase_ratio_length;
      float st_toe_heel_phase_ratio;
      float * toe_heel_phase_ratio;
      bool use_toe_joint;
      bool use_toe_heel_transition;
      float zmp_weight_map[4];
      std_msgs::Float64MultiArray leg_default_translate_pos;
      int32_t optional_go_pos_finalize_footstep_num;

    OpenHRP_AutoBalancerService_GaitGeneratorParam():
      default_step_time(0),
      default_step_height(0),
      default_double_support_ratio(0),
      default_double_support_ratio_before(0),
      default_double_support_ratio_after(0),
      default_double_support_static_ratio(0),
      default_double_support_static_ratio_before(0),
      default_double_support_static_ratio_after(0),
      default_double_support_ratio_swing_before(0),
      default_double_support_ratio_swing_after(0),
      stride_parameter(),
      default_orbit_type(0),
      swing_trajectory_delay_time_offset(0),
      swing_trajectory_final_distance_weight(0),
      stair_trajectory_way_point_offset(),
      cycloid_delay_kick_point_offset(),
      gravitational_acceleration(0),
      toe_pos_offset_x(0),
      heel_pos_offset_x(0),
      toe_zmp_offset_x(0),
      heel_zmp_offset_x(0),
      toe_angle(0),
      heel_angle(0),
      toe_heel_phase_ratio_length(0), toe_heel_phase_ratio(NULL),
      use_toe_joint(0),
      use_toe_heel_transition(0),
      zmp_weight_map(),
      leg_default_translate_pos(),
      optional_go_pos_finalize_footstep_num(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->default_step_time);
      offset += serializeAvrFloat64(outbuffer + offset, this->default_step_height);
      offset += serializeAvrFloat64(outbuffer + offset, this->default_double_support_ratio);
      offset += serializeAvrFloat64(outbuffer + offset, this->default_double_support_ratio_before);
      offset += serializeAvrFloat64(outbuffer + offset, this->default_double_support_ratio_after);
      offset += serializeAvrFloat64(outbuffer + offset, this->default_double_support_static_ratio);
      offset += serializeAvrFloat64(outbuffer + offset, this->default_double_support_static_ratio_before);
      offset += serializeAvrFloat64(outbuffer + offset, this->default_double_support_static_ratio_after);
      offset += serializeAvrFloat64(outbuffer + offset, this->default_double_support_ratio_swing_before);
      offset += serializeAvrFloat64(outbuffer + offset, this->default_double_support_ratio_swing_after);
      for( uint8_t i = 0; i < 4; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->stride_parameter[i]);
      }
      union {
        int64_t real;
        uint64_t base;
      } u_default_orbit_type;
      u_default_orbit_type.real = this->default_orbit_type;
      *(outbuffer + offset + 0) = (u_default_orbit_type.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_default_orbit_type.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_default_orbit_type.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_default_orbit_type.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_default_orbit_type.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_default_orbit_type.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_default_orbit_type.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_default_orbit_type.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->default_orbit_type);
      offset += serializeAvrFloat64(outbuffer + offset, this->swing_trajectory_delay_time_offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->swing_trajectory_final_distance_weight);
      for( uint8_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->stair_trajectory_way_point_offset[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->cycloid_delay_kick_point_offset[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->gravitational_acceleration);
      offset += serializeAvrFloat64(outbuffer + offset, this->toe_pos_offset_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->heel_pos_offset_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->toe_zmp_offset_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->heel_zmp_offset_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->toe_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->heel_angle);
      *(outbuffer + offset++) = toe_heel_phase_ratio_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < toe_heel_phase_ratio_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->toe_heel_phase_ratio[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_use_toe_joint;
      u_use_toe_joint.real = this->use_toe_joint;
      *(outbuffer + offset + 0) = (u_use_toe_joint.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->use_toe_joint);
      union {
        bool real;
        uint8_t base;
      } u_use_toe_heel_transition;
      u_use_toe_heel_transition.real = this->use_toe_heel_transition;
      *(outbuffer + offset + 0) = (u_use_toe_heel_transition.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->use_toe_heel_transition);
      for( uint8_t i = 0; i < 4; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->zmp_weight_map[i]);
      }
      offset += this->leg_default_translate_pos.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_optional_go_pos_finalize_footstep_num;
      u_optional_go_pos_finalize_footstep_num.real = this->optional_go_pos_finalize_footstep_num;
      *(outbuffer + offset + 0) = (u_optional_go_pos_finalize_footstep_num.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_optional_go_pos_finalize_footstep_num.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_optional_go_pos_finalize_footstep_num.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_optional_go_pos_finalize_footstep_num.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->optional_go_pos_finalize_footstep_num);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->default_step_time));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->default_step_height));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->default_double_support_ratio));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->default_double_support_ratio_before));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->default_double_support_ratio_after));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->default_double_support_static_ratio));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->default_double_support_static_ratio_before));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->default_double_support_static_ratio_after));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->default_double_support_ratio_swing_before));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->default_double_support_ratio_swing_after));
      for( uint8_t i = 0; i < 4; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->stride_parameter[i]));
      }
      union {
        int64_t real;
        uint64_t base;
      } u_default_orbit_type;
      u_default_orbit_type.base = 0;
      u_default_orbit_type.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_default_orbit_type.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_default_orbit_type.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_default_orbit_type.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_default_orbit_type.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_default_orbit_type.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_default_orbit_type.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_default_orbit_type.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->default_orbit_type = u_default_orbit_type.real;
      offset += sizeof(this->default_orbit_type);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->swing_trajectory_delay_time_offset));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->swing_trajectory_final_distance_weight));
      for( uint8_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->stair_trajectory_way_point_offset[i]));
      }
      for( uint8_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->cycloid_delay_kick_point_offset[i]));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->gravitational_acceleration));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->toe_pos_offset_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->heel_pos_offset_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->toe_zmp_offset_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->heel_zmp_offset_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->toe_angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->heel_angle));
      uint8_t toe_heel_phase_ratio_lengthT = *(inbuffer + offset++);
      if(toe_heel_phase_ratio_lengthT > toe_heel_phase_ratio_length)
        this->toe_heel_phase_ratio = (float*)realloc(this->toe_heel_phase_ratio, toe_heel_phase_ratio_lengthT * sizeof(float));
      offset += 3;
      toe_heel_phase_ratio_length = toe_heel_phase_ratio_lengthT;
      for( uint8_t i = 0; i < toe_heel_phase_ratio_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_toe_heel_phase_ratio));
        memcpy( &(this->toe_heel_phase_ratio[i]), &(this->st_toe_heel_phase_ratio), sizeof(float));
      }
      union {
        bool real;
        uint8_t base;
      } u_use_toe_joint;
      u_use_toe_joint.base = 0;
      u_use_toe_joint.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->use_toe_joint = u_use_toe_joint.real;
      offset += sizeof(this->use_toe_joint);
      union {
        bool real;
        uint8_t base;
      } u_use_toe_heel_transition;
      u_use_toe_heel_transition.base = 0;
      u_use_toe_heel_transition.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->use_toe_heel_transition = u_use_toe_heel_transition.real;
      offset += sizeof(this->use_toe_heel_transition);
      for( uint8_t i = 0; i < 4; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->zmp_weight_map[i]));
      }
      offset += this->leg_default_translate_pos.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_optional_go_pos_finalize_footstep_num;
      u_optional_go_pos_finalize_footstep_num.base = 0;
      u_optional_go_pos_finalize_footstep_num.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_optional_go_pos_finalize_footstep_num.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_optional_go_pos_finalize_footstep_num.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_optional_go_pos_finalize_footstep_num.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->optional_go_pos_finalize_footstep_num = u_optional_go_pos_finalize_footstep_num.real;
      offset += sizeof(this->optional_go_pos_finalize_footstep_num);
     return offset;
    }

    const char * getType(){ return "hrpsys_ros_bridge/OpenHRP_AutoBalancerService_GaitGeneratorParam"; };
    const char * getMD5(){ return "982f9c39a5e8b26fbb5adeec86e7c6ce"; };

  };

}
#endif