#ifndef _ROS_hrpsys_ros_bridge_OpenHRP_StabilizerService_stParam_h
#define _ROS_hrpsys_ros_bridge_OpenHRP_StabilizerService_stParam_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Float64MultiArray.h"
#include "hrpsys_ros_bridge/OpenHRP_StabilizerService_SupportPolygonVertices.h"

namespace hrpsys_ros_bridge
{

  class OpenHRP_StabilizerService_stParam : public ros::Msg
  {
    public:
      float k_tpcc_p[2];
      float k_tpcc_x[2];
      float k_brot_p[2];
      float k_brot_tc[2];
      float eefm_k1[2];
      float eefm_k2[2];
      float eefm_k3[2];
      float eefm_zmp_delay_time_const[2];
      float eefm_ref_zmp_aux[2];
      std_msgs::Float64MultiArray eefm_rot_damping_gain;
      std_msgs::Float64MultiArray eefm_rot_time_const;
      std_msgs::Float64MultiArray eefm_pos_damping_gain;
      std_msgs::Float64MultiArray eefm_pos_time_const_support;
      uint8_t eefm_pos_compensation_limit_length;
      float st_eefm_pos_compensation_limit;
      float * eefm_pos_compensation_limit;
      uint8_t eefm_rot_compensation_limit_length;
      float st_eefm_rot_compensation_limit;
      float * eefm_rot_compensation_limit;
      float eefm_pos_time_const_swing;
      float eefm_pos_transition_time;
      float eefm_pos_margin_time;
      float eefm_leg_inside_margin;
      float eefm_leg_outside_margin;
      float eefm_leg_front_margin;
      float eefm_leg_rear_margin;
      float eefm_body_attitude_control_gain[2];
      float eefm_body_attitude_control_time_const[2];
      float eefm_cogvel_cutoff_freq;
      float eefm_wrench_alpha_blending;
      float eefm_alpha_cutoff_freq;
      float eefm_gravitational_acceleration;
      float eefm_ee_pos_error_p_gain;
      float eefm_ee_rot_error_p_gain;
      float eefm_ee_error_cutoff_freq;
      uint8_t eefm_support_polygon_vertices_sequence_length;
      hrpsys_ros_bridge::OpenHRP_StabilizerService_SupportPolygonVertices st_eefm_support_polygon_vertices_sequence;
      hrpsys_ros_bridge::OpenHRP_StabilizerService_SupportPolygonVertices * eefm_support_polygon_vertices_sequence;
      bool eefm_use_force_difference_control;
      int64_t st_algorithm;
      int64_t controller_mode;
      float transition_time;
      uint8_t is_ik_enable_length;
      bool st_is_ik_enable;
      bool * is_ik_enable;
      uint8_t is_feedback_control_enable_length;
      bool st_is_feedback_control_enable;
      bool * is_feedback_control_enable;
      uint8_t is_zmp_calc_enable_length;
      bool st_is_zmp_calc_enable;
      bool * is_zmp_calc_enable;
      float cop_check_margin;
      float cp_check_margin;
      float ref_capture_point[2];
      float act_capture_point[2];
      float contact_decision_threshold;
      std_msgs::Float64MultiArray foot_origin_offset;
      int64_t emergency_check_mode;

    OpenHRP_StabilizerService_stParam():
      k_tpcc_p(),
      k_tpcc_x(),
      k_brot_p(),
      k_brot_tc(),
      eefm_k1(),
      eefm_k2(),
      eefm_k3(),
      eefm_zmp_delay_time_const(),
      eefm_ref_zmp_aux(),
      eefm_rot_damping_gain(),
      eefm_rot_time_const(),
      eefm_pos_damping_gain(),
      eefm_pos_time_const_support(),
      eefm_pos_compensation_limit_length(0), eefm_pos_compensation_limit(NULL),
      eefm_rot_compensation_limit_length(0), eefm_rot_compensation_limit(NULL),
      eefm_pos_time_const_swing(0),
      eefm_pos_transition_time(0),
      eefm_pos_margin_time(0),
      eefm_leg_inside_margin(0),
      eefm_leg_outside_margin(0),
      eefm_leg_front_margin(0),
      eefm_leg_rear_margin(0),
      eefm_body_attitude_control_gain(),
      eefm_body_attitude_control_time_const(),
      eefm_cogvel_cutoff_freq(0),
      eefm_wrench_alpha_blending(0),
      eefm_alpha_cutoff_freq(0),
      eefm_gravitational_acceleration(0),
      eefm_ee_pos_error_p_gain(0),
      eefm_ee_rot_error_p_gain(0),
      eefm_ee_error_cutoff_freq(0),
      eefm_support_polygon_vertices_sequence_length(0), eefm_support_polygon_vertices_sequence(NULL),
      eefm_use_force_difference_control(0),
      st_algorithm(0),
      controller_mode(0),
      transition_time(0),
      is_ik_enable_length(0), is_ik_enable(NULL),
      is_feedback_control_enable_length(0), is_feedback_control_enable(NULL),
      is_zmp_calc_enable_length(0), is_zmp_calc_enable(NULL),
      cop_check_margin(0),
      cp_check_margin(0),
      ref_capture_point(),
      act_capture_point(),
      contact_decision_threshold(0),
      foot_origin_offset(),
      emergency_check_mode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint8_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->k_tpcc_p[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->k_tpcc_x[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->k_brot_p[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->k_brot_tc[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_k1[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_k2[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_k3[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_zmp_delay_time_const[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_ref_zmp_aux[i]);
      }
      offset += this->eefm_rot_damping_gain.serialize(outbuffer + offset);
      offset += this->eefm_rot_time_const.serialize(outbuffer + offset);
      offset += this->eefm_pos_damping_gain.serialize(outbuffer + offset);
      offset += this->eefm_pos_time_const_support.serialize(outbuffer + offset);
      *(outbuffer + offset++) = eefm_pos_compensation_limit_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < eefm_pos_compensation_limit_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_pos_compensation_limit[i]);
      }
      *(outbuffer + offset++) = eefm_rot_compensation_limit_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < eefm_rot_compensation_limit_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_rot_compensation_limit[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_pos_time_const_swing);
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_pos_transition_time);
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_pos_margin_time);
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_leg_inside_margin);
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_leg_outside_margin);
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_leg_front_margin);
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_leg_rear_margin);
      for( uint8_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_body_attitude_control_gain[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_body_attitude_control_time_const[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_cogvel_cutoff_freq);
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_wrench_alpha_blending);
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_alpha_cutoff_freq);
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_gravitational_acceleration);
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_ee_pos_error_p_gain);
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_ee_rot_error_p_gain);
      offset += serializeAvrFloat64(outbuffer + offset, this->eefm_ee_error_cutoff_freq);
      *(outbuffer + offset++) = eefm_support_polygon_vertices_sequence_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < eefm_support_polygon_vertices_sequence_length; i++){
      offset += this->eefm_support_polygon_vertices_sequence[i].serialize(outbuffer + offset);
      }
      union {
        bool real;
        uint8_t base;
      } u_eefm_use_force_difference_control;
      u_eefm_use_force_difference_control.real = this->eefm_use_force_difference_control;
      *(outbuffer + offset + 0) = (u_eefm_use_force_difference_control.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->eefm_use_force_difference_control);
      union {
        int64_t real;
        uint64_t base;
      } u_st_algorithm;
      u_st_algorithm.real = this->st_algorithm;
      *(outbuffer + offset + 0) = (u_st_algorithm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_st_algorithm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_st_algorithm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_st_algorithm.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_st_algorithm.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_st_algorithm.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_st_algorithm.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_st_algorithm.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->st_algorithm);
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
      offset += serializeAvrFloat64(outbuffer + offset, this->transition_time);
      *(outbuffer + offset++) = is_ik_enable_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < is_ik_enable_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_is_ik_enablei;
      u_is_ik_enablei.real = this->is_ik_enable[i];
      *(outbuffer + offset + 0) = (u_is_ik_enablei.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_ik_enable[i]);
      }
      *(outbuffer + offset++) = is_feedback_control_enable_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < is_feedback_control_enable_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_is_feedback_control_enablei;
      u_is_feedback_control_enablei.real = this->is_feedback_control_enable[i];
      *(outbuffer + offset + 0) = (u_is_feedback_control_enablei.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_feedback_control_enable[i]);
      }
      *(outbuffer + offset++) = is_zmp_calc_enable_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < is_zmp_calc_enable_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_is_zmp_calc_enablei;
      u_is_zmp_calc_enablei.real = this->is_zmp_calc_enable[i];
      *(outbuffer + offset + 0) = (u_is_zmp_calc_enablei.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_zmp_calc_enable[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->cop_check_margin);
      offset += serializeAvrFloat64(outbuffer + offset, this->cp_check_margin);
      for( uint8_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->ref_capture_point[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->act_capture_point[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->contact_decision_threshold);
      offset += this->foot_origin_offset.serialize(outbuffer + offset);
      union {
        int64_t real;
        uint64_t base;
      } u_emergency_check_mode;
      u_emergency_check_mode.real = this->emergency_check_mode;
      *(outbuffer + offset + 0) = (u_emergency_check_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_emergency_check_mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_emergency_check_mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_emergency_check_mode.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_emergency_check_mode.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_emergency_check_mode.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_emergency_check_mode.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_emergency_check_mode.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->emergency_check_mode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint8_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->k_tpcc_p[i]));
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->k_tpcc_x[i]));
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->k_brot_p[i]));
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->k_brot_tc[i]));
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_k1[i]));
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_k2[i]));
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_k3[i]));
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_zmp_delay_time_const[i]));
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_ref_zmp_aux[i]));
      }
      offset += this->eefm_rot_damping_gain.deserialize(inbuffer + offset);
      offset += this->eefm_rot_time_const.deserialize(inbuffer + offset);
      offset += this->eefm_pos_damping_gain.deserialize(inbuffer + offset);
      offset += this->eefm_pos_time_const_support.deserialize(inbuffer + offset);
      uint8_t eefm_pos_compensation_limit_lengthT = *(inbuffer + offset++);
      if(eefm_pos_compensation_limit_lengthT > eefm_pos_compensation_limit_length)
        this->eefm_pos_compensation_limit = (float*)realloc(this->eefm_pos_compensation_limit, eefm_pos_compensation_limit_lengthT * sizeof(float));
      offset += 3;
      eefm_pos_compensation_limit_length = eefm_pos_compensation_limit_lengthT;
      for( uint8_t i = 0; i < eefm_pos_compensation_limit_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_eefm_pos_compensation_limit));
        memcpy( &(this->eefm_pos_compensation_limit[i]), &(this->st_eefm_pos_compensation_limit), sizeof(float));
      }
      uint8_t eefm_rot_compensation_limit_lengthT = *(inbuffer + offset++);
      if(eefm_rot_compensation_limit_lengthT > eefm_rot_compensation_limit_length)
        this->eefm_rot_compensation_limit = (float*)realloc(this->eefm_rot_compensation_limit, eefm_rot_compensation_limit_lengthT * sizeof(float));
      offset += 3;
      eefm_rot_compensation_limit_length = eefm_rot_compensation_limit_lengthT;
      for( uint8_t i = 0; i < eefm_rot_compensation_limit_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_eefm_rot_compensation_limit));
        memcpy( &(this->eefm_rot_compensation_limit[i]), &(this->st_eefm_rot_compensation_limit), sizeof(float));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_pos_time_const_swing));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_pos_transition_time));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_pos_margin_time));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_leg_inside_margin));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_leg_outside_margin));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_leg_front_margin));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_leg_rear_margin));
      for( uint8_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_body_attitude_control_gain[i]));
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_body_attitude_control_time_const[i]));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_cogvel_cutoff_freq));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_wrench_alpha_blending));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_alpha_cutoff_freq));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_gravitational_acceleration));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_ee_pos_error_p_gain));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_ee_rot_error_p_gain));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->eefm_ee_error_cutoff_freq));
      uint8_t eefm_support_polygon_vertices_sequence_lengthT = *(inbuffer + offset++);
      if(eefm_support_polygon_vertices_sequence_lengthT > eefm_support_polygon_vertices_sequence_length)
        this->eefm_support_polygon_vertices_sequence = (hrpsys_ros_bridge::OpenHRP_StabilizerService_SupportPolygonVertices*)realloc(this->eefm_support_polygon_vertices_sequence, eefm_support_polygon_vertices_sequence_lengthT * sizeof(hrpsys_ros_bridge::OpenHRP_StabilizerService_SupportPolygonVertices));
      offset += 3;
      eefm_support_polygon_vertices_sequence_length = eefm_support_polygon_vertices_sequence_lengthT;
      for( uint8_t i = 0; i < eefm_support_polygon_vertices_sequence_length; i++){
      offset += this->st_eefm_support_polygon_vertices_sequence.deserialize(inbuffer + offset);
        memcpy( &(this->eefm_support_polygon_vertices_sequence[i]), &(this->st_eefm_support_polygon_vertices_sequence), sizeof(hrpsys_ros_bridge::OpenHRP_StabilizerService_SupportPolygonVertices));
      }
      union {
        bool real;
        uint8_t base;
      } u_eefm_use_force_difference_control;
      u_eefm_use_force_difference_control.base = 0;
      u_eefm_use_force_difference_control.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->eefm_use_force_difference_control = u_eefm_use_force_difference_control.real;
      offset += sizeof(this->eefm_use_force_difference_control);
      union {
        int64_t real;
        uint64_t base;
      } u_st_algorithm;
      u_st_algorithm.base = 0;
      u_st_algorithm.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_algorithm.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_algorithm.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_algorithm.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_algorithm.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_algorithm.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_algorithm.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_algorithm.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_algorithm = u_st_algorithm.real;
      offset += sizeof(this->st_algorithm);
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
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->transition_time));
      uint8_t is_ik_enable_lengthT = *(inbuffer + offset++);
      if(is_ik_enable_lengthT > is_ik_enable_length)
        this->is_ik_enable = (bool*)realloc(this->is_ik_enable, is_ik_enable_lengthT * sizeof(bool));
      offset += 3;
      is_ik_enable_length = is_ik_enable_lengthT;
      for( uint8_t i = 0; i < is_ik_enable_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_is_ik_enable;
      u_st_is_ik_enable.base = 0;
      u_st_is_ik_enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_is_ik_enable = u_st_is_ik_enable.real;
      offset += sizeof(this->st_is_ik_enable);
        memcpy( &(this->is_ik_enable[i]), &(this->st_is_ik_enable), sizeof(bool));
      }
      uint8_t is_feedback_control_enable_lengthT = *(inbuffer + offset++);
      if(is_feedback_control_enable_lengthT > is_feedback_control_enable_length)
        this->is_feedback_control_enable = (bool*)realloc(this->is_feedback_control_enable, is_feedback_control_enable_lengthT * sizeof(bool));
      offset += 3;
      is_feedback_control_enable_length = is_feedback_control_enable_lengthT;
      for( uint8_t i = 0; i < is_feedback_control_enable_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_is_feedback_control_enable;
      u_st_is_feedback_control_enable.base = 0;
      u_st_is_feedback_control_enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_is_feedback_control_enable = u_st_is_feedback_control_enable.real;
      offset += sizeof(this->st_is_feedback_control_enable);
        memcpy( &(this->is_feedback_control_enable[i]), &(this->st_is_feedback_control_enable), sizeof(bool));
      }
      uint8_t is_zmp_calc_enable_lengthT = *(inbuffer + offset++);
      if(is_zmp_calc_enable_lengthT > is_zmp_calc_enable_length)
        this->is_zmp_calc_enable = (bool*)realloc(this->is_zmp_calc_enable, is_zmp_calc_enable_lengthT * sizeof(bool));
      offset += 3;
      is_zmp_calc_enable_length = is_zmp_calc_enable_lengthT;
      for( uint8_t i = 0; i < is_zmp_calc_enable_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_is_zmp_calc_enable;
      u_st_is_zmp_calc_enable.base = 0;
      u_st_is_zmp_calc_enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_is_zmp_calc_enable = u_st_is_zmp_calc_enable.real;
      offset += sizeof(this->st_is_zmp_calc_enable);
        memcpy( &(this->is_zmp_calc_enable[i]), &(this->st_is_zmp_calc_enable), sizeof(bool));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->cop_check_margin));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->cp_check_margin));
      for( uint8_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ref_capture_point[i]));
      }
      for( uint8_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->act_capture_point[i]));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->contact_decision_threshold));
      offset += this->foot_origin_offset.deserialize(inbuffer + offset);
      union {
        int64_t real;
        uint64_t base;
      } u_emergency_check_mode;
      u_emergency_check_mode.base = 0;
      u_emergency_check_mode.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_emergency_check_mode.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_emergency_check_mode.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_emergency_check_mode.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_emergency_check_mode.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_emergency_check_mode.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_emergency_check_mode.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_emergency_check_mode.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->emergency_check_mode = u_emergency_check_mode.real;
      offset += sizeof(this->emergency_check_mode);
     return offset;
    }

    const char * getType(){ return "hrpsys_ros_bridge/OpenHRP_StabilizerService_stParam"; };
    const char * getMD5(){ return "4accb0735c2c8f78338df6dae3b48a98"; };

  };

}
#endif