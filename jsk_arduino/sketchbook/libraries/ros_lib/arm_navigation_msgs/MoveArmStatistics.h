#ifndef _ROS_arm_navigation_msgs_MoveArmStatistics_h
#define _ROS_arm_navigation_msgs_MoveArmStatistics_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "arm_navigation_msgs/ArmNavigationErrorCodes.h"

namespace arm_navigation_msgs
{

  class MoveArmStatistics : public ros::Msg
  {
    public:
      int32_t request_id;
      const char* result;
      arm_navigation_msgs::ArmNavigationErrorCodes error_code;
      float planning_time;
      float smoothing_time;
      float ik_time;
      float time_to_execution;
      float time_to_result;
      bool preempted;
      float num_replans;
      float trajectory_duration;
      const char* planner_service_name;

    MoveArmStatistics():
      request_id(0),
      result(""),
      error_code(),
      planning_time(0),
      smoothing_time(0),
      ik_time(0),
      time_to_execution(0),
      time_to_result(0),
      preempted(0),
      num_replans(0),
      trajectory_duration(0),
      planner_service_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_request_id;
      u_request_id.real = this->request_id;
      *(outbuffer + offset + 0) = (u_request_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_request_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_request_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_request_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->request_id);
      uint32_t length_result = strlen(this->result);
      memcpy(outbuffer + offset, &length_result, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->result, length_result);
      offset += length_result;
      offset += this->error_code.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->planning_time);
      offset += serializeAvrFloat64(outbuffer + offset, this->smoothing_time);
      offset += serializeAvrFloat64(outbuffer + offset, this->ik_time);
      offset += serializeAvrFloat64(outbuffer + offset, this->time_to_execution);
      offset += serializeAvrFloat64(outbuffer + offset, this->time_to_result);
      union {
        bool real;
        uint8_t base;
      } u_preempted;
      u_preempted.real = this->preempted;
      *(outbuffer + offset + 0) = (u_preempted.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->preempted);
      offset += serializeAvrFloat64(outbuffer + offset, this->num_replans);
      offset += serializeAvrFloat64(outbuffer + offset, this->trajectory_duration);
      uint32_t length_planner_service_name = strlen(this->planner_service_name);
      memcpy(outbuffer + offset, &length_planner_service_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->planner_service_name, length_planner_service_name);
      offset += length_planner_service_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_request_id;
      u_request_id.base = 0;
      u_request_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_request_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_request_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_request_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->request_id = u_request_id.real;
      offset += sizeof(this->request_id);
      uint32_t length_result;
      memcpy(&length_result, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_result; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_result-1]=0;
      this->result = (char *)(inbuffer + offset-1);
      offset += length_result;
      offset += this->error_code.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->planning_time));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->smoothing_time));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ik_time));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time_to_execution));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time_to_result));
      union {
        bool real;
        uint8_t base;
      } u_preempted;
      u_preempted.base = 0;
      u_preempted.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->preempted = u_preempted.real;
      offset += sizeof(this->preempted);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->num_replans));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->trajectory_duration));
      uint32_t length_planner_service_name;
      memcpy(&length_planner_service_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_planner_service_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_planner_service_name-1]=0;
      this->planner_service_name = (char *)(inbuffer + offset-1);
      offset += length_planner_service_name;
     return offset;
    }

    const char * getType(){ return "arm_navigation_msgs/MoveArmStatistics"; };
    const char * getMD5(){ return "d83dee1348791a0d1414257b41bc161f"; };

  };

}
#endif