#ifndef _ROS_hrpsys_ros_bridge_OpenHRP_RobotHardwareService_RobotState2_h
#define _ROS_hrpsys_ros_bridge_OpenHRP_RobotHardwareService_RobotState2_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "hrpsys_ros_bridge/OpenHRP_RobotHardwareService_BatteryState.h"

namespace hrpsys_ros_bridge
{

  class OpenHRP_RobotHardwareService_RobotState2 : public ros::Msg
  {
    public:
      uint8_t angle_length;
      float st_angle;
      float * angle;
      uint8_t command_length;
      float st_command;
      float * command;
      uint8_t torque_length;
      float st_torque;
      float * torque;
      std_msgs::Int32MultiArray servoState;
      std_msgs::Float64MultiArray force;
      std_msgs::Float64MultiArray rateGyro;
      std_msgs::Float64MultiArray accel;
      uint8_t batteries_length;
      hrpsys_ros_bridge::OpenHRP_RobotHardwareService_BatteryState st_batteries;
      hrpsys_ros_bridge::OpenHRP_RobotHardwareService_BatteryState * batteries;
      float voltage;
      float current;
      uint8_t temperature_length;
      float st_temperature;
      float * temperature;

    OpenHRP_RobotHardwareService_RobotState2():
      angle_length(0), angle(NULL),
      command_length(0), command(NULL),
      torque_length(0), torque(NULL),
      servoState(),
      force(),
      rateGyro(),
      accel(),
      batteries_length(0), batteries(NULL),
      voltage(0),
      current(0),
      temperature_length(0), temperature(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = angle_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < angle_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->angle[i]);
      }
      *(outbuffer + offset++) = command_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < command_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->command[i]);
      }
      *(outbuffer + offset++) = torque_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < torque_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->torque[i]);
      }
      offset += this->servoState.serialize(outbuffer + offset);
      offset += this->force.serialize(outbuffer + offset);
      offset += this->rateGyro.serialize(outbuffer + offset);
      offset += this->accel.serialize(outbuffer + offset);
      *(outbuffer + offset++) = batteries_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < batteries_length; i++){
      offset += this->batteries[i].serialize(outbuffer + offset);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->voltage);
      offset += serializeAvrFloat64(outbuffer + offset, this->current);
      *(outbuffer + offset++) = temperature_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < temperature_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->temperature[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t angle_lengthT = *(inbuffer + offset++);
      if(angle_lengthT > angle_length)
        this->angle = (float*)realloc(this->angle, angle_lengthT * sizeof(float));
      offset += 3;
      angle_length = angle_lengthT;
      for( uint8_t i = 0; i < angle_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_angle));
        memcpy( &(this->angle[i]), &(this->st_angle), sizeof(float));
      }
      uint8_t command_lengthT = *(inbuffer + offset++);
      if(command_lengthT > command_length)
        this->command = (float*)realloc(this->command, command_lengthT * sizeof(float));
      offset += 3;
      command_length = command_lengthT;
      for( uint8_t i = 0; i < command_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_command));
        memcpy( &(this->command[i]), &(this->st_command), sizeof(float));
      }
      uint8_t torque_lengthT = *(inbuffer + offset++);
      if(torque_lengthT > torque_length)
        this->torque = (float*)realloc(this->torque, torque_lengthT * sizeof(float));
      offset += 3;
      torque_length = torque_lengthT;
      for( uint8_t i = 0; i < torque_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_torque));
        memcpy( &(this->torque[i]), &(this->st_torque), sizeof(float));
      }
      offset += this->servoState.deserialize(inbuffer + offset);
      offset += this->force.deserialize(inbuffer + offset);
      offset += this->rateGyro.deserialize(inbuffer + offset);
      offset += this->accel.deserialize(inbuffer + offset);
      uint8_t batteries_lengthT = *(inbuffer + offset++);
      if(batteries_lengthT > batteries_length)
        this->batteries = (hrpsys_ros_bridge::OpenHRP_RobotHardwareService_BatteryState*)realloc(this->batteries, batteries_lengthT * sizeof(hrpsys_ros_bridge::OpenHRP_RobotHardwareService_BatteryState));
      offset += 3;
      batteries_length = batteries_lengthT;
      for( uint8_t i = 0; i < batteries_length; i++){
      offset += this->st_batteries.deserialize(inbuffer + offset);
        memcpy( &(this->batteries[i]), &(this->st_batteries), sizeof(hrpsys_ros_bridge::OpenHRP_RobotHardwareService_BatteryState));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->voltage));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->current));
      uint8_t temperature_lengthT = *(inbuffer + offset++);
      if(temperature_lengthT > temperature_length)
        this->temperature = (float*)realloc(this->temperature, temperature_lengthT * sizeof(float));
      offset += 3;
      temperature_length = temperature_lengthT;
      for( uint8_t i = 0; i < temperature_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_temperature));
        memcpy( &(this->temperature[i]), &(this->st_temperature), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "hrpsys_ros_bridge/OpenHRP_RobotHardwareService_RobotState2"; };
    const char * getMD5(){ return "55bedf35a5f953cd1f113a0e7665626f"; };

  };

}
#endif