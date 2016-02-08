#ifndef _ROS_hrpsys_gazebo_atlas_JointStateCompressed_h
#define _ROS_hrpsys_gazebo_atlas_JointStateCompressed_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hrpsys_gazebo_atlas
{

  class JointStateCompressed : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t index_length;
      int8_t st_index;
      int8_t * index;
      uint8_t position_length;
      float st_position;
      float * position;

    JointStateCompressed():
      header(),
      index_length(0), index(NULL),
      position_length(0), position(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = index_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < index_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_indexi;
      u_indexi.real = this->index[i];
      *(outbuffer + offset + 0) = (u_indexi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->index[i]);
      }
      *(outbuffer + offset++) = position_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < position_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->position[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t index_lengthT = *(inbuffer + offset++);
      if(index_lengthT > index_length)
        this->index = (int8_t*)realloc(this->index, index_lengthT * sizeof(int8_t));
      offset += 3;
      index_length = index_lengthT;
      for( uint8_t i = 0; i < index_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_st_index;
      u_st_index.base = 0;
      u_st_index.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_index = u_st_index.real;
      offset += sizeof(this->st_index);
        memcpy( &(this->index[i]), &(this->st_index), sizeof(int8_t));
      }
      uint8_t position_lengthT = *(inbuffer + offset++);
      if(position_lengthT > position_length)
        this->position = (float*)realloc(this->position, position_lengthT * sizeof(float));
      offset += 3;
      position_length = position_lengthT;
      for( uint8_t i = 0; i < position_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_position));
        memcpy( &(this->position[i]), &(this->st_position), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "hrpsys_gazebo_atlas/JointStateCompressed"; };
    const char * getMD5(){ return "40b573e5ce6068f22b324b721732f7de"; };

  };

}
#endif