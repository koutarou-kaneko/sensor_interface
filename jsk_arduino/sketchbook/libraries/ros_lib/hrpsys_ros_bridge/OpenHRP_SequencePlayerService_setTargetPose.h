#ifndef _ROS_SERVICE_OpenHRP_SequencePlayerService_setTargetPose_h
#define _ROS_SERVICE_OpenHRP_SequencePlayerService_setTargetPose_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

static const char OPENHRP_SEQUENCEPLAYERSERVICE_SETTARGETPOSE[] = "hrpsys_ros_bridge/OpenHRP_SequencePlayerService_setTargetPose";

  class OpenHRP_SequencePlayerService_setTargetPoseRequest : public ros::Msg
  {
    public:
      const char* name;
      uint8_t xyz_length;
      float st_xyz;
      float * xyz;
      uint8_t rpy_length;
      float st_rpy;
      float * rpy;
      float tm;

    OpenHRP_SequencePlayerService_setTargetPoseRequest():
      name(""),
      xyz_length(0), xyz(NULL),
      rpy_length(0), rpy(NULL),
      tm(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      *(outbuffer + offset++) = xyz_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < xyz_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->xyz[i]);
      }
      *(outbuffer + offset++) = rpy_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < rpy_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->rpy[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->tm);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint8_t xyz_lengthT = *(inbuffer + offset++);
      if(xyz_lengthT > xyz_length)
        this->xyz = (float*)realloc(this->xyz, xyz_lengthT * sizeof(float));
      offset += 3;
      xyz_length = xyz_lengthT;
      for( uint8_t i = 0; i < xyz_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_xyz));
        memcpy( &(this->xyz[i]), &(this->st_xyz), sizeof(float));
      }
      uint8_t rpy_lengthT = *(inbuffer + offset++);
      if(rpy_lengthT > rpy_length)
        this->rpy = (float*)realloc(this->rpy, rpy_lengthT * sizeof(float));
      offset += 3;
      rpy_length = rpy_lengthT;
      for( uint8_t i = 0; i < rpy_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_rpy));
        memcpy( &(this->rpy[i]), &(this->st_rpy), sizeof(float));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tm));
     return offset;
    }

    const char * getType(){ return OPENHRP_SEQUENCEPLAYERSERVICE_SETTARGETPOSE; };
    const char * getMD5(){ return "fbd6f07e5a5bde9514a1389b662bf2e1"; };

  };

  class OpenHRP_SequencePlayerService_setTargetPoseResponse : public ros::Msg
  {
    public:
      bool operation_return;

    OpenHRP_SequencePlayerService_setTargetPoseResponse():
      operation_return(0)
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
     return offset;
    }

    const char * getType(){ return OPENHRP_SEQUENCEPLAYERSERVICE_SETTARGETPOSE; };
    const char * getMD5(){ return "8dd59ee39c15084c92106411b8c3e8fc"; };

  };

  class OpenHRP_SequencePlayerService_setTargetPose {
    public:
    typedef OpenHRP_SequencePlayerService_setTargetPoseRequest Request;
    typedef OpenHRP_SequencePlayerService_setTargetPoseResponse Response;
  };

}
#endif
