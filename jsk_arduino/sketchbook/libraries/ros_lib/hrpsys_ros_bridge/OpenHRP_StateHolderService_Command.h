#ifndef _ROS_hrpsys_ros_bridge_OpenHRP_StateHolderService_Command_h
#define _ROS_hrpsys_ros_bridge_OpenHRP_StateHolderService_Command_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hrpsys_ros_bridge
{

  class OpenHRP_StateHolderService_Command : public ros::Msg
  {
    public:
      uint8_t jointRefs_length;
      float st_jointRefs;
      float * jointRefs;
      uint8_t baseTransform_length;
      float st_baseTransform;
      float * baseTransform;
      uint8_t zmp_length;
      float st_zmp;
      float * zmp;

    OpenHRP_StateHolderService_Command():
      jointRefs_length(0), jointRefs(NULL),
      baseTransform_length(0), baseTransform(NULL),
      zmp_length(0), zmp(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = jointRefs_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < jointRefs_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->jointRefs[i]);
      }
      *(outbuffer + offset++) = baseTransform_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < baseTransform_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->baseTransform[i]);
      }
      *(outbuffer + offset++) = zmp_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < zmp_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->zmp[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t jointRefs_lengthT = *(inbuffer + offset++);
      if(jointRefs_lengthT > jointRefs_length)
        this->jointRefs = (float*)realloc(this->jointRefs, jointRefs_lengthT * sizeof(float));
      offset += 3;
      jointRefs_length = jointRefs_lengthT;
      for( uint8_t i = 0; i < jointRefs_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_jointRefs));
        memcpy( &(this->jointRefs[i]), &(this->st_jointRefs), sizeof(float));
      }
      uint8_t baseTransform_lengthT = *(inbuffer + offset++);
      if(baseTransform_lengthT > baseTransform_length)
        this->baseTransform = (float*)realloc(this->baseTransform, baseTransform_lengthT * sizeof(float));
      offset += 3;
      baseTransform_length = baseTransform_lengthT;
      for( uint8_t i = 0; i < baseTransform_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_baseTransform));
        memcpy( &(this->baseTransform[i]), &(this->st_baseTransform), sizeof(float));
      }
      uint8_t zmp_lengthT = *(inbuffer + offset++);
      if(zmp_lengthT > zmp_length)
        this->zmp = (float*)realloc(this->zmp, zmp_lengthT * sizeof(float));
      offset += 3;
      zmp_length = zmp_lengthT;
      for( uint8_t i = 0; i < zmp_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_zmp));
        memcpy( &(this->zmp[i]), &(this->st_zmp), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "hrpsys_ros_bridge/OpenHRP_StateHolderService_Command"; };
    const char * getMD5(){ return "c456a517c9c8704b756ab8fb10b08d6d"; };

  };

}
#endif