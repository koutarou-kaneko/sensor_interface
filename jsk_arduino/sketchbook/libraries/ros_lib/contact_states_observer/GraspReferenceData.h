#ifndef _ROS_contact_states_observer_GraspReferenceData_h
#define _ROS_contact_states_observer_GraspReferenceData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace contact_states_observer
{

  class GraspReferenceData : public ros::Msg
  {
    public:
      const char* name;
      uint8_t av_length;
      float st_av;
      float * av;

    GraspReferenceData():
      name(""),
      av_length(0), av(NULL)
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
      *(outbuffer + offset++) = av_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < av_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->av[i]);
      }
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
      uint8_t av_lengthT = *(inbuffer + offset++);
      if(av_lengthT > av_length)
        this->av = (float*)realloc(this->av, av_lengthT * sizeof(float));
      offset += 3;
      av_length = av_lengthT;
      for( uint8_t i = 0; i < av_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_av));
        memcpy( &(this->av[i]), &(this->st_av), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "contact_states_observer/GraspReferenceData"; };
    const char * getMD5(){ return "42ebda9900cd534e5ac93b2d87081425"; };

  };

}
#endif