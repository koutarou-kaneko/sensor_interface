#ifndef _ROS_bayesian_belief_networks_Result_h
#define _ROS_bayesian_belief_networks_Result_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bayesian_belief_networks
{

  class Result : public ros::Msg
  {
    public:
      const char* node;
      const char* Value;
      float Marginal;

    Result():
      node(""),
      Value(""),
      Marginal(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_node = strlen(this->node);
      memcpy(outbuffer + offset, &length_node, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->node, length_node);
      offset += length_node;
      uint32_t length_Value = strlen(this->Value);
      memcpy(outbuffer + offset, &length_Value, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->Value, length_Value);
      offset += length_Value;
      offset += serializeAvrFloat64(outbuffer + offset, this->Marginal);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_node;
      memcpy(&length_node, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_node; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_node-1]=0;
      this->node = (char *)(inbuffer + offset-1);
      offset += length_node;
      uint32_t length_Value;
      memcpy(&length_Value, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_Value; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_Value-1]=0;
      this->Value = (char *)(inbuffer + offset-1);
      offset += length_Value;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Marginal));
     return offset;
    }

    const char * getType(){ return "bayesian_belief_networks/Result"; };
    const char * getMD5(){ return "c12bc86306f498c9d6b12dcfb4492003"; };

  };

}
#endif