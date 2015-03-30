#include "mocap_retime/mocap_retime.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "mocap_retime");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  MocapRetime*  retimeNode = new MocapRetime(nh, nh_private);
  ros::spin ();
  delete retimeNode;
  return 0;
}
