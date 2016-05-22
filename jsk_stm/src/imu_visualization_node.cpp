#include "jsk_stm/imu_visualization.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "imu_visualization");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ImuVisualization*  imuVisualizationNode
      = new ImuVisualization(nh, nh_private);

  ros::spin ();
  delete imuVisualizationNode;
  return 0;
}
