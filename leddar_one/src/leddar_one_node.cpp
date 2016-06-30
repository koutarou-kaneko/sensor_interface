#include <leddar_one/leddar_one_driver.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "leddar_one");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  boost::shared_ptr<LeddarOne> leddarONeNode(new LeddarOne(nh, nh_private));
  ros::spin ();
  return 0;
}

