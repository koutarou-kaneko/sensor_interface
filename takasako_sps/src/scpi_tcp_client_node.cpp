#include <takasako_sps/scpi_tcp_client.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "takasago_sps");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  TakasokoSps* TakasokoSpsNode = new TakasokoSps(nh, nh_private);

  ros::spin();
  delete TakasokoSpsNode;
  return 0;
}
