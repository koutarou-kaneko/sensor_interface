#ifndef LEDDAR_ONE_DRIVER_H
#define LEDDAR_ONE_DRIVER_H

#include <ros/ros.h>
#include <string.h>
#include <boost/thread.hpp>
#include <modbus/modbus.h>
#include <sensor_msgs/Range.h>

#include <dynamic_reconfigure/server.h>
#include <leddar_one/LeddarOneConfig.h>

#define LEDDAR_MAX_DETECTIONS 3

#ifndef MAX
#define MAX(x,y) (((x) > (y)) ? (x) : (y))
#endif


class LeddarOne
{
public:
  LeddarOne(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~LeddarOne();

  static const uint8_t READ_DETECTION_LENGTH = 10;
  static const uint8_t READ_DETECTION_ADRRESS = 20;
  static const uint8_t LEDDARONE_CONFIG_ACCUMULATION = 0;
  static const uint8_t LEDDARONE_CONFIG_OVERSAMPLING = 1;
  static const uint8_t LEDDARONE_CONFIG_SAMPLE_COUNT = 2;
  static const uint8_t LEDDARONE_CONFIG_LED_POWER    = 4;
  static const uint8_t LEDDARONE_CONFIG_ACQ_OPTIONS  = 6;                    // Bit 0 = Auto LED intensity
  static const uint8_t LEDDARONE_CONFIG_BAUD_RATE    = 29;
  static const uint8_t LEDDARONE_CONFIG_MODBUS_ADDRESS  = 30;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher range_pub_;
  dynamic_reconfigure::Server<leddar_one::LeddarOneConfig>* config_server_;
  dynamic_reconfigure::Server<leddar_one::LeddarOneConfig>::CallbackType config_func_;

  std::string port_;
  int baud_;
  int module_id_;
  double loop_rate_;

  double distance_;  // distance from the sensor, in meters
  double amplitude_; // signal amplitude
  double time_offset_;
  double max_range_; //the min value of this sensor
  double min_range_; //the max value of this sensor
  bool start_connection_;
  bool configure_;

  int accumulation_;
  int oversample_;
  int points_;

  modbus_t* mb_;

  boost::thread main_thread_;

  void communicationFunc();
  void getParameter();
  void configCallback(leddar_one::LeddarOneConfig &config, uint32_t level);
};



#endif // LEDDAR_ONE_DRIVER_H
