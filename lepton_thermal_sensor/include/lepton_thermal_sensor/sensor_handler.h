#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include "lepton_thermal_sensor/palettes.h"
#include "lepton_thermal_sensor/spi.h"
#include "lepton_thermal_sensor/i2c.h"

#include <iostream>

class SensorHandler
{

public:
  SensorHandler(std::string i2c_dev, std::string spi_dev, uint16_t packets_per_frame = 60, uint16_t packet_size = 164, uint8_t head_bytes = 4, uint16_t reboot_max_cnt = 750);
  ~SensorHandler();

  void connect();
  void disconnect();
  void reboot();

  bool grabImage(cv::Mat &temp_map, double& timestamp);

  const float getSensorTemperature();
  const cv::Size getImageSize() {return cv::Size((packet_size_- head_bytes_)/2, packets_per_frame_);}

private:
  std::string i2c_dev_;
  std::string spi_dev_;

  bool connected_;
  uint16_t packets_per_frame_; // equal to rows
  uint16_t packet_size_; // bytes per packet
  uint8_t head_bytes_;

  uint16_t wrong_packet_cnt_;
  uint16_t reboot_max_cnt_;

  uint8_t* raw_data_;
  cv::Mat temperature_map_;
};

