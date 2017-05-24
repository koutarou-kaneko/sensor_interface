#include <leddar_one/leddar_one_driver.h>

LeddarOne::LeddarOne(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp)
{

  nhp_.param("port", port_, std::string("/dev/ttyUSB0"));
  nhp_.param("baud", baud_, 115200);
  nhp_.param("module_id", module_id_, 1);
  nhp_.param("loop_rate", loop_rate_, 100.0);
  nhp_.param("min_range", min_range_, 0.0); //0[m]
  nhp_.param("max_range", max_range_, 40.0); //40[m]

  range_pub_ = nh_.advertise<sensor_msgs::Range>("/distance", 10);

  configure_ = false;
  start_connection_ = true;

  mb_ = modbus_new_rtu(port_.c_str(), baud_, 'N', 8, 1);
  if (mb_ == NULL)
    {
      ROS_FATAL("Unable to create the libmodbus context\n");
      start_connection_ = false;
    }

  //modbus_set_debug(mb_, true);      // uncomment to view debug info

  if (modbus_connect(mb_) != 0)
    {
      modbus_free(mb_);
      ROS_FATAL("Connection error\n");
      start_connection_ = false;
    }


  if (modbus_set_slave(mb_, module_id_) != 0)
    {
      modbus_close(mb_);
      modbus_free(mb_);
      ROS_FATAL("Error setting module id\n");
      start_connection_ = false;
    }


  if(start_connection_)
    {
      /* get main param first */
      getParameter();

      /* then update dnyamic reconfigure */
      config_server_ = new dynamic_reconfigure::Server<leddar_one::LeddarOneConfig>(nhp_);
      config_func_ = boost::bind(&LeddarOne::configCallback, this, _1, _2);
      config_server_->setCallback(config_func_);

      main_thread_ = boost::thread(boost::bind(&LeddarOne::communicationFunc, this));
    }
}

LeddarOne::~LeddarOne()
{
  if(start_connection_) 
    {
      if (mb_)
        {
          modbus_close(mb_);
          modbus_free(mb_);
        }

      main_thread_.interrupt();
      main_thread_.join();

    }
}

void LeddarOne::communicationFunc()
{
  double du = 1 / loop_rate_ * 1000; // to millisec

  uint16_t tab_reg_values[32];
  while(ros::ok())
    {
      boost::this_thread::sleep(boost::posix_time::milliseconds(du));

      // write processing
      if(configure_)
        {
          configure_ = false;
          //write all regsiters
          modbus_write_register(mb_, LEDDARONE_CONFIG_ACCUMULATION, accumulation_);
          modbus_write_register(mb_, LEDDARONE_CONFIG_OVERSAMPLING, oversample_);
          modbus_write_register(mb_, LEDDARONE_CONFIG_SAMPLE_COUNT, points_);
        }

      // read processing 
      int num_read = modbus_read_input_registers(mb_, READ_DETECTION_ADRRESS, READ_DETECTION_LENGTH, tab_reg_values);
      if (num_read == READ_DETECTION_LENGTH)
        {
          uint32_t u_time_stamp = tab_reg_values[0] + (tab_reg_values[1] << 16);
          float f_temperature = (float)tab_reg_values[2] / 256.f;
          int u_detection_count = tab_reg_values[3] < LEDDAR_MAX_DETECTIONS ? tab_reg_values[3] : LEDDAR_MAX_DETECTIONS;

          double distance[3];
          double amplitude[3];

          for (int i = 0; i < u_detection_count; ++i)
            {
              distance[i] = (double)tab_reg_values[i * 2 + 4] / 1000.0;
              amplitude[i] = (double)tab_reg_values[i * 2 + 5] / 256.0;

              distance_ += distance[i];
              amplitude_ += amplitude[i];
            }

          distance_ /= u_detection_count;
          amplitude_ /= u_detection_count;

          sensor_msgs::Range range_msg;
          range_msg.header.stamp = ros::Time::now();  // TODO: use time offset
          range_msg.radiation_type = sensor_msgs::Range::INFRARED;
          range_msg.min_range = min_range_;
          range_msg.max_range = max_range_;
          range_msg.range = distance_;
          range_pub_.publish(range_msg);

          //debug
          //ROS_INFO("time: %f, distance: %f, amplitude: %f", ros::Time::now().toSec(), distance_, amplitude_);
          distance_ = 0;
          amplitude_ = 0;
        }
      else
        {
          ROS_ERROR("can not read date from sensor");
        }
    }
}

void LeddarOne::configCallback(leddar_one::LeddarOneConfig &config, uint32_t level)
{
  if(config.configure_flag)
    {
      configure_ = true;
      accumulation_ = config.accumulation;
      oversample_ = config.oversample;
      points_ = config.points;
      ROS_INFO("change config to: accumulation:%d, oversample:%d, pints:%d", accumulation_, oversample_, points_);
    }
}

void LeddarOne::getParameter()
{
  //Accmulation
  uint16_t reg_value;
  if(modbus_read_input_registers(mb_, LEDDARONE_CONFIG_ACCUMULATION, 1, &reg_value) == 1);
  nhp_.setParam("accumulation", accumulation_);

  //Over Sample
  if(modbus_read_input_registers(mb_, LEDDARONE_CONFIG_OVERSAMPLING, 1, &reg_value) == 1);
  nhp_.setParam("oversample", oversample_);

  //Over Sample
  if(modbus_read_input_registers(mb_, LEDDARONE_CONFIG_SAMPLE_COUNT, 1, &reg_value) == 1);
  nhp_.setParam("points", points_);

}
