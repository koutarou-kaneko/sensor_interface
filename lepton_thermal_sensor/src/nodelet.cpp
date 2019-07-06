#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>

#include "lepton_thermal_sensor/sensor_handler.h"

#include <boost/thread.hpp> // Needed for the nodelet to launch the reading thread.
#include <fstream>

namespace lepton_camera_driver
{
  class LeptonThermalSensorNodelet: public nodelet::Nodelet
  {
  public:
    LeptonThermalSensorNodelet() {}

    ~LeptonThermalSensorNodelet()
    {
      if(pubThread_)
        {
          pubThread_->interrupt();
          pubThread_->join();

          try
            {
              NODELET_DEBUG("Disconnecting from camera.");
              sh_->disconnect();
            }
          catch(std::runtime_error& e)
            {
              NODELET_ERROR("%s", e.what());
            }
        }
    }

  private:
    void onInit()
    {
      // Get nodeHandles
      ros::NodeHandle &nh = getMTNodeHandle();
      ros::NodeHandle &pnh = getMTPrivateNodeHandle();

      // Get the sensor polling configuration and create the sensor handler
      std::string spi_dev;
      int i2c_dev;
      pnh.param("spi_dev", spi_dev, std::string("/dev/spidev0.1"));
      pnh.param("i2c_dev", i2c_dev, 0); // i.e., /dev/i2c_dev
      int packets_per_frame, packet_size, head_bytes, reboot_max_cnt;
      pnh.param("packets_per_frame", packets_per_frame, 60);
      pnh.param("packet_size", packet_size, 164);
      pnh.param("head_bytes", head_bytes, 4);
      pnh.param("reboot_max_cnt", reboot_max_cnt, 750);
      sh_.reset(new SensorHandler(i2c_dev, spi_dev, packets_per_frame, packet_size, head_bytes, reboot_max_cnt));

      // Get the location of our camera config yaml
      std::string camera_info_url;
      pnh.param("camera_info_url", camera_info_url, std::string(""));
      pnh.param("frame_id", frame_id_, std::string("camera"));

      // Start the camera info manager and attempt to load any configurations
      cinfo_.reset(new camera_info_manager::CameraInfoManager(nh, frame_id_, camera_info_url));
      ci_.reset(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
      ci_->header.frame_id = frame_id_;

      // Publish topics using ImageTransport through camera_info_manager (gives cool things like compression)
      it_.reset(new image_transport::ImageTransport(nh));
      image_transport::SubscriberStatusCallback cb = boost::bind(&lepton_camera_driver::LeptonThermalSensorNodelet::connectCb, this);
      it_pub_ = it_->advertiseCamera("image_raw", 5, cb, cb);
    }

    void connectCb()
    {
      NODELET_DEBUG("Connect callback!");
      // Check if we should disconnect (there are 0 subscribers to our data)
      if(it_pub_.getNumSubscribers() == 0)
        {
          if (pubThread_)
            {
              NODELET_DEBUG("Disconnecting.");
              pubThread_->interrupt();
              pubThread_->join();
              pubThread_.reset();

              sh_->disconnect();
            }
        }
      else if(!pubThread_)  // We need to connect
        {
          // Start the thread to loop through and publish messages
          pubThread_.reset(new boost::thread(boost::bind(&LeptonThermalSensorNodelet::devicePoll, this)));
        }
      else
        {
          NODELET_DEBUG("Do nothing in callback.");
        }
    }

    void devicePoll()
    {
      enum State
      {
        NONE
        , ERROR
        , DISCONNECTED
        , CONNECTED
      };

      State state = DISCONNECTED;
      State previous_state = NONE;

      cv::Mat temperature_map(sh_->getImageSize(), CV_16UC1);
      cv::Mat thermal_map(sh_->getImageSize(), CV_8UC3);

      while(!boost::this_thread::interruption_requested())   // Block until we need to stop this thread.
        {
          bool state_changed = state != previous_state;

          previous_state = state;

          switch(state)
            {
            case ERROR:
            case DISCONNECTED:
              // Try connecting to the camera
              try
                {
                  NODELET_DEBUG("Connecting to thermal sensor.");
                  sh_->connect();
                  NODELET_INFO("Connected to thermal sensor.");

                  state = CONNECTED;
                }
              catch(std::runtime_error& e)
                {
                  NODELET_ERROR_COND(state_changed,
                                     "Failed to connect with error: %s", e.what());
                  ros::Duration(1.0).sleep(); // sleep for one second each time
                }

              break;
            case CONNECTED:
              try
                {
                  double timestamp;
                  NODELET_DEBUG("Starting a new grab from camera.");
                  if(!sh_->grabImage(temperature_map, timestamp))  return;


#if 0
                  /* get temperature */
                  auto ref_temp = sh_->getSensorTemperature();
                  ROS_INFO("board temp: %f", ref_temp);

                  auto frameval2celsius = [](float frameval, float fpatemp)
                    {
                      // based on http://takesan.hatenablog.com/entry/2016/02/18/194252
                      return ((0.05872*frameval-472.22999f+fpatemp) - 32.0f)/1.8f;
                    };

                  float value_temp = frameval2celsius(frameBuffer[i], fpatemp_f);
#endif


                  // Publish the message using standard image transport
                  if(it_pub_.getNumSubscribers() > 0)
                    {
                      double min_val, max_val;
                      cv::minMaxLoc(temperature_map, &min_val, &max_val);

                      thermal_map.forEach<cv::Vec3b>([&](cv::Vec3b &p, const int position[2]) -> void
                        {
                          uint8_t normalized_val = (temperature_map.ptr<uint16_t>(position[0])[ position[1] ] - min_val ) / (max_val - min_val) * 255.0;
                          //ROS_INFO("[%d, %d]: %d", position[1], position[0], );
                          p[0] = Lepton::colorMap(3 * normalized_val);
                          p[1] = Lepton::colorMap(3 * normalized_val + 1);
                          p[2] = Lepton::colorMap(3 * normalized_val + 2);
                        });

                      ci_->header.stamp.fromSec(timestamp);
                      sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(ci_->header, sensor_msgs::image_encodings::RGB8, thermal_map).toImageMsg();
                      it_pub_.publish(img_msg, ci_);
                    }
                }
              catch(std::runtime_error& e)
                {
                  NODELET_ERROR("%s", e.what());

                  state = ERROR;
                }
              break;
            default:
              NODELET_ERROR("Unknown camera state %d!", state);
            }
        }
      NODELET_DEBUG("Leaving thread.");
    }

    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
    image_transport::CameraPublisher it_pub_;


    boost::shared_ptr<SensorHandler> sh_; ///< Instance of the PointGreyCamera library, used to interface with the hardware.
    sensor_msgs::CameraInfoPtr ci_; ///< Camera Info message.
    std::string frame_id_; ///< Frame id for the camera messages, defaults to 'camera'
    boost::shared_ptr<boost::thread> pubThread_; ///< The thread that reads and publishes the images.
  };

  PLUGINLIB_EXPORT_CLASS(lepton_camera_driver::LeptonThermalSensorNodelet, nodelet::Nodelet)
}


