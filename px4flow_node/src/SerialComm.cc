#include <boost/bind.hpp>

// ROS includes
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include "px4flow_node/SerialComm.h"




namespace px
{

  bool  SerialComm::terminatingStartFlag = false;

SerialComm::SerialComm(const std::string& frameId)
 : m_port(m_uartService)
 , m_timer(m_uartService)
 , m_imageSize(0)
 , m_imagePackets(0)
 , m_imagePayload(0)
 , m_imageWidth(0)
 , m_imageHeight(0)
 , m_systemId(-1)
 , m_compId(0)
 , m_frameId(frameId)
 , m_timeout(false)
 , m_errorCount(0)
 , m_connected(false)
{

  stopGyroMsg = false;
  stopImageMsg = false;

  nSecOffset = 0;
  secOffset = 0;
  offset = 0;

  m_systemId =81;
  m_compId =50;

  terminatingStartFlag = false;
  commStartFlag = false;

  prevDistance = 0;

}

SerialComm::~SerialComm()
{
  printf(" destructor\n");

  close();
}

bool
SerialComm::open(const std::string& portStr, int baudrate)
{
    m_timeout = false;
    m_errorCount = 0;
    m_connected = false;

    //add by bakui    
    timeSyncFlag = TIME_SYNC_CALIB_COUNT + 1;
    // open port
    try
    {
        m_port.open(portStr);

        ROS_INFO("Opened serial port %s.", portStr.c_str());
    }
    catch (boost::system::system_error::exception e)
    {
        ROS_ERROR("Could not open serial port %s. Reason: %s.", portStr.c_str(), e.what());

        return false;
    }

    // configure baud rate
    try
    {
        m_port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        m_port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        m_port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        m_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        m_port.set_option(boost::asio::serial_port_base::character_size(8));

        ROS_INFO("Set baudrate %d.", baudrate);
    }
    catch (boost::system::system_error::exception e)
    {
        ROS_ERROR("Could not set baudrate %d. Reason: %s.", baudrate, e.what());

        return false;
    }

    ros::NodeHandle nh("px4flow");
    ros::NodeHandle nh_private("~");

    // rosparam
    nh_private.param("sonar_min_range", sonar_min_range_, 0.305);
    nh_private.param("sonar_min_range", sonar_min_range_, 5.0);

    // set up publishers
    m_optFlowPub = nh.advertise<geometry_msgs::TwistStamped>("opt_flow", 5);
    m_sonarPub = nh.advertise<sensor_msgs::Range>("sonar", 5);

    // set up thread to asynchronously read data from serial port
    readStart(1000);
    m_uartThread = boost::thread(boost::bind(&boost::asio::io_service::run, &m_uartService));


    terminateTimer = nh_private.createTimer(ros::Duration(0.1), &SerialComm::terminateCallback, this);
    // synchronize the time per 5 sec
    syncTimer = nh_private.createTimer(ros::Duration(5.0), &SerialComm::syncCallback, this);
    commStartTimer = nh_private.createTimer(ros::Duration(0.1), &SerialComm::commStartCallback, this);

    m_connected = true;

    /* シグナルハンドラの設定 */
    signal(SIGINT, &px::SerialComm::signalCatch);
    return true;
}

void
SerialComm::close(void)
{
  static bool onceFlag = true;

  if(onceFlag)
    {
      onceFlag = false;

      printf("       close!!1 \n");
      m_timer.cancel();
      m_port.close();
      printf("       close!!2 \n");

    }
}

void
SerialComm::signalCatch(int sig)
{
  printf("            signal:%d\n", sig);

  px::SerialComm::terminatingStartFlag = true;
}


void
SerialComm::readCallback(const boost::system::error_code& error, size_t bytesTransferred)
{
    if (error)
    {
        if (error == boost::asio::error::operation_aborted)
        {
            // if serial connection timed out, try reading again
            if (m_timeout)
            {
                m_timeout = false;
                readStart(1000);

                return;
            }
        }

        ROS_WARN("Read error: %s", error.message().c_str());

        if (m_errorCount < 10)
        {
            readStart(1000);
        }
        else
        {
            ROS_ERROR("# read errors exceeded 10. Aborting...");
            ros::shutdown();
        }

        ++m_errorCount;

        return;
    }

    m_timer.cancel();

    mavlink_message_t message;
    mavlink_status_t status;

    for (size_t i = 0; i < bytesTransferred; i++) {
        bool msgReceived = mavlink_parse_char(MAVLINK_COMM_1, m_buffer[i], &message, &status);

        if (msgReceived)
        {
            m_systemId = message.sysid;
            m_compId = message.compid;
            //ROS_WARN("systemId: %d, compId: %d", message.sysid, message.compid);


            switch (message.msgid)
            {
            case MAVLINK_MSG_ID_SYSTEM_TIME:
              {
                ROS_WARN("time sync");
                break;
              }
            case MAVLINK_MSG_ID_OPTICAL_FLOW:
            {
                // decode message
                mavlink_optical_flow_t flow;
                mavlink_msg_optical_flow_decode(&message, &flow);

              ros::Time px4Time 
                = ros::Time(flow.time_usec / 1000000, (flow.time_usec % 1000000) * 1000);

              if(timeSyncFlag > 0)
                {
                  timeSyncFlag --;
                  if(timeSyncFlag < TIME_SYNC_CALIB_COUNT)
                    {
                      offsetTmp = ros::Time::now().toNSec() - px4Time.toNSec();
                      secOffset  = secOffset +  (offsetTmp / 1000000000UL);
                      nSecOffset = nSecOffset + (offsetTmp % 1000000000UL);
                    }
                  if(timeSyncFlag == 0)
                    {
                      commStartFlag = true;
                      secOffset /= TIME_SYNC_CALIB_COUNT;
                      nSecOffset /= TIME_SYNC_CALIB_COUNT;
                      offset = secOffset * 1000000000UL + nSecOffset - 8000000; // 0.008のletency
                    }
                }

              if(commStartFlag)
                {
                  //publish the sonar only when sonar change the value
                  if(prevDistance != flow.ground_distance)
                    {
                      sensor_msgs::Range range_msg;
                      range_msg.header.stamp.fromNSec(px4Time.toNSec() + offset); //temporarily
                      range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
                      range_msg.min_range = sonar_min_range_;
                      range_msg.max_range = sonar_max_range_;
                      range_msg.range = flow.ground_distance;
                      m_sonarPub.publish(range_msg);
                    }

                  //publish the optical flow data
                  //optFlowMsg.header.stamp.fromNSec(px4Time.toNSec());
                  geometry_msgs::TwistStamped optFlowMsg;
                  optFlowMsg.header.stamp.fromNSec(px4Time.toNSec() + offset); //temporarily
                  // linear: add the raw optical flow(is not metric scale, is pixel scale)
                  optFlowMsg.twist.linear.x = flow.flow_x /1000.0;
                  optFlowMsg.twist.linear.y = flow.flow_y /1000.0;
                  optFlowMsg.twist.linear.z = flow.quality;
                  // angular: add the metric optical flow(this is metric value using the focal length and distance)
                  optFlowMsg.twist.angular.x = flow.flow_comp_m_x;
                  optFlowMsg.twist.angular.y = flow.flow_comp_m_y;
                  optFlowMsg.twist.angular.z = 0; //facal length: this means we have the absolute(metric) value of the optical flow
                  /////////////////////////////////////////////////
                  // optFlowMsg.velocity_x = flow.flow_comp_m_x; //
                  // optFlowMsg.velocity_y = flow.flow_comp_m_y; //
                  // optFlowMsg.quality = flow.quality;          //
                  // optFlowMsg.flow_x = flow.flow_x;            //
                  // optFlowMsg.flow_y = flow.flow_y;            //
                  /////////////////////////////////////////////////
                  m_optFlowPub.publish(optFlowMsg);
                }

              prevDistance = flow.ground_distance;
                break;
            }
            case MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE:
            {
              ROS_WARN(" hand shake");
                mavlink_data_transmission_handshake_t handshake;
                mavlink_msg_data_transmission_handshake_decode(&message, &handshake);

                m_imageSize = handshake.size;
                m_imagePackets = handshake.packets;
                m_imagePayload = handshake.payload;
                m_imageWidth = handshake.width;
                m_imageHeight = handshake.height;

                if (m_imageBuffer.size() < m_imageSize)
                {
                    m_imageBuffer.resize(m_imageSize);
                }

                break;
            }
           case MAVLINK_MSG_ID_PARAM_VALUE:
            {
              ROS_WARN(" param set response");
              mavlink_param_value_t param;
              mavlink_msg_param_value_decode(&message, &param);

              char gyro_debug_msg_id[16];
              char video_msg_id[16];
              char flow_msg_id[16];

              strcpy(gyro_debug_msg_id,"USB_SEND_GYRO");
              strcpy(video_msg_id,"USB_SEND_VIDEO");
              strcpy(flow_msg_id,"USB_SEND_FLOW");

              if(strcmp(gyro_debug_msg_id, param.param_id) == 0)
                {
                  ROS_INFO("gyro debug msg response, value is %f", param.param_value);
                  if(param.param_value == 0)
                    stopGyroMsg = false;
                }
              else if(strcmp(video_msg_id, param.param_id) == 0)
                {
                  ROS_INFO("image msg response, value is %f", param.param_value);
                  if(param.param_value == 0)
                    stopImageMsg = false;
                }
              else if(strcmp(flow_msg_id, param.param_id) == 0)
                {

                  if(param.param_value == 0)
                    {
                      if(timeSyncFlag > TIME_SYNC_CALIB_COUNT)
                        {  // the remained previous data  in buffer
                          ROS_ERROR("The remained previous data  in buffer!!");
                        }
                      else
                        { // true stopping process 
                          ROS_INFO("flow msg response, value is %f", param.param_value);
                          terminatingStartFlag = false;
                          //close();
                          printf("       close!!3 \n");
                          ros::shutdown();
                        }
                    }
                }

              break;
            }
            case MAVLINK_MSG_ID_ENCAPSULATED_DATA:
            {
                break;
            }
            default:
              {
                //ROS_WARN("somethingelse, %d",message.msgid);
              }
            }
        }
    }

    readStart(1000);
}

void
SerialComm::readStart(uint32_t timeout_ms)
{

  m_port.async_read_some(boost::asio::buffer(m_buffer, sizeof(m_buffer)),
                         boost::bind(&SerialComm::readCallback, this, boost::asio::placeholders::error,
                                     boost::asio::placeholders::bytes_transferred));
  if (timeout_ms != 0)
    {
      m_timer.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
      m_timer.async_wait(boost::bind(&SerialComm::timeoutCallback, this, boost::asio::placeholders::error));
    }

}
  
void
SerialComm::commStartCallback(const ros::TimerEvent& timerEvent)
{

  if(m_connected && !commStartFlag )
    {

      //ROS_ERROR("init");

#if 1 //original
      struct timeval tv;
      gettimeofday(&tv, NULL);
      uint64_t timeNow = static_cast<uint64_t>(tv.tv_sec) * 1000000 + tv.tv_usec;
#else //ros time
      uint64_t timeNow = ros::Time::now().sec * 1000000 + (int)(ros::Time::now().nsec / 1000);
#endif
    

      mavlink_message_t msg;
      mavlink_msg_system_time_pack(m_systemId, m_compId, &msg, timeNow, 0);

      size_t messageLength = mavlink_msg_to_send_buffer(m_buffer, &msg);
      if (messageLength != boost::asio::write(m_port, boost::asio::buffer(m_buffer, messageLength)))
        {
          ROS_WARN("Unable to send system time over serial port.");
        }
    }




    if(stopImageMsg)
      {
        mavlink_message_t msg;
        char usb_send_video_id[16];
        strcpy(usb_send_video_id,"USB_SEND_VIDEO");

        mavlink_msg_param_set_pack(m_systemId, m_compId, &msg, m_systemId, m_compId, 
                                   usb_send_video_id, 0, MAV_PARAM_TYPE_UINT8);
        size_t messageLength = mavlink_msg_to_send_buffer(m_buffer, &msg);
        if (messageLength != boost::asio::write(m_port, boost::asio::buffer(m_buffer, messageLength)))
          {
            ROS_WARN("Unable to send video disable msg over serial port.");
          }
      }
}


void
SerialComm::syncCallback(const ros::TimerEvent& timerEvent)
{
  if(commStartFlag && timeSyncFlag == 0)
  {
    timeSyncFlag = TIME_SYNC_CALIB_COUNT + 1;
    secOffset = 0;
    nSecOffset = 0;
  }
}


void
SerialComm::terminateCallback(const ros::TimerEvent& timerEvent)
{
  if(terminatingStartFlag)
    {
      //not good
      m_connected = false;

      ROS_ERROR("OK");

        mavlink_message_t msg;
        char usb_send_flow_id[16];
        strcpy(usb_send_flow_id,"USB_SEND_FLOW");

        mavlink_msg_param_set_pack(m_systemId, m_compId, &msg, m_systemId, m_compId, 
                                   usb_send_flow_id, 0, MAV_PARAM_TYPE_UINT8);
        size_t messageLength = mavlink_msg_to_send_buffer(m_buffer, &msg);
        if (messageLength != boost::asio::write(m_port, boost::asio::buffer(m_buffer, messageLength)))
          {
            ROS_WARN("Unable to send terminating stop msg over serial port.");
          }
    }

}



void
SerialComm::timeoutCallback(const boost::system::error_code& error)
{
    if (!error)
    {
        m_port.cancel();
        m_timeout = true;
        ROS_WARN("Serial connection timed out.");
    }
}

}
