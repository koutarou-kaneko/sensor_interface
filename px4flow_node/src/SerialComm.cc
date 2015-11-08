#include <boost/bind.hpp>


// ROS includes
#include <px_comm/OpticalFlow.h>
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

  //ROS_ERROR("init");
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

    // set up publishers
    m_optFlowPub = nh.advertise<px_comm::OpticalFlow>("opt_flow", 5);

    image_transport::ImageTransport it(nh);
    m_imagePub = it.advertise("camera_image", 5);

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


      // if (!m_connected)
      //   {
      //     return;
      //   }

      printf("       close!!1 \n");
      m_timer.cancel();
      m_port.close();
       // m_uartService.post(boost::bind(&boost::asio::deadline_timer::cancel, &m_timer));
       // m_uartService.post(boost::bind(&boost::asio::serial_port::close, &m_port));
       // m_uartThread.join();
      printf("       close!!2 \n");
      //ros::shutdown();
      //exit(0);
    }
}

void
SerialComm::signalCatch(int sig)
{
  printf("            signal:%d\n", sig);

  px::SerialComm::terminatingStartFlag = true;
  //while(!terminatingStartFlag) {} //terminate flagが立つまで待つ
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
                      //printf("%i.",timeSyncFlag);
                      offsetTmp = ros::Time::now().toNSec() - px4Time.toNSec();
                      secOffset  = secOffset +  (offsetTmp / 1000000000UL);
                      nSecOffset = nSecOffset + (offsetTmp % 1000000000UL);
                      // printf("%i. %ld, %ld , %ld, %ld\n", timeSyncFlag, 
                      //        offset % 1000000000UL, nSecOffset,
                      //        offset / 1000000000UL, secOffset);
                    }
                  if(timeSyncFlag == 0)
                    {
                      commStartFlag = true;
                      secOffset /= TIME_SYNC_CALIB_COUNT;
                      nSecOffset /= TIME_SYNC_CALIB_COUNT;
                      offset = secOffset * 1000000000UL + nSecOffset - 8000000; // 0.008のletency
                      //printf("  time synchronization finished, %ld, %ld\n", nSecOffset, secOffset);
                      
                    }
                }

              if(commStartFlag)
                {
                  px_comm::OpticalFlow optFlowMsg;

                  //optFlowMsg.header.stamp.fromNSec(px4Time.toNSec());
                  optFlowMsg.header.stamp.fromNSec(px4Time.toNSec() + offset ); //temporarily
                  //ROS_WARN("time stamp is %lf", optFlowMsg.header.stamp.toSec());
                  optFlowMsg.header.frame_id = m_frameId;
                  optFlowMsg.ground_distance = flow.ground_distance;
                  optFlowMsg.flow_x = flow.flow_x;
                  optFlowMsg.flow_y = flow.flow_y;
                  optFlowMsg.velocity_x = flow.flow_comp_m_x;
                  optFlowMsg.velocity_y = flow.flow_comp_m_y;
                  optFlowMsg.quality = flow.quality;


                  m_optFlowPub.publish(optFlowMsg);

                }
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
              ROS_WARN(" image msg");
                if (m_imageSize == 0 || m_imagePackets == 0)
                {
                    break;
                }

                mavlink_encapsulated_data_t img;
                mavlink_msg_encapsulated_data_decode(&message, &img);
                size_t seq = img.seqnr;
                size_t pos = seq * m_imagePayload;

                if (seq + 1 > m_imagePackets)
                {
                    break;
                }

                size_t bytesToCopy = m_imagePayload;
                if (pos + m_imagePayload >= m_imageSize)
                {
                     bytesToCopy = m_imageSize - pos;
                }

                memcpy(&m_imageBuffer[pos], img.data, bytesToCopy);

                if (seq + 1 == m_imagePackets)
                {
                    sensor_msgs::Image image;
                    image.header.frame_id = m_frameId;
                    image.height = m_imageHeight;
                    image.width = m_imageWidth;
                    image.encoding = sensor_msgs::image_encodings::MONO8;
                    image.is_bigendian = false;
                    image.step = m_imageWidth;

                    image.data.resize(m_imageSize);
                    memcpy(&image.data[0], &m_imageBuffer[0], m_imageSize);

                    m_imagePub.publish(image);
                }
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
