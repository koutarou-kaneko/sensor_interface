#include <jsk_stm/serial_comm.h>

bool SerialComm::terminate_start_flag_ = false;

SerialComm::SerialComm(ros::NodeHandle nh, ros::NodeHandle nhp, const std::string& frame_id)
 : comm_port_(comm_uart_service_)
 , nh_(nh), nhp_(nhp)
 , comm_timer_(comm_uart_service_)
 , comm_system_id_(-1)
 , comm_comp_id_(0)
 , comm_frame_id_(frame_id)
 , comm_timeout_(false)
 , comm_error_count_(0)
 , comm_connected_(false)
{

  imu_pub_ = nh_.advertise<jsk_stm::JskImu>("jsk_imu", 5);
  imu2_pub_ = nh_.advertise<sensor_msgs::Imu>("jsk_imu2", 5);
  config_cmd_sub_ = nh_.subscribe<std_msgs::UInt8>("config_cmd", 1, &SerialComm::configCmdCallback, this, ros::TransportHints().tcpNoDelay());

  tfB_ = new tf::TransformBroadcaster();


  n_sec_offset_ = 0;
  sec_offset_ = 0;
  offset_ = 0;

  comm_system_id_ =81;
  comm_comp_id_ =50;

  start_flag_ = true;
  terminate_start_flag_ = false;


  packet_stage_ = FIRST_HEADER_STAGE;
  receive_data_size_ = 1;
  msg_type_ = 0;
  packet_chksum_ = 0;

  //debug
  //terminate_start_flag_ = true;

  time_offset = 0;
}

SerialComm::~SerialComm()
{
  delete tfB_;
  printf(" destructor\n");

  close();
}

bool SerialComm::open(const std::string& port_str, int baudrate)
{
    comm_timeout_ = false;
    comm_error_count_ = 0;
    comm_connected_ = false;


    time_sync_flag_ = TIME_SYNC_CALIB_COUNT + 1;

    // open port
    try
    {
        comm_port_.open(port_str);

        ROS_INFO("Opened serial port %s.", port_str.c_str());
    }
    catch (boost::system::system_error::exception e)
    {
        ROS_ERROR("Could not open serial port %s. Reason: %s.", port_str.c_str(), e.what());
        return false;
    }

    // configure baud rate
    try
    {
        comm_port_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        comm_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        comm_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        comm_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        comm_port_.set_option(boost::asio::serial_port_base::character_size(8));

        ROS_INFO("Set baudrate %d.", baudrate);
    }
    catch (boost::system::system_error::exception e)
    {
        ROS_ERROR("Could not set baudrate %d. Reason: %s.", baudrate, e.what());

        return false;
    }


    // set up thread to asynchronously read data from serial port
    readStart(1000);
    comm_uart_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &comm_uart_service_));

    tx_timer_ = nhp_.createTimer(ros::Duration(0.01), &SerialComm::txCallback, this);

    comm_connected_ = true;

    /* シグナルハンドラの設定 */
    signal(SIGINT, &SerialComm::signalCatch);
    return true;
}

void SerialComm::close(void)
{
  static bool once_flag = true;

  if(once_flag)
    {
      once_flag = false;
      //comm_timer_.cancel();
      //comm_port_.close();
    }
}

void SerialComm::signalCatch(int sig)
{
  printf("            signal:%d\n", sig);

  terminate_start_flag_ = true;
}


void SerialComm::readCallback(const boost::system::error_code& error, size_t bytes_transferred)
{
    if (error)
    {
        if (error == boost::asio::error::operation_aborted)
        {
            // if serial connection timed out, try reading again
            if (comm_timeout_)
            {
                comm_timeout_ = false;
                readStart(1000);
                ROS_WARN("restart");
                return;
            }
        }

        ROS_WARN("Read error: %s", error.message().c_str());

        if (comm_error_count_ < 10)
        {
            readStart(1000);
        }
        else
        {
            ROS_ERROR("# read errors exceeded 10. Aborting...");
            ros::shutdown();
        }

        ++comm_error_count_;

        return;
    }

    comm_timer_.cancel();

    switch(packet_stage_)
      {

      case FIRST_HEADER_STAGE:
        {
          if(comm_buffer_[0] == FIRST_HEADER)
            {
              packet_stage_ = SECOND_HEADER_STAGE;
              receive_data_size_ = 1;
              //ROS_INFO("first header ok");
            }
          else
            {
              packet_stage_ = FIRST_HEADER_STAGE;
              receive_data_size_ = 1;
              ROS_ERROR("first header bad");
            }
          break;
        }
      case SECOND_HEADER_STAGE:
        {
          if(comm_buffer_[0] == SECOND_HEADER)
            {
              packet_stage_ = MSG_TYPE_STAGE;
              receive_data_size_ = 2;
              //ROS_INFO("second header ok");
            }
          else
            {
              packet_stage_ = FIRST_HEADER_STAGE;
              receive_data_size_ = 1;
              ROS_ERROR("second header bad");
            }
          break;
        }
      case MSG_TYPE_STAGE:
        {
          if(comm_buffer_[0] + comm_buffer_[1] == 0xff)
            {
              //ROS_INFO("msg type ok");
              msg_type_ = comm_buffer_[0];
              switch(msg_type_)
                {
                case IMU_DATA_MSG:
                  {
                    packet_stage_ = MSG_DATA_STAGE;
                    receive_data_size_ = IMU_DATA_SIZE;
                    //ROS_INFO("msg type imu data");
                    break;
                  }
                case MPU_ACC_GYRO_CALIB_CMD:
                  { 
                    ROS_INFO("calib acc and gyro ack");
                    packet_stage_ = FIRST_HEADER_STAGE;
                    receive_data_size_ = 1;
                    break;
                  }
                case START_CMD:
                  { 
                    ROS_INFO("start communication");
                    packet_stage_ = FIRST_HEADER_STAGE;
                    receive_data_size_ = 1;
                    start_flag_ = false;
                    break;
                  }
                case END_CMD:
                  {
                    if(terminate_start_flag_)
                      {
                        ROS_WARN("end communication \n");
                        ros::shutdown();
                      }
                    packet_stage_ = FIRST_HEADER_STAGE;
                    receive_data_size_ = 1;
                    break;
                  }
                case ROTOR_START_ACK_MSG:
                  {
                    ROS_WARN("start control ack\n");
                    break;
                  }
                case ROTOR_STOP_ACK_MSG:
                  {
                    ROS_WARN("stop control ack\n");
                    break;
                  }
                case FOUR_ELEMENTS_CMD:
                  {
                    ROS_INFO("four elements cmd ack");
                    packet_stage_ = FIRST_HEADER_STAGE;
                    receive_data_size_ = 1;
                    break;
                  }
                default:
                  {
                    packet_stage_ = FIRST_HEADER_STAGE;
                    receive_data_size_ = 1;
                    ROS_ERROR("wrong msg type");
                    break;
                  }
                }
            }
          else
            {
              packet_stage_ = FIRST_HEADER_STAGE;
              receive_data_size_ = 1;
              ROS_ERROR("msg type bad");
            }
          break;
        }
      case MSG_DATA_STAGE:
        {
          //ROS_INFO("%d", bytes_transferred);
          uint8_t chksum = 0;
          for (size_t i = 0; i < bytes_transferred -1 ; i++) 
            {
              chksum += comm_buffer_[(int)i];
              //debug
              //ROS_INFO("%d, %x", i, comm_buffer_[(int)i]);
            }


          //ROS_INFO("%x, %x", chksum, comm_buffer_[bytes_transferred -1]);
          
          if((255 - chksum %256) == comm_buffer_[bytes_transferred -1])
            {
              //ROS_INFO("ok");

              if(msg_type_ == IMU_DATA_MSG)
                {// IMU DATA
                  jsk_stm::JskImu imu_msg;
                  imu_msg.header.stamp = ros::Time::now();

                  FloatVectorUnion acc_union , gyro_union, mag_union, angles_union;
                  FloatUnion alt_union;

                  tf::Quaternion q;
                  uint64_t time_stamp_int = 0;

#if 1 // no quternion
                   for(int i = 0; i < 8; i ++)
                     time_stamp_int |= comm_buffer_[i] << (4*i); // should be | not +
                   //ROS_INFO("time: %ld", time_stamp_int);
                  time_stamp_int *=  1000000;
                   if(time_offset == 0)
                     {
                       time_offset = ros::Time::now().toNSec() 
                         - time_stamp_int;// - 1000000;  //1ms delay
                     }
                   imu_msg.header.stamp.fromNSec(time_stamp_int + time_offset );
                   //ROS_INFO("time: %f", imu_msg.header.stamp.toSec());

                  for(int i = 0; i < 12; i ++)
                    {
                      acc_union.bytes[i] = comm_buffer_[8 + i];
                      angles_union.bytes[i] = comm_buffer_[20 + i];
                    }
                  q.setRPY(angles_union.vector[0],angles_union.vector[1],angles_union.vector[2]);

                  for(int i = 0; i < 3; i ++) //degree conversion
                    angles_union.vector[i] = angles_union.vector[i] * 180 / M_PI;

                  for(int i = 0; i < 4; i ++)
                    {
                      alt_union.bytes[i] = comm_buffer_[32 + i];
                    }
#else //no mag and alt
                  for(int i = 0; i < 12; i ++)
                    {
                      acc_union.bytes[i] = comm_buffer_[i];
                      gyro_union.bytes[i] = comm_buffer_[12 + i];
                      mag_union.bytes[i] = comm_buffer_[24 + i];
                    }

                  float quaternion[4];
                  for(int i = 0; i < 4; i ++)
                    {
                      quaternion[i] = *((float *)(&comm_buffer_[36 + i*4]));
                    }
                  q.setValue(quaternion[1], quaternion[2],quaternion[3], quaternion[0]);

#endif
                  ros::Time tfStamp = ros::Time::now();
                  tf::Transform baseToImu;

                  baseToImu.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
                  baseToImu.setRotation(q);
                  tfB_->sendTransform(tf::StampedTransform(baseToImu, tfStamp, "base", "imu"));

                  imu_msg.acc_data.x = acc_union.vector[0];
                  imu_msg.acc_data.y = acc_union.vector[1];
                  imu_msg.acc_data.z = acc_union.vector[2];
                  imu_msg.gyro_data.x = gyro_union.vector[0];
                  imu_msg.gyro_data.y = gyro_union.vector[1];
                  imu_msg.gyro_data.z = gyro_union.vector[2];
                  imu_msg.mag_data.x = mag_union.vector[0];
                  imu_msg.mag_data.y = mag_union.vector[1];
                  imu_msg.mag_data.z = mag_union.vector[2];
                  imu_msg.angles.x = angles_union.vector[0];
                  imu_msg.angles.y = angles_union.vector[1];
                  imu_msg.angles.z = angles_union.vector[2];
                  imu_msg.altitude = alt_union.f_data;

                  imu_pub_.publish(imu_msg);

                  /*
                  sensor_msgs::Imu imu2_data;
                  imu2_data.header.stamp = ros::Time::now();
                  imu2_data.header.frame_id = "base";
                  imu2_data.orientation.w = quaternion[0];
                  imu2_data.orientation.x = quaternion[1];
                  imu2_data.orientation.y = quaternion[2];
                  imu2_data.orientation.z = quaternion[3];
                  imu2_pub_.publish(imu2_data);
                  */
                }
            }
          else
            {
              ROS_WARN("wrong checksum");
            }

          packet_stage_ = FIRST_HEADER_STAGE;
          receive_data_size_ = 1;
          break;
        }
      default:
        {
          packet_stage_ = FIRST_HEADER_STAGE;
          receive_data_size_ = 1;
          break;
        }
      }
    //ROS_INFO("The bytes is %d", bytes_transferred);

    readStart(1000);
}

void SerialComm::readStart(uint32_t timeout_ms)
{
  boost::asio::async_read(comm_port_,boost::asio::buffer(comm_buffer_, receive_data_size_),
                          boost::bind(&SerialComm::readCallback, this, 
                                      boost::asio::placeholders::error,
                                      boost::asio::placeholders::bytes_transferred));

  // boost::asio::async_read(comm_port_,boost::asio::buffer(comm_buffer_, sizeof(comm_buffer_)),
  //                         boost::asio::transfer_at_least(50),
  //                   boost::bind(&SerialComm::readCallback, this, boost::asio::placeholders::error,
  //                               boost::asio::placeholders::bytes_transferred));


  if (timeout_ms != 0)
    {
      comm_timer_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
      comm_timer_.async_wait(boost::bind(&SerialComm::timeoutCallback, this, boost::asio::placeholders::error));
    }
}



void SerialComm::txCallback(const ros::TimerEvent& timer_event)
{
  static int test = 0;
  uint8_t write_buffer[21];
  size_t message_len = 21;

  for(int i = 0; i < message_len; i ++)
    write_buffer[i] = 0;


  if(start_flag_)
    {
      write_buffer[0] = 0xff;
      write_buffer[1] = 0xff;
      write_buffer[2] = START_CMD;
      write_buffer[3] = 255 - write_buffer[2] % 256;

      if (message_len != boost::asio::write(comm_port_, boost::asio::buffer(write_buffer, message_len)))
        ROS_WARN("Unable to send terminating stop msg over serial port_.");
    }

  if(terminate_start_flag_)
    {
      write_buffer[0] = 0xff;
      write_buffer[1] = 0xff;
      write_buffer[2] = END_CMD;
      write_buffer[3] = 255 - write_buffer[2] % 256;

      if (message_len != boost::asio::write(comm_port_, boost::asio::buffer(write_buffer, message_len)))
        ROS_WARN("Unable to send terminating stop msg over serial port_.");
    }
  else
    {
        // write_buffer[0] = 0xff;
        // write_buffer[1] = 0xff;
        // write_buffer[2] = 64;
        // write_buffer[3] = 255 - write_buffer[2] % 256;

        // if (message_len != boost::asio::write(comm_port_, boost::asio::buffer(write_buffer, message_len)))
        //   ROS_WARN("Unable to send terminating stop msg over serial port_.");

#if 1 //test

      if(!start_flag_)
        {
          //comm_connected_ = false;

          FourElements four_elements;
          four_elements.Elements.roll_cmd = 1.2345;
          four_elements.Elements.pitch_cmd = -6.789;
          four_elements.Elements.yaw_cmd = -0.12121;
          four_elements.Elements.throttle_cmd = -1234544.23;

          write_buffer[0] = 0xff;
          write_buffer[1] = 0xff;
          write_buffer[2] = FOUR_ELEMENTS_CMD;
          write_buffer[3] = 255 - write_buffer[2] % 256;

          uint32_t chksum = 0;

          for(int i = 0; i < 16; i ++)
            {
              write_buffer[i + 4] = four_elements.message[i];
              chksum += write_buffer[i + 4];
            }
          write_buffer[20] = 255 - chksum%256;

          if (message_len != boost::asio::write(comm_port_, boost::asio::buffer(write_buffer, message_len)))
            {
              ROS_WARN("Unable to send terminating stop msg over serial port_.");
            }
        }
#endif

    }

}



void SerialComm::timeoutCallback(const boost::system::error_code& error)
{
    if (!error)
    {
        comm_port_.cancel();
        comm_timeout_ = true;
        ROS_WARN("Serial connection timed out.");
    }
    //ROS_WARN("Read error: %s", error.message().c_str());
}


void SerialComm::configCmdCallback(const std_msgs::UInt8ConstPtr & msg)
{

  uint8_t write_buffer[21];
  size_t message_len = 21;
  for(int i = 0; i < message_len; i ++)
    write_buffer[i] = 0;
  write_buffer[0] = 0xff;
  write_buffer[1] = 0xff;
  write_buffer[2] = msg->data;
  write_buffer[3] = 255 - write_buffer[2] % 256;

  if (message_len != boost::asio::write(comm_port_, boost::asio::buffer(write_buffer, message_len)))
    ROS_WARN("Unable to send terminating stop msg over serial port_.");
}
