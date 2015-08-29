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

  n_sec_offset_ = 0;
  sec_offset_ = 0;
  offset_ = 0;

  comm_system_id_ =81;
  comm_comp_id_ =50;

  terminate_start_flag_ = false;

  //debug
  terminate_start_flag_ = true;

  

}

SerialComm::~SerialComm()
{
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

    terminate_timer_ = nhp_.createTimer(ros::Duration(0.005), &SerialComm::terminateCallback, this);
    // synchronize the time per 5 sec
    sync_timer_ = nhp_.createTimer(ros::Duration(5.0), &SerialComm::syncCallback, this);

    comm_connected_ = true;

    /* シグナルハンドラの設定 */
    //signal(SIGINT, &SerialComm::signalCatch);
    return true;
}

void SerialComm::close(void)
{
  static bool once_flag = true;

  if(once_flag)
    {
      once_flag = false;
      comm_timer_.cancel();
      comm_port_.close();
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

        //ROS_WARN("Read error: %s", error.message().c_str());

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


    uint32_t chksum = 0;

    for (size_t i = 0; i < bytes_transferred; i++) 
      {
        //ROS_INFO("no.%d: %d", (int)i+1, comm_buffer_[(int)i]);
        
        if(((int)i > 1) && ((int)i < bytes_transferred - 1) )
          chksum += comm_buffer_[(int)i];
      }

    if((comm_buffer_[0] == 0xff) && (comm_buffer_[1] == 0xff) &&
       ((255 - chksum %256) == comm_buffer_[bytes_transferred -1]))
      ROS_INFO(" OKOKOKOKOKO");

    readStart(1000);
}

void SerialComm::readStart(uint32_t timeout_ms)
{

  comm_port_.async_read_some(boost::asio::buffer(comm_buffer_, sizeof(comm_buffer_)),
                             boost::bind(&SerialComm::readCallback, this, 
                                         boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred));
  if (timeout_ms != 0)
    {
      comm_timer_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
      comm_timer_.async_wait(boost::bind(&SerialComm::timeoutCallback, this, boost::asio::placeholders::error));
    }
}


void SerialComm::syncCallback(const ros::TimerEvent& timer_event)
{
  if(time_sync_flag_ == 0)
  {
    time_sync_flag_ = TIME_SYNC_CALIB_COUNT + 1;
    sec_offset_ = 0;
    n_sec_offset_ = 0;
  }
}


void SerialComm::terminateCallback(const ros::TimerEvent& timer_event)
{
  static int test = 0;

  if(terminate_start_flag_)
    {

      //ROS_WARN("Test");
      comm_connected_ = false;


      FourElements four_elements;
      four_elements.Elements.roll_cmd = 1.2345;
      four_elements.Elements.pitch_cmd = -6.789;
      four_elements.Elements.yaw_cmd = -0.12121;
      four_elements.Elements.throttle_cmd = -1234544.23;


      size_t message_len = 3;
      comm_buffer_[0] = 0xff;
      comm_buffer_[1] = 0xff;
      comm_buffer_[2] = 200;
      comm_buffer_[3] = 255 - comm_buffer_[2] % 256;

#if 1
      if (test == 0)
        {
          test = 0;
          uint32_t chksum = 0;
          message_len = 2 + 1 + 1 + 16  + 1; //2head + cmd + cmd_chksum + message +  checksum

          for(int i = 0; i < 16; i ++)
            {
              comm_buffer_[i + 4] = four_elements.message[i];
              chksum += comm_buffer_[i + 4];
            }
          comm_buffer_[4 + 16] = 255 - chksum%256;

          if (message_len != boost::asio::write(comm_port_, boost::asio::buffer(comm_buffer_, message_len)))
            {
              ROS_WARN("Unable to send terminating stop msg over serial port_.");
            }
        }

#else
      if (message_len != boost::asio::write(comm_port_, boost::asio::buffer(comm_buffer_, message_len)))
        {
          ROS_WARN("Unable to send terminating stop msg over serial port_.");
        }

      message_len = 16;

      for(int i = 0; i < 16; i ++)
        {
          comm_buffer_[i] = four_elements.message[i];
          //ROS_INFO("comm_buffer[%i], %d", i, comm_buffer_[i]);
        }

      if (message_len != boost::asio::write(comm_port_, boost::asio::buffer(comm_buffer_, message_len)))
        {
          ROS_WARN("Unable to send terminating stop msg over serial port_.");
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


