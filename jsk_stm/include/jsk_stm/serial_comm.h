#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

// for ros
#include <ros/ros.h>


// for singal handler
#include <signal.h>
#include <errno.h>

class SerialComm
{
public:
  SerialComm(ros::NodeHandle nh, ros::NodeHandle nhp_, const std::string& frame_id);
  ~SerialComm();

    bool open(const std::string& port_str, int baudrate);
    void close(void);

    static void signalCatch(int sig);
    static bool terminate_start_flag_;

    static const int TIME_SYNC_CALIB_COUNT = 40;
    static const int MAX_PACKET_LEN = 512;//temp

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;


    void readCallback(const boost::system::error_code& error, size_t bytes_transferred);
    void readStart(uint32_t timeout_ms);
    void syncCallback(const ros::TimerEvent& timer_event);
    void terminateCallback(const ros::TimerEvent& timer_event);

    void timeoutCallback(const boost::system::error_code& error);

    boost::asio::io_service comm_uart_service_;
    boost::asio::serial_port comm_port_;
    boost::asio::deadline_timer comm_timer_;
    boost::thread comm_uart_thread_;

    int comm_system_id_;
    int comm_comp_id_;

    uint8_t comm_buffer_[MAX_PACKET_LEN];


    ros::Timer sync_timer_;
    ros::Timer terminate_timer_;

    bool comm_timeout_;
    int comm_error_count_;
    bool comm_connected_;

    std::string comm_frame_id_;

    //for ros timestamp sync
    ros::Time time_offest_;
    unsigned long offset_;
    unsigned long offset_tmp_;
    unsigned long sec_offset_;
    unsigned long n_sec_offset_;
    int time_sync_flag_;

      typedef union{
        uint8_t message[16];
        struct{
          float roll_cmd;
          float pitch_cmd;
          float yaw_cmd;
          float throttle_cmd;
        }Elements;
      }FourElements;

};

