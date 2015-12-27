#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

// for ros
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <jsk_stm/JskImu.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

//for parsing the flight control message
#include <aerial_robot_msgs/RcData.h> //old 
#include <aerial_robot_msgs/FourAxisCommand.h>

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

  static const uint8_t FIRST_HEADER = 0xff;
  static const uint8_t SECOND_HEADER = 0xff;

  static const uint8_t START_CMD = 0x00;
  static const uint8_t MPU_ACC_GYRO_CALIB_CMD = 0x01;
  static const uint8_t MPU_MAG_CALIB_CMD = 0x02;
  static const uint8_t BARO_CALIB_CMD = 0x03;
  static const uint8_t RESET_CALIB_CMD = 0x10;
  static const uint8_t END_CMD = 0x11;

  static const uint8_t ROTOR_START_MSG = 0x20;
  static const uint8_t ROTOR_STOP_MSG = 0x21;
  static const uint8_t PWM_TEST_CMD = 0x30;
  static const float PWM_MAX = 2000; //2000us

  static const uint8_t IMU_DATA_MSG = 190;
  static const uint8_t FOUR_ELEMENTS_CMD = 200;

  //static const uint8_t IMU_DATA_SIZE =o 37; // 8 + 2*12 + 4 + 1(chksum)
  static const uint8_t IMU_DATA_SIZE = 61; // 8 + 4*12 + 4 + 1(chksum)
  //static const uint8_t IMU_DATA_SIZE = 53; // 4*12 + 4 + 1(chksum)
  //static const uint8_t IMU_DATA_SIZE = 69; // 4*12 + 16 + 4 + 1(chksum)

  static const uint8_t FOUR_ELEMNET_CMD = 0x10;
  static const uint8_t GYRO_ACC_CALIB_CMD = 0x11;
  static const uint8_t GYRO_CALIB_CMD = 0x12;
  static const uint8_t MAG_CALIB_CMD = 0x13;

  static const uint8_t FIRST_HEADER_STAGE = 0x00;
  static const uint8_t SECOND_HEADER_STAGE = 0x01;
  static const uint8_t MSG_TYPE_STAGE = 0x02;
  static const uint8_t MSG_DATA_STAGE = 0x03;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher  imu_pub_;
  ros::Publisher  imu2_pub_;
  ros::Subscriber  config_cmd_sub_;

  ros::Publisher  test_pub_;

  ros::Subscriber  pwm_test_cmd_sub_;

ros::Subscriber  aerial_robot_control_sub_;
ros::Publisher arming_ack_pub_;

  tf::TransformBroadcaster* tfB_;

  void readCallback(const boost::system::error_code& error, size_t bytes_transferred);
  void readStart(uint32_t timeout_ms);
  void txCallback(const ros::TimerEvent& timer_event);
  void timeoutCallback(const boost::system::error_code& error);

  void configCmdCallback(const std_msgs::UInt16ConstPtr & msg);
  void pwmCmdCallback(const std_msgs::UInt16ConstPtr & msg);

  //temporarily
  void aerialRobotControlCmdCallback(const aerial_robot_msgs::RcDataConstPtr & msg);



  boost::asio::io_service comm_uart_service_;
  boost::asio::serial_port comm_port_;
  boost::asio::deadline_timer comm_timer_;
  boost::thread comm_uart_thread_;

  int comm_system_id_;
  int comm_comp_id_;

  unsigned long time_offset;

  //uint8_t comm_buffer_[MAX_PACKET_LEN]
  uint8_t comm_buffer_[1024];

  uint8_t packet_stage_;
  uint8_t receive_data_size_;
  uint8_t packet_chksum_;
  uint8_t msg_type_;

  ros::Timer tx_timer_;

  bool start_flag_;

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

  typedef union{
    uint8_t bytes[12];
    float vector[3];
  }FloatVectorUnion;

  typedef union{
    uint8_t bytes[4];
    float f_data;
  }FloatUnion;


};

