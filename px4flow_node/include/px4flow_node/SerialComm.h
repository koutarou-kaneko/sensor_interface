#include <boost/asio.hpp>
#include <boost/thread.hpp>

// MAVLINK includes
#include <pixhawk/mavlink.h>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/TwistStamped.h>
// for singal handler
#include <signal.h>
#include <errno.h>

namespace px
{

  class SerialComm
  {
  public:
    SerialComm(const std::string& frameId);
    ~SerialComm();

    bool open(const std::string& portStr, int baudrate);
    void close(void);

    static void signalCatch(int sig);
    static bool terminatingStartFlag;
    static const int TIME_SYNC_CALIB_COUNT = 40;

  private:
    void readCallback(const boost::system::error_code& error, size_t bytesTransferred);
    void readStart(uint32_t timeout_ms);
    void syncCallback(const ros::TimerEvent& timerEvent);
    void terminateCallback(const ros::TimerEvent& timerEvent);
    void commStartCallback(const ros::TimerEvent& timerEvent);
    void timeoutCallback(const boost::system::error_code& error);

    boost::asio::io_service m_uartService;
    boost::asio::serial_port m_port;
    boost::asio::deadline_timer m_timer;
    boost::thread m_uartThread;

    size_t m_imageSize;       ///< Image size being transmitted (bytes)
    size_t m_imagePackets;    ///< Number of data packets being sent for this image
    int m_imagePayload;       ///< Payload size per transmitted packet (bytes). Standard is 254, and decreases when image resolution increases.
    int m_imageWidth;         ///< Width of the image stream
    int m_imageHeight;        ///< Width of the image stream
    std::vector<uint8_t> m_imageBuffer;

    int m_systemId;
    int m_compId;

    uint8_t m_buffer[MAVLINK_MAX_PACKET_LEN];

    ros::Publisher m_optFlowPub, m_sonarPub;
    std::string m_frameId;
    ros::Timer syncTimer;
    ros::Timer commStartTimer;
    ros::Timer terminateTimer;

    bool m_timeout;
    int m_errorCount;

    bool m_connected;



    //add by bakui
    ros::Time timeOffest;
    unsigned long offset;
    unsigned long offsetTmp;
    unsigned long secOffset;
    unsigned long nSecOffset;
    int timeSyncFlag;
    
    bool stopGyroMsg;
    bool stopImageMsg;
    bool commStartFlag;
    float prevDistance;

    double sonar_min_range_;
    double sonar_max_range_;

  };
}
