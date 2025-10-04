#ifndef CFS_SENSOR_NODE_H
#define CFS_SENSOR_NODE_H

#include <ros/ros.h>

#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Int32.h>
//#include <sensor_msgs/Imu.h>
//#include <geometry_msgs/Vector3Stamped.h>
#include <std_srvs/Empty.h>//for calibration service

#include "cfs_sensor/pCommon.h"
//#include "cfs_sensor/rs_comm.h"
#include "cfs_sensor/pComResInternal.h"

namespace cfs_sensor
{
  //$B%;%s%5$NDj3J2Y=E$r@_Dj(B
  typedef struct {
    int maxfx;
    int maxfy;
    int maxfz;
    int maxmx;
    int maxmy;
    int maxmz;
  } CFS_DEVICE_RATE_VAL;

  typedef struct {
    double fx;
    double fy;
    double fz;
    double mx;
    double my;
    double mz;
  } CFS_SENSOR_DATA;

  // =============================================================================
  //	system info
  // =============================================================================
  typedef struct ST_SystemInfo {
    int com_ok;
  } SystemInfo;

  class CFS_Sensor_Node
  {
  public:
    CFS_Sensor_Node (ros::NodeHandle & n, ros::NodeHandle & nhp);
    ~CFS_Sensor_Node (void);

    std::string cfs_device_name; //default 
    // ServiceServer$B$N(BCallback$B$K$h$k%-%c%j%V%l!<%7%g%s(B
    //bool start_calibration(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    bool start_calibration(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  private:
    ros::NodeHandle n_;
    ros::NodeHandle nhp_;
    ros::Publisher cfs_sensor_Pub_;
    ros::ServiceServer cfs_sensor_Svs_;
    ros::Publisher connection_state_pub_;

    SystemInfo gSys;
    bool update_flag;
    CFS_DEVICE_RATE_VAL cfs_device_rate_val;
    std::string cfs_frame_id;
    std::string cfs_default_device_name;
    std::string cfs_sensor_pub_name;
    std::string cfs_sensor_calib_srv_name;

    CFS_SENSOR_DATA *cfs_sensor_raw;  //sensor data before convert
    CFS_SENSOR_DATA *cfs_sensor_offset;  //sensor data offset
    CFS_SENSOR_DATA *cfs_sensor_conv; //sensor data after converted

    UCHAR CommRcvBuff[256];
    UCHAR CommSendBuff[1024];
    UCHAR SendBuff[512];

    //sensor data functions
    void Data_Init(void);
    void cfs_sensor_data_init(CFS_SENSOR_DATA *cfs_sensor_data);
    void convert_sensor_value(void);
    int  update_sensor_data(void);

    //ros functions
    void publish_sensor_msg(void);
    void publish_connection_state(int);
    void Node_Close(void);

    //serial communication with cfs sensor
    void Comm_Init(void);
    ULONG SendData(UCHAR *pucInput, USHORT usSize);
    void GetProductInfo(void);

    void SerialStart(void);
    void SerialStop(void);
  };
}
#endif
