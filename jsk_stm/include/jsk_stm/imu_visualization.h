#ifndef IMU_VISUALIZATION_H
#define IMU_VISUALIZATION_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <spinal/Imu.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

class ImuVisualization{
 public :

  ImuVisualization(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~ImuVisualization();


 protected:
  ros::NodeHandle imuVisualizationNodeHandle_;
  ros::NodeHandle imuVisualizationNodeHandlePrivate_;
  ros::Subscriber imu_data_sub;
  ros::Publisher  imu_visualization_pub;
  ros::Publisher  imu_pub, mag_pub;

  tf::TransformBroadcaster* tfB;

  ros::Timer visualTimer;
  ros::Time imuDataStamp;

  double visualLoopRate_;

  bool visualization_;
  std::string preffix_;
  std::string imuFrame_;  /// frame id for IMU
  std::string imuOtherFrame_;  /// frame id for sensor_msgs::Imu
  std::string baseFrame_;  /// parent id of IMU
  double trans_roll_,trans_pitch_,trans_yaw_;
  tf::Matrix3x3 matrix_trans_;

  float position_x;
  float position_y;
  float position_z;
  //geometry_msgs::Quaternion q;
  tf::Quaternion q;

  void rosParamInit(ros::NodeHandle& nh);
  void visualFunction(const ros::TimerEvent & e);
  void imuCallback(const spinal::ImuConstPtr& imu_msg);

  inline void setOrientation(
      geometry_msgs::Pose& pose,
      double pos_x, double pos_y, double pos_z,
      double rot_x, double rot_y, double rot_z, double rot_w) {
    pose.position.x = pos_x;
    pose.position.y = pos_y;
    pose.position.z = pos_z;
    pose.orientation.x = rot_x;
    pose.orientation.y = rot_y;
    pose.orientation.z = rot_z;
    pose.orientation.w = rot_w;
  }
  inline void setScale(geometry_msgs::Vector3& scale,
                       double sx, double sy, double sz) {
    scale.x = sx;
    scale.y = sy;
    scale.z = sz;
  }
  inline void setRGBA(std_msgs::ColorRGBA& color,
                      double r, double g, double b, double a) {
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
  }

};

#endif

