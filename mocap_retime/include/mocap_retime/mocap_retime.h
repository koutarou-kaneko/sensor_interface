#ifndef MOCAP_RETIME_H
#define MOCAP_RETIME_H

#include <ros/ros.h>
#include <jsk_quadcopter/MirrorModuleDebug.h>
#include <jsk_quadcopter/ImuDebug.h>
#include <jsk_quadcopter/SlamDebug.h>
#include <jsk_quadcopter/digital_filter.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf/transform_broadcaster.h"

class MocapRetime{
 public : 
  MocapRetime(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~MocapRetime();

  static const int TIME_SYNC_CALIB_COUNT = 10;

 protected:
  ros::NodeHandle nodeHandle_;
  ros::NodeHandle nodeHandlePrivate_;

  ros::Subscriber poseSub_;
  ros::Subscriber qaudcopterSub_;
  ros::Publisher poseStampedPub_;

  //deprecated
  ros::Subscriber poseSimSub_;

  bool retimeFlag_;
  bool simFlag_;
  IirFilter filterX_,filterY_,filterZ_, filterPsi_;
  IirFilter filterAccX_,filterAccY_,filterAccZ_;
  double rxFreq_;
  double cutoffPosFreq_;
  double cutoffVelFreq_;
  std::string pubName_;

  long offset, secOffset, nSecOffset;
  int timeSyncCount;
  float posXOffset, posYOffset, posZOffset;

  void poseCallback(const geometry_msgs::PoseConstPtr & msg);
  void quadcopterCallback(const jsk_quadcopter::ImuDebugConstPtr & msg);

  //deprecated
  void poseSimCallback(const jsk_quadcopter::SlamDebugConstPtr & msg);

};
#endif
