/// @file imu_visualization.cpp
/// @brief visualize and convert scale for KduinoImu
///
/// @par Publish:
/// - orientation_data (visualization_msgs::MarkerArray)
/// - imu/data_raw (sensor_msgs::Imu)
/// - imu/mag (geometry_msgs::Vector3Stamped)
/// @par Subscribe:
/// - kduino/imu (kduino::KduinoImu)
/// @par Params:
/// - visualization
///     - if false, timer thread does not run (default true)
/// - visualLoopRate
///     - set running rate [Hz] (default 20)
/// - baseFrame
///     - frame ID of base link (default "map")
/// - imuFrame
///     - frame ID of IMU (default "kduino_frame")
///     - if preffix is not empty, finally it is set as ${preffix}/${imuFrame}
/// - imuOtherFrame
///     - frame ID for sensor_msgs::Imu (default "other_frame")
///     - if preffix is not empty, finally it is set as ${preffix}/${imuOtherFrame}
/// - trans_roll
///     - offset of roll (default 0)
/// - trans_pitch
///     - offset of pitch (default 0)
/// - trans_yaw
///     - offset of yaw (default 0)
///

#include "jsk_stm/imu_visualization.h"


ImuVisualization::ImuVisualization(ros::NodeHandle& nh,
                                   ros::NodeHandle& nh_private)
    : imuVisualizationNodeHandle_(nh),
      imuVisualizationNodeHandlePrivate_(nh_private)
{
  //ros param init
  rosParamInit(imuVisualizationNodeHandlePrivate_);

  imu_data_sub
      = imuVisualizationNodeHandle_.subscribe(
          "imu",
          5,
          &ImuVisualization::imuCallback,
          this);

  imu_visualization_pub
      = imuVisualizationNodeHandle_.advertise<visualization_msgs::MarkerArray>(
          "orientation_data", 5);

  imu_pub
      = imuVisualizationNodeHandle_.advertise<sensor_msgs::Imu>(
          "imu/data_raw", 5);

  mag_pub
      = imuVisualizationNodeHandle_.advertise<geometry_msgs::Vector3Stamped>(
          "imu/mag", 5);


  //timer init
  if (visualization_) {
    visualTimer =
        imuVisualizationNodeHandlePrivate_.createTimer(
            ros::Duration(1.0 /  visualLoopRate_),
            &ImuVisualization::visualFunction,
            this);
  }

  //tf publisher init
  tfB = new tf::TransformBroadcaster();

}


/// @brief destructor
ImuVisualization::~ImuVisualization()
{
  delete tfB;
}


/// @brief set params from rosparam
/// @param nh node handle
void ImuVisualization::rosParamInit(ros::NodeHandle& nh)
{
  if (!nh.getParam ("preffix", preffix_))
    preffix_ = "imu";
  printf(" preffix is %s\n", preffix_.c_str());

  if (!nh.getParam ("visualization", visualization_))
    visualization_ = true;
  printf(" visualization is %s\n", visualization_?("true"):("flase"));

  if (!nh.getParam ("visualLoopRate", visualLoopRate_))
    visualLoopRate_ = 20;
  printf(" visualization loop rate is %.3f\n", visualLoopRate_);

  if (!nh.getParam ("baseFrame", baseFrame_))
    baseFrame_ = "map";
  printf(" base frame is %s\n", baseFrame_.c_str());

  if (!nh.getParam ("imuFrame", imuFrame_))
    imuFrame_ = "imu_frame";
  imuFrame_ = preffix_ + "/" + imuFrame_;
  printf(" imu frame is %s\n", imuFrame_.c_str());

  if (!nh.getParam ("imuOtherFrame", imuOtherFrame_))
    imuOtherFrame_ = "other_frame";
  imuOtherFrame_ = preffix_ + "/" + imuOtherFrame_;
  printf(" imu other frame is %s\n", imuOtherFrame_.c_str());

  if (!nh.getParam ("trans_roll", trans_roll_))
    trans_roll_ = 0;
  printf(" transform roll  is %.3f\n", trans_roll_);

  if (!nh.getParam ("trans_pitch", trans_pitch_))
    trans_pitch_ = 0;
  printf(" transform pitch  is %.3f\n", trans_pitch_);

  if (!nh.getParam ("trans_yaw", trans_yaw_))
    trans_yaw_ = 0;
  printf(" transform yaw  is %.3f\n", trans_yaw_);

  matrix_trans_.setRPY(trans_roll_, trans_pitch_, trans_yaw_);

  float roll = 0;
  float pitch = 0;
  float yaw = 0;
  q.setRPY(tfScalar(roll), tfScalar(pitch), tfScalar(yaw));
}

/// @brief main routine
/// @param e ros::TimerEvent
void ImuVisualization::visualFunction(const ros::TimerEvent & e)
{

  ros::Time tfStamp = ros::Time::now();
  tf::Transform baseToImu;

  baseToImu.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  baseToImu.setRotation(q);
  tfB->sendTransform(
      tf::StampedTransform(baseToImu, tfStamp, baseFrame_, imuFrame_));

  //box publish
    visualization_msgs::MarkerArray IMU;
    visualization_msgs::Marker ImuBigBox;
    ImuBigBox.header.frame_id = imuFrame_;
    ImuBigBox.header.stamp = ros::Time::now();
    ImuBigBox.ns = "big_box";
    ImuBigBox.id = 0;
    ImuBigBox.type = visualization_msgs::Marker::CUBE;
    ImuBigBox.action = visualization_msgs::Marker::ADD;
    ImuBigBox.lifetime = ros::Duration();
    setOrientation(ImuBigBox.pose,
                   0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 1.0);
    setScale(ImuBigBox.scale, 1.0, 1.0, 0.1);
    setRGBA(ImuBigBox.color, 0.1, 0.1, 0.1, 1.0);
    IMU.markers.push_back(ImuBigBox);

    visualization_msgs::Marker ImuSmallCulmn;
    ImuSmallCulmn.header.frame_id = imuFrame_;
    ImuSmallCulmn.header.stamp = ros::Time::now();
    ImuSmallCulmn.ns = "small_coulmn";
    ImuSmallCulmn.id = 0;
    ImuSmallCulmn.type = visualization_msgs::Marker::CUBE;
    ImuSmallCulmn.action = visualization_msgs::Marker::ADD;
    ImuSmallCulmn.lifetime = ros::Duration();
    setOrientation(ImuSmallCulmn.pose,
                   0.45, -0.2, 0.02,
                   0.0, 0.0, 0.0, 1.0);
    setScale(ImuSmallCulmn.scale, 0.1, 0.3, 0.1);
    setRGBA(ImuSmallCulmn.color, 1.0, 1.0, 1.0, 1.0);
    IMU.markers.push_back(ImuSmallCulmn);

    visualization_msgs::Marker ImuSmallCulmn2;
    ImuSmallCulmn.header.frame_id = imuFrame_;
    ImuSmallCulmn.header.stamp = ros::Time::now();
    ImuSmallCulmn.ns = "small_coulmn2";
    ImuSmallCulmn.id = 0;
    ImuSmallCulmn.type = visualization_msgs::Marker::CUBE;
    ImuSmallCulmn.action = visualization_msgs::Marker::ADD;
    ImuSmallCulmn.lifetime = ros::Duration();
    setOrientation(ImuSmallCulmn.pose,
                   -0.45, 0.1, 0.02,
                   0.0, 0.0, 0.0, 1.0);
    setScale(ImuSmallCulmn.scale, 0.05, 0.15, 0.1);
    setRGBA(ImuSmallCulmn.color, 1.0, 1.0, 1.0, 1.0);
    IMU.markers.push_back(ImuSmallCulmn);


    imu_visualization_pub.publish(IMU);
}

/// @brief Imu message callback
/// @param imu_msgs aerial_robot_msgs::ImuConstPtr
void ImuVisualization::imuCallback(const aerial_robot_msgs::ImuConstPtr& imu_msg)
{
  imuDataStamp = imu_msg->stamp;

  tf::Transform baseToImu;

  // angle to radian
  tfScalar roll  =  imu_msg->angles[0];
  tfScalar pitch =  imu_msg->angles[1];
  tfScalar yaw   =  imu_msg->angles[2];

  q.setRPY(roll, pitch, yaw);
  baseToImu.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  baseToImu.setRotation(q);
  tfB->sendTransform(
      tf::StampedTransform(baseToImu, imuDataStamp, baseFrame_, imuFrame_));

  //sensor imu
  sensor_msgs::ImuPtr imu_data(new sensor_msgs::Imu);
  imu_data->header.stamp = imu_msg->stamp;
  imu_data->header.frame_id = imuOtherFrame_;
  imu_data->orientation.w = 1.0;

  tf::Vector3 gyro(imu_msg->gyro_data[0],
                   imu_msg->gyro_data[1],
                   imu_msg->gyro_data[2]);
  tf::Vector3 acc(imu_msg->acc_data[0],
                  imu_msg->acc_data[1],
                  imu_msg->acc_data[2]);

  tf::Vector3 gyro_transed = (matrix_trans_ * gyro);
  tf::Vector3 acc_transed = (matrix_trans_ * acc);

  imu_data->angular_velocity.x = gyro_transed.x();
  imu_data->angular_velocity.y = gyro_transed.y();
  imu_data->angular_velocity.z = gyro_transed.z();
  imu_data->linear_acceleration.x = acc_transed.x();
  imu_data->linear_acceleration.y = acc_transed.y();
  imu_data->linear_acceleration.z = acc_transed.z();

  imu_pub.publish(imu_data);

  geometry_msgs::Vector3StampedPtr mag_data(new geometry_msgs::Vector3Stamped);
  mag_data->header.stamp = imu_msg->stamp;

  tf::Vector3 mag(imu_msg->mag_data[0],
                  imu_msg->mag_data[1],
                  imu_msg->mag_data[2]);

  tf::Vector3 mag_transed = (matrix_trans_ * mag);
  mag_data->vector.x = mag_transed.x();
  mag_data->vector.y = mag_transed.y();
  mag_data->vector.z = mag_transed.z();

  mag_pub.publish(mag_data);

}

