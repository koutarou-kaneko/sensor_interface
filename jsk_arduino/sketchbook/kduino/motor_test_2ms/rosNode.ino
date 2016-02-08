#ifdef USE_ROS
// Rosnode.h
#include <ros.h>

// msg

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>


void motor1Cmd( const std_msgs::Int32& cmd)		// callback func.
{
  motor[0] = cmd.data;
}
ros::Subscriber<std_msgs::Int32> sub1Cmd("kduino/motor1_cmd", &motor1Cmd );

void motor2Cmd( const std_msgs::Int32& cmd)		// callback func.
{
  motor[1] = cmd.data;
}
ros::Subscriber<std_msgs::Int32> sub2Cmd("kduino/motor2_cmd", &motor2Cmd );

void motor3Cmd( const std_msgs::Int32& cmd)		// callback func.
{
  motor[2] = cmd.data;
}
ros::Subscriber<std_msgs::Int32> sub3Cmd("kduino/motor3_cmd", &motor3Cmd );

void motor4Cmd( const std_msgs::Int32& cmd)		// callback func.
{
  motor[3] = cmd.data;
}
ros::Subscriber<std_msgs::Int32> sub4Cmd("kduino/motor4_cmd", &motor4Cmd );


// nodeHandle
ros::NodeHandle nh;

// setup for ROS
void ros_setup()
{
  nh.getHardware()->setBaud(ROS_BAUD);
  nh.initNode();
  nh.subscribe(sub1Cmd);
  nh.subscribe(sub2Cmd);
  nh.subscribe(sub3Cmd);
  nh.subscribe(sub4Cmd);
}

// loop func for ROS
void ros_loop()
{

  int debug;
  debug = nh.spinOnce();  
}
#endif// USE_ROS




