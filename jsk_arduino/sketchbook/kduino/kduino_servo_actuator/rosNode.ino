// START rosNode.ino
#ifdef USE_ROS

#define ROS_PUBLISH
#define ROS_PUBLISH_ROSSERVO
#define ROS_PUBLISH_KDUINO

#define ROS_NAMESPACE "kduino/"

// rosNode.h
#if 0
#include <ros.h>
#else
#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"
#include "ArduinoHardwareEx.h"

namespace ros
{
  typedef NodeHandle_<ArduinoHardware> NodeHandle;
}
#endif// _ROS_H_
#endif

// -----------------------------------------------------------------------------
// msg
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <kduino/KduinoImu.h>

// -----------------------------------------------------------------------------
// etc
unsigned long rosLoopCount;
bool  servoSetFlag;

// -----------------------------------------------------------------------------
// nodeHandle
ros::NodeHandle nh;

// -----------------------------------------------------------------------------
static int16_t  RosServo[NUMBER_SERVO];
static int16_t  RosServoSpeedMax[NUMBER_SERVO];
static uint16_t RosServoCnt[NUMBER_SERVO];

// -----------------------------------------------------------------------------
// msg(publisher)
#ifdef ROS_PUBLISH_ROSSERVO
std_msgs::Int16MultiArray rosServo;
ros::Publisher pubServo(ROS_NAMESPACE "ServoState", &rosServo);
#endif// ROS_PUBLISH_ROSSERVO

#ifdef ROS_PUBLISH_KDUINO
kduino::KduinoImu kduinoImu;
ros::Publisher pubkduinoImu(ROS_NAMESPACE "imu", &kduinoImu);
#endif// ROS_PUBLISH_KDUINO

// -----------------------------------------------------------------------------
// msg(subscriber)

void cbCommand( const std_msgs::UInt16& cmd) // callback func.
{
  switch (cmd.data) {
  case MSP_ACC_CALIBRATION:
    if(!f.ARMED) {
      calibratingA=512;
    }
    break;
  case MSP_MAG_CALIBRATION:
    if(!f.ARMED) {
      f.CALIBRATE_MAG = 1;
    }
    break;
  }
}
ros::Subscriber<std_msgs::UInt16> subCommand(ROS_NAMESPACE "msp_cmd", &cbCommand );

void cbServoSet( const std_msgs::Int16MultiArray& data) // callback func.
{
  //+*+*+* this is an example, which show the method to subscribe servo command from 
  for (int i=0;i<NUMBER_SERVO;i++) {
    if (i<data.data_length) 
      {
        RosServo[i] = data.data[i];
        servoSetFlag = true;
      }
    else 
      servoSetFlag = false;
  }
}
ros::Subscriber<std_msgs::Int16MultiArray> subServoSet(ROS_NAMESPACE "servo_set", &cbServoSet );

void cbServoSetSpeedMax( const std_msgs::Int16MultiArray& data) // callback func.
{
  for (int i=0;i<data.data_length;i++) {
     RosServoSpeedMax[i] = data.data[i];
  }
}
ros::Subscriber<std_msgs::Int16MultiArray> subServoSetSpeedMax(ROS_NAMESPACE "servo_set_speed_max", &cbServoSetSpeedMax );


// setup for ROS
void ros_setup()
{
  // initialize
  rosLoopCount = 0;
  for (int i=0;i<NUMBER_SERVO;i++) {
    RosServo[i] = INISERVO;
    RosServoCnt[i] = 0;
    RosServoSpeedMax[i] = 10;
  }

  // node initialize
  nh.getHardware()->setPort(ROS_PORT);
  nh.getHardware()->setBaud(ROS_BAUD);
  nh.initNode();


#ifdef ROS_PUBLISH_ROSSERVO
  nh.advertise(pubServo);  
  rosServo.data_length = NUMBER_SERVO;  
  rosServo.data = servo;
#endif// ROS_PUBLISH_ROSSERVO
#ifdef ROS_PUBLISH_KDUINO
  nh.advertise(pubkduinoImu);
#endif// ROS_PUBLISH_KDUINO

  // msg(subcriber)
  nh.subscribe(subCommand);
  nh.subscribe(subServoSet);
  nh.subscribe(subServoSetSpeedMax);
}

// loop func for ROS
void ros_loop()
{
  static uint32_t rosTime = 0;
  if (!nh.connected()) {
    // check once and exit
    nh.spinOnce();
    return;
  }

  if (currentTime > rosTime ) 
    {
      rosTime = currentTime + 20000;  // 50Hz

      //for servo command set
      if (servoSetFlag) 
        {
          for (int i=0;i<NUMBER_SERVO;i++) 
            servo[i] = RosServo[i];
          servoSetFlag = false;
        }
    }

#ifdef ROS_PUBLISH_ROSSERVO
    if ((rosLoopCount%10)==1) {
      pubServo.publish(&rosServo);
    } 
#endif// ROS_PUBLISH_ROSSERVO


#ifdef ROS_PUBLISH_KDUINO
    if ((rosLoopCount%10)==0) {
      // pubKawasakiImu.publish(&kawasakiImu);   
      //new version
      //time stamp
      kduinoImu.header.stamp = nh.now();
      //acceleration
      kduinoImu.accData[0] = -imu.accSmooth[1];
      kduinoImu.accData[1] =  imu.accSmooth[0];
      kduinoImu.accData[2] =  imu.accSmooth[2];

      kduinoImu.gyroData[0] =-imu.gyroData[1];
      kduinoImu.gyroData[1] = imu.gyroData[0];
      kduinoImu.gyroData[2] = imu.gyroData[2];

      //angles
      kduinoImu.angle[0] = (float)0.1*att.angle[0];
      kduinoImu.angle[1] = (float)0.1*att.angle[1];
      kduinoImu.angle[2] = - (float)att.heading; //yaw is adverse
      kduinoImu.altitude = alt.EstAlt;

      int debug;
      debug = pubkduinoImu.publish(&kduinoImu);
    }
#endif// ROS_PUBLISH_KDUINO

    nh.spinOnce();
    rosLoopCount++;
}
#endif// USE_ROS

// END rosNode.ino



