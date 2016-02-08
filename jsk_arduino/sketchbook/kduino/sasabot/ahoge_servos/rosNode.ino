// START rosNode.ino
#ifdef USE_ROS

#define ROS_PUBLISH
//#define ROS_PUBLISH_ROSSERVO
//#define ROS_PUBLISH_KDUINO

#define ROS_NAMESPACE "kduino/"

// rosNode.h
#if 0
#include <ros.h>
#else
#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"
#include "ArduinoHardwareEx.h"
#ifdef ROS_PUBLISH_ROSIMU

#endif// ROS_PUBLISH_ROSIMU

namespace ros
{
  typedef NodeHandle_<ArduinoHardwareEx> NodeHandle;
}
#endif// _ROS_H_
#endif

// -----------------------------------------------------------------------------
// msg
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <kduino/KduinoImu.h>
#include <kduino/KduinoServo.h>

//#define UPPER_BODY  0
//#define BOTTOM_BODY  1

// -----------------------------------------------------------------------------
// etc
unsigned long rosLoopCount;
bool test_flag;
int servo_cnt;
// -----------------------------------------------------------------------------
// nodeHandle
ros::NodeHandle nh;

// -----------------------------------------------------------------------------
static int16_t  RosRc[RC_CHANS];
static uint16_t RosRcCnt=0;
static int16_t  RosServo[NUMBER_SERVO];
static int16_t  RosServoSpeedMax[NUMBER_SERVO];
static uint16_t RosServoCnt[NUMBER_SERVO];
static int16_t  RosMotor[NUMBER_SERVO];
static int16_t  RosMotorSpeedMax[NUMBER_SERVO];
static uint16_t RosMotorCnt[NUMBER_SERVO];


kduino::KduinoImu kduinoImu;
ros::Publisher pubkduinoImu(ROS_NAMESPACE "imu", &kduinoImu);


void cbServoTest( const std_msgs::Empty&cmd) 
{
  test_flag =true;
}
ros::Subscriber<std_msgs::Empty> subServoTest("servo_test", &cbServoTest );

void cbCommand( const std_msgs::UInt16& cmd) // callback func.
{
  switch (cmd.data) {
  case MSP_ACC_CALIBRATION:
    if(!f.ARMED) 
      calibratingA=512;
    break;
  case MSP_MAG_CALIBRATION:
    if(!f.ARMED) 
      f.CALIBRATE_MAG = 1;
    break;
  }
}
ros::Subscriber<std_msgs::UInt16> subCommand(ROS_NAMESPACE "command", &cbCommand );

void cbServoSet( const std_msgs::Int16MultiArray& data) 
{
  for (int i=0;i<NUMBER_SERVO;i++) {
    if (i<data.data_length) {
      RosServo[i] = data.data[i];
      RosServoCnt[i] = 10000;
    }
    else {
      RosServoCnt[i] = 0;
    }
  }
}
ros::Subscriber<std_msgs::Int16MultiArray> subServoSet(ROS_NAMESPACE "servo_set", &cbServoSet);

void cbServoSetSpeedMax( const std_msgs::Int16MultiArray& data) // callback func.
{
  for (int i=0;i<data.data_length;i++) {
     RosServoSpeedMax[i] = data.data[i];
  }
}
ros::Subscriber<std_msgs::Int16MultiArray> subServoSetSpeedMax(ROS_NAMESPACE "servo_set_speed_max", &cbServoSetSpeedMax );


void cbUpperServoControl( const kduino::KduinoServo& servo_control_data)
{
  servo[servo_control_data.servo_id] = servo_control_data.servo_value;
}
ros::Subscriber<kduino::KduinoServo> subUpperServoControl(ROS_NAMESPACE "ahoge_servos_control", &cbUpperServoControl);

void cbBottomServoControl( const kduino::KduinoServo& servo_control_data) // callback func.
{
  servo[servo_control_data.servo_id] = servo_control_data.servo_value;
}
ros::Subscriber<kduino::KduinoServo> subBottomServoControl(ROS_NAMESPACE "eyes_servos_control", &cbBottomServoControl);


// setup for ROS
void ros_setup()
{
  servo_cnt = 0;
  pinMode(13, OUTPUT);
  // initialize
  rosLoopCount = 0;
  test_flag = false;

  // node initialize
  nh.getHardware()->setPort(ROS_PORT);
  nh.getHardware()->setBaud(ROS_BAUD);
  nh.initNode();
  // msg(publisher)

  nh.advertise(pubkduinoImu);

  // msg(subcriber)
  //nh.subscribe(subServoTest);
  nh.subscribe(subCommand);
  nh.subscribe(subUpperServoControl);
  //nh.subscribe(subBottomServoControl);
  //nh.subscribe(subServoSet);
  //nh.subscribe(subServoSetSpeedMax);


  // servo[0] = 1000;
  // servo[1] = 1000;
  // servo[2] = 1000;
  // servo[3] = 1000;
  // servo[4] = 1000;
  // servo[5] = 1000;
  // servo[6] = 1000;
  // servo[7] = 1000;

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
  if (currentTime > rosTime ) {
    rosTime = currentTime + 1000000;  
    //  rosTime = currentTime + 50000;  // 20Hz
    //  rosTime = currentTime + 10000;  // 100Hz

    if(test_flag)
      {
        digitalWrite(13, HIGH-digitalRead(13));   // blink the led

        if(servo[0] == 1000)
          {
            for(int i=0;i<NUMBER_SERVO;i++)
              servo[i] = 2000;
          }
        else
          {
            for(int i=0;i<NUMBER_SERVO;i++)
              servo[i] = 1000;
          }
      }

#if 0
    for (int i=0;i<NUMBER_SERVO;i++) {
      if (RosServoCnt[i]>0) {
        if (servo[i]<RosServo[i]) {
          servo[i] += RosServoSpeedMax[i];
          if (servo[i]>RosServo[i]) {
            servo[i] = RosServo[i];
          }
        }else {
          servo[i] -= RosServoSpeedMax[i];
          if (servo[i]<RosServo[i]) {
            servo[i] = RosServo[i];
          }
        }
        RosServoCnt[i]--;
      }else {
      }
    }
#endif


    // pubKawasakiImu.publish(&kawasakiImu);   
    //new version
    //time stamp
    kduinoImu.header.stamp = nh.now();
    //acceleration
    kduinoImu.accData[0] = -imu.accSmooth[1];
    kduinoImu.accData[1] =  imu.accSmooth[0];
    kduinoImu.accData[2] =  imu.accSmooth[2];

    // gyro
    kduinoImu.gyroData[0] =-imu.gyroData[1];
    kduinoImu.gyroData[1] = imu.gyroData[0];
    kduinoImu.gyroData[2] = imu.gyroData[2];

    //angles
    kduinoImu.angle[0] = (float)0.1*att.angle[0];
    kduinoImu.angle[1] = (float)0.1*att.angle[1];
    kduinoImu.angle[2] = (float)att.heading;
    kduinoImu.altitude = alt.EstAlt;

    int debug;
    debug = pubkduinoImu.publish(&kduinoImu);

    rosLoopCount++;
  }else {
	// non-cycelic process
  }


  //delay(1000);
  //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  nh.spinOnce();
}
#endif// USE_ROS

// END rosNode.ino



