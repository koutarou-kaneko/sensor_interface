// START rosNode.ino
#ifdef USE_ROS

#define ROS_PUBLISH
#define ROS_PUBLISH_ROSRC
#define ROS_PUBLISH_ROSSERVO
#define ROS_PUBLISH_ROSMOTOR
//#define ROS_PUBLISH_ROSIMU
//#define ROS_PUBLISH_ROSSENSOR
#define ROS_PUBLISH_KDUINO
//#define USE_QUATERNION
#define ROS_LOG(s)  rosLog.data = s;pubLog.publish( &rosLog );

#define ROS_NAMESPACE "kduino/"
//#define ROS_NAMESPACE "JSK_IMU/"
//#define ROS_NAMESPACE "SKN_IMU/"
//#define ROS_NAMESPACE

// rosNode.h
//#include "config.h"
#if 0
#include <ros.h>
#else
#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"
#include "ArduinoHardwareEx.h"
#ifdef ROS_PUBLISH_ROSIMU
#include "Quaternion.h"
#endif// ROS_PUBLISH_ROSIMU

namespace ros
{
  typedef NodeHandle_<ArduinoHardwareEx> NodeHandle;
}
#endif// _ROS_H_
#endif

// -----------------------------------------------------------------------------
// msg
#ifdef ROS_PUBLISH_ROSIMU
#include <sensor_msgs/Imu.h>
#endif// ROS_PUBLISH_ROSIMU
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#ifdef ROS_PUBLISH_KDUINO
#include <kduino/KduinoImu.h>
#endif// ROS_PUBLISH_KDUINO
//#include <rosserial_arduino/KawasakiImu.h>

// -----------------------------------------------------------------------------
// etc
unsigned long rosLoopCount;

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

// -----------------------------------------------------------------------------
// msg(publisher)
std_msgs::String rosLog;
ros::Publisher pubLog(ROS_NAMESPACE "Log", &rosLog);

#ifdef ROS_PUBLISH_ROSRC
std_msgs::Int16MultiArray rosRc;
ros::Publisher pubRc(ROS_NAMESPACE "RcState", &rosRc);
#endif// ROS_PUBLISH_ROSRC

#ifdef ROS_PUBLISH_ROSSERVO
std_msgs::Int16MultiArray rosServo;
ros::Publisher pubServo(ROS_NAMESPACE "ServoState", &rosServo);
#endif// ROS_PUBLISH_ROSSERVO

#ifdef ROS_PUBLISH_ROSMOTOR
std_msgs::Int16MultiArray rosMotor;
ros::Publisher pubMotor(ROS_NAMESPACE "MotorState", &rosMotor);
#endif// ROS_PUBLISH_ROSMOTOR

#ifdef ROS_PUBLISH_ROSIMU
sensor_msgs::Imu rosImu;
ros::Publisher pubImu(ROS_NAMESPACE "Imu", &rosImu);
#endif// ROS_PUBLISH_ROSIMU

#ifdef ROS_PUBLISH_ROSSENSOR
std_msgs::Int32MultiArray rosSensor;
ros::Publisher pubSensor(ROS_NAMESPACE "SensorState", &rosSensor);
#define NUMBER_SENSOR 6
int32_t sensor_data[NUMBER_SENSOR];
#endif// ROS_PUBLISH_ROSSENSOR

#ifdef ROS_PUBLISH_KDUINO
// rosserial_arduino::KawasakiImu kawasakiImu;
// ros::Publisher pubKawasakiImu("kawasaki/imu", &kawasakiImu);
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
      ROS_LOG("RxCommand_MSP_ACC_CALIBRATION");
      calibratingA=512;
    }else {
      ROS_LOG("ERROR:RxCommand_MSP_ACC_CALIBRATION");
    }
    break;
  case MSP_MAG_CALIBRATION:
    if(!f.ARMED) {
      ROS_LOG("RxCommand_MSP_MAG_CALIBRATION");
      f.CALIBRATE_MAG = 1;
    }else {
      ROS_LOG("ERROR:RxCommand_MSP_MAG_CALIBRATION");
    }
    break;
  }
  ROS_LOG("RxCommand");
}
ros::Subscriber<std_msgs::UInt16> subCommand(ROS_NAMESPACE "Command", &cbCommand );

void cbServoSet( const std_msgs::Int16MultiArray& data) // callback func.
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
  ROS_LOG("RxServoSet");
}
ros::Subscriber<std_msgs::Int16MultiArray> subServoSet(ROS_NAMESPACE "ServoSet", &cbServoSet );

void cbServoSetSpeedMax( const std_msgs::Int16MultiArray& data) // callback func.
{
  for (int i=0;i<data.data_length;i++) {
     RosServoSpeedMax[i] = data.data[i];
  }
  ROS_LOG("RxServoSetSpeedMax");
}
ros::Subscriber<std_msgs::Int16MultiArray> subServoSetSpeedMax(ROS_NAMESPACE "ServoSetSpeedMax", &cbServoSetSpeedMax );

void cbMotorSet( const std_msgs::Int16MultiArray& data) // callback func.
{
  for (int i=0;i<NUMBER_MOTOR;i++) {
    if (i<data.data_length) {
      RosMotor[i] = data.data[i];
      RosMotorCnt[i] = 10000;
    }
    else {
      RosMotorCnt[i] = 0;
    }
  }
  ROS_LOG("RxMotorSet");
}
ros::Subscriber<std_msgs::Int16MultiArray> subMotorSet(ROS_NAMESPACE "MotorSet", &cbMotorSet );

void cbMotorSetSpeedMax( const std_msgs::Int16MultiArray& data) // callback func.
{
  for (int i=0;i<data.data_length;i++) {
    RosMotorSpeedMax[i] = data.data[i];
  }
  ROS_LOG("RxMotorSetSpeedMax");
}
ros::Subscriber<std_msgs::Int16MultiArray> subMotorSetSpeedMax(ROS_NAMESPACE "MotorSetSpeedMax", &cbMotorSetSpeedMax );


void cbAcc1G( const std_msgs::Int16MultiArray& data) // callback func.
{
  if (data.data_length<3) {
    ROS_LOG("ERROR: RxAcc1G");
    return;
  }
  for (int i=0;i<data.data_length;i++) {
    acc1G[i] = data.data[i];
  }
  AXIS_CONV_ACC_R2M(acc1G);
  ROS_LOG("RxAcc1G");
}
ros::Subscriber<std_msgs::Int16MultiArray> subAcc1G(ROS_NAMESPACE "Acc1G", &cbAcc1G );

// srv(server)
// srv(client)

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
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
  for (int i=0;i<NUMBER_MOTOR;i++) {
    RosMotor[i] = INIMOTOR;
    RosMotorCnt[i] = 0;
    RosMotorSpeedMax[i] = 10;
  }

  // node initialize
  nh.getHardware()->setPort(ROS_PORT);
  nh.getHardware()->setBaud(ROS_BAUD);
  nh.initNode();

  // msg(publisher)
  nh.advertise(pubLog);
#ifdef ROS_PUBLISH_ROSIMU
  nh.advertise(pubImu);
#endif// ROS_PUBLISH_ROSIMU
#ifdef ROS_PUBLISH_ROSMOTOR
  nh.advertise(pubMotor);  
  rosMotor.data_length = NUMBER_MOTOR;  
  rosMotor.data = motor;
#endif// ROS_PUBLISH_ROSMOTOR
#ifdef ROS_PUBLISH_ROSSERVO
  nh.advertise(pubServo);  
  rosServo.data_length = NUMBER_SERVO;  
  rosServo.data = servo;
#endif// ROS_PUBLISH_ROSSERVO
#ifdef ROS_PUBLISH_ROSSENSOR
  nh.advertise(pubSensor);  
  rosSensor.data_length = NUMBER_SENSOR;  
  rosSensor.data = sensor_data;
#endif// ROS_PUBLISH_ROSSENSOR
#ifdef ROS_PUBLISH_KDUINO
  nh.advertise(pubkduinoImu);
#endif// ROS_PUBLISH_KDUINO

  // msg(subcriber)
  nh.subscribe(subCommand);
  nh.subscribe(subServoSet);
  nh.subscribe(subMotorSet);
  nh.subscribe(subServoSetSpeedMax);
  nh.subscribe(subMotorSetSpeedMax);
  nh.subscribe(subAcc1G);

  // srv(server)

  // srv(client)

#if 0
  // wait for connect(need for client?)
  while(!nh.connected()) nh.spinOnce();
  nh.loginfo("Startup complete");
#endif
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
//  rosTime = currentTime + 100000;  // 10Hz
//    rosTime = currentTime + 50000;  // 20Hz
  rosTime = currentTime + 20000;  // 50Hz
//  rosTime = currentTime + 10000;  // 100Hz

    // input
    // msg(subcriber)
    for (int i=0;i<NUMBER_MOTOR;i++) {
      if (RosMotorCnt[i]>0) {
#if 1
        if (motor[i]<RosMotor[i]) {
          motor[i] += RosMotorSpeedMax[i];
          if (motor[i]>RosMotor[i]) {
            motor[i] = RosMotor[i];
          }
        }else {
          motor[i] -= RosMotorSpeedMax[i];
          if (motor[i]<RosMotor[i]) {
            motor[i] = RosMotor[i];
          }
        }
#else
        motor[i] = RosMotor[i];
#endif       
        RosMotorCnt[i]--;
      }else {
	// FailSafe
      }
    }
    for (int i=0;i<NUMBER_SERVO;i++) {
      if (RosServoCnt[i]>0) {
#if 1
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
#else
        servo[i] = RosServo[i];
#endif
        RosServoCnt[i]--;
      }else {
	// FailSafe
      }
    }

    // srv(client)

    // output
    // srv(server)
    // msg(publisher)
#ifdef ROS_PUBLISH   

#ifdef ROS_PUBLISH_ROSMOTOR
    if ((rosLoopCount%4)==0) {
      pubMotor.publish(&rosMotor);
    }
#endif// ROS_PUBLISH_ROSMOTOR

#ifdef ROS_PUBLISH_ROSSERVO
    if ((rosLoopCount%4)==1) {
      pubServo.publish(&rosServo);
    } 
#endif// ROS_PUBLISH_ROSSERVO

#ifdef ROS_PUBLISH_ROSSENSOR
  if ((rosLoopCount%4)==2) {
    int32_t* pdata;
    sensor_data[0] = EstM32.V.X;     
    sensor_data[1] = EstM32.V.Y;     
    sensor_data[2] = EstM32.V.Z;
    pdata=&(sensor_data[0]);AXIS_CONV_MAG_M2R(pdata);
    sensor_data[3] = EstG32.V.X;     
    sensor_data[4] = EstG32.V.Y;     
    sensor_data[5] = EstG32.V.Z;
    pdata=&(sensor_data[3]);AXIS_CONV_ACC_M2R(pdata);
#if 0
    sensor_data[6] = alt.EstAlt;
    sensor_data[7] = imu.gyroData[1];
    sensor_data[8] = imu.gyroData[0];
    sensor_data[9] = imu.gyroData[2];
    pdata=&(sensor_data[7]);AXIS_CONV_GYRO_M2R(pdata);
    sensor_data[10] =  imu.accSmooth[0];
    sensor_data[11] = -imu.accSmooth[1];
    sensor_data[12] =  imu.accSmooth[2];
    pdata=&(sensor_data[10]);AXIS_CONV_ACC_M2R(pdata);
#endif
    pubSensor.publish(&rosSensor);
  }
#endif// ROS_PUBLISH_ROSSENSOR

#ifdef ROS_PUBLISH_ROSIMU
    if ((rosLoopCount%4)==3) {
      // *** need unit conversion!! ***
      float q[4];
      float vx[3],vy[3],vz[3];
      vx[0] = EstM32.V.X;     
      vx[1] = EstM32.V.Y;     
      vx[2] = EstM32.V.Z;
      vz[0] = EstG32.V.X;     
      vz[1] = EstG32.V.Y;     
      vz[2] = EstG32.V.Z;
#if 0
      outerProduct(vy, vz, vx);
      outerProduct(vx, vy, vz);
      unitVector(vx,vx);
      unitVector(vy,vy);
      unitVector(vz,vz);
      transformRotMatToQuaternion(q, vx, vy, vz);
#endif
      rosImu.header.frame_id =  "/Imu";
      rosImu.header.stamp = nh.now();
      rosImu.orientation.x = q[0];
      rosImu.orientation.y = q[1];
      rosImu.orientation.z = q[2];
      rosImu.orientation.w = q[3];
      rosImu.angular_velocity.x = imu.gyroData[1]*1.0;
      rosImu.angular_velocity.y = imu.gyroData[0]*1.0;
      rosImu.angular_velocity.z = imu.gyroData[2]*1.0;
      rosImu.linear_acceleration.x = imu.accSmooth[0]*9.8/512;
      rosImu.linear_acceleration.y = -imu.accSmooth[1]*9.8/512;
      rosImu.linear_acceleration.z = imu.accSmooth[2]*9.8/512;
      for (int i=0;i<9;i++) {
        rosImu.orientation_covariance[i] = 0.0;
        rosImu.angular_velocity_covariance[i] = 0.0;
        rosImu.linear_acceleration_covariance[i] = 0.0;
      }
      for (int i=0;i<3;i++) {
        rosImu.orientation_covariance[i*4] = 1.0;
        rosImu.angular_velocity_covariance[i*4] = 1.0;
        rosImu.linear_acceleration_covariance[i*4] = 1.0;
      }
      pubImu.publish(&rosImu);
      ROS_LOG("PUB_IMU");
    }
#endif// ROS_PUBLISH_ROSIMU

#ifdef ROS_PUBLISH_KDUINO
    if ((rosLoopCount%4)==3) {
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
    }
#endif// ROS_PUBLISH_KDUINO
#endif// ROS_PUBLISH

    //  if ((rosLoopCount%50)==0) {
    if ((rosLoopCount%200)==0) {
      rosLog.data = "ROS:Loop";    
      pubLog.publish( &rosLog );
    }
    
    rosLoopCount++;
  }else {
	// non-cycelic process
  }

  // 
  nh.spinOnce();
}
#endif// USE_ROS

// END rosNode.ino



