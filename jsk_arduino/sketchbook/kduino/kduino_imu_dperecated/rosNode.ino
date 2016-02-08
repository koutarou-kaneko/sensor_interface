#ifdef USE_ROS
// Rosnode.h
#include <ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Joy.h>
#include <kduino/KduinoImu.h>
#include <ros/time.h>
#define ARMING_COUNT 50

// etc
unsigned long ros_loop_cnt;
unsigned int arming_on_cnt;
unsigned int arming_off_cnt;
int tmp_flag;

// nodeHandle
ros::NodeHandle nh;

// msg(publisher)

// new publisher
kduino::KduinoImu kduinoImu;
ros::Publisher pubkduinoImu("kduino/imu", &kduinoImu);

void cbCMD( const std_msgs::UInt16& cmd)                // callback func.
{
  switch (cmd.data) {
  case MSP_ACC_CALIBRATION:
    if(!f.ARMED) calibratingA=512;
    //     headSerialReply(0);
    break;
  case MSP_MAG_CALIBRATION:
    if(!f.ARMED) f.CALIBRATE_MAG = 1;
    //     headSerialReply(0);
    break;
  }
}
ros::Subscriber<std_msgs::UInt16> subCMD("kduino/msp_cmd", &cbCMD );

// srv(server)

// srv(client)

// setup for ROS
void ros_setup()
{

  pinMode(31, OUTPUT); //30, 31
  //digitalWrite(31, HIGH);   // blink the led
  // initialize
  ros_loop_cnt = 0;
  arming_on_cnt = 0;
  arming_off_cnt = 0;
  tmp_flag = 0;

  // node initialize
  nh.getHardware()->setBaud(ROS_BAUD);
  nh.initNode();

  nh.advertise(pubkduinoImu);

  // msg(subcriber)
  nh.subscribe(subCMD);
 
}

// loop func for ROS
void ros_loop()
{
 
  // input
  // msg(subcriber)

  // msg(publisher)
  //30-> 10Hz, 3 -> 100Hz
  if ((ros_loop_cnt%3)==1) 
    {
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
      kduinoImu.angle[2] = (float)att.heading;
      kduinoImu.altitude = alt.EstAlt;

      int debug;
      debug = pubkduinoImu.publish(&kduinoImu);

    }

  //debug
  int debug;
  debug = nh.spinOnce();  
  //if(debug == -5)
  //  digitalWrite(13, HIGH-digitalRead(13));   // blink the led  
  ros_loop_cnt++;
}
#endif// USE_ROS




