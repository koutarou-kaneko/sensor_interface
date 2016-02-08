#ifdef USE_ROS
// Rosnode.h
#include <ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <kduino/KduinoImu.h>
#include <ros/time.h>
#define ARMING_COUNT 50

// etc
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

void cbSetBaroGroundPressure(const std_msgs::Float32& cmd)
{
  baroGroundPressure = (int32_t)cmd.data;
}
ros::Subscriber<std_msgs::Float32> subSetBaroGroundPressure(
    "kduino/set_baro_ground_pressure", &cbSetBaroGroundPressure);

// srv(server)

// srv(client)

// setup for ROS
void ros_setup()
{

  pinMode(31, OUTPUT); //30, 31
  //digitalWrite(31, HIGH);   // blink the led
  // initialize
  arming_on_cnt = 0;
  arming_off_cnt = 0;
  tmp_flag = 0;

  // node initialize
  nh.getHardware()->setBaud(ROS_BAUD);
  nh.initNode();

  nh.advertise(pubkduinoImu);

  // msg(subcriber)
  nh.subscribe(subCMD);
  nh.subscribe(subSetBaroGroundPressure);
 
}

// loop func for ROS
void ros_loop()
{
  static uint8_t ros_loop_cnt = 0;
  int debug;
  
  //30-> 10Hz, 3 -> 100Hz
  if (ros_loop_cnt >= 3) ros_loop_cnt = 0;
  
  switch (ros_loop_cnt) {
    case 0:
      // input
      // msg(subcriber)
      
      // msg(publisher)

      //time stamp
      kduinoImu.header.stamp = nh.now();
      
      kduinoImu.accData[0] =  imu.accSmooth[0];
      kduinoImu.accData[1] =  imu.accSmooth[1];
      kduinoImu.accData[2] =  imu.accSmooth[2];

      kduinoImu.gyroData[0] = imu.gyroData[0];
      kduinoImu.gyroData[1] = imu.gyroData[1];
      kduinoImu.gyroData[2] = imu.gyroData[2];

      kduinoImu.magData[0] = imu.magADC[0];
      kduinoImu.magData[1] = imu.magADC[1];
      kduinoImu.magData[2] = imu.magADC[2];

      //angles
      kduinoImu.angle[0] = (float)0.1*att.angle[0];
      kduinoImu.angle[1] = (float)0.1*att.angle[1];
      kduinoImu.angle[2] = (float)att.heading;

      kduinoImu.altitude = alt.EstAlt;
      kduinoImu.baroPressure = (float)baroPressureSum / (BARO_TAB_SIZE - 1);
      kduinoImu.baroTemperature = baroTemperature / 100;
      break;
      
    case 1:
      //debug
      debug = pubkduinoImu.publish(&kduinoImu);
      break;

    case 2:
      //debug
      debug = nh.spinOnce();
      break;
    }
  ros_loop_cnt++;
}
#endif// USE_ROS
