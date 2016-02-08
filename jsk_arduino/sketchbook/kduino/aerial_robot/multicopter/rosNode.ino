#ifdef USE_ROS
// Rosnode.h
#include <ros.h>

// msg

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int8.h>

#include <aerial_robot_msgs/KduinoImu.h>
#include <aerial_robot_msgs/RcData.h> //baffur for virtual rc
#include <aerial_robot_msgs/AttitudePDGain.h>
#include <aerial_robot_msgs/AttGains.h>

#define ARMING_COUNT 50

// etc
unsigned long ros_loop_cnt;
unsigned int arming_on_cnt;
unsigned int arming_off_cnt;
unsigned int drop_flag;
unsigned int router_num;
unsigned long drop_cmd_time;
uint32_t drop_time;
aerial_robot_msgs::RcData rcSet; //buffer for virtual rc command

// nodeHandle
ros::NodeHandle nh;

// msg(publisher)
aerial_robot_msgs::KduinoImu kduinoImu;
ros::Publisher pubKduinoImu("kduino/imu", &kduinoImu);

std_msgs::Int8 armingAck;
ros::Publisher pubArmingAck("kduino/arming_ack", &armingAck);

aerial_robot_msgs::AttitudePDGain pdGainRes;
ros::Publisher pubPDGain("kduino/pd_gain", &pdGainRes);

// msg(subscriber)
void cbRc( const aerial_robot_msgs::RcData &cmd_msg) 
{
  rcSet.roll     = MIDRC + cmd_msg.roll;
  rcSet.pitch    = MIDRC + cmd_msg.pitch;
  rcSet.yaw      = MIDRC + cmd_msg.yaw;
  rcSet.throttle = cmd_msg.throttle; 

  if((rcSet.yaw > 1900) && (rcSet.throttle < 1000))
    { // arming on 
      arming_on_cnt ++;
      if(arming_on_cnt > ARMING_COUNT)
      {
        arming_off_cnt = 0;
        rcSet.yaw = 1520;
        rcSet.throttle = 1000;
        armingAck.data = 1;
        pubArmingAck.publish(&armingAck);
      }
    }
    
  if((rcSet.yaw < 1100) && (rcSet.throttle < 1000))
    {  //arming off
      arming_off_cnt ++;
      //digitalWrite(31, HIGH-digitalRead(31));   // blink the led 
      if(arming_off_cnt > ARMING_COUNT)
      {
        arming_on_cnt = 0;
        rcSet.yaw = 1520;
        rcSet.throttle = 1000;
        armingAck.data = 0;
        pubArmingAck.publish(&armingAck);
      }
    }
}
ros::Subscriber<aerial_robot_msgs::RcData> subRc("kduino/rc_cmd", &cbRc);

void cbCmd( const std_msgs::UInt16& cmd)		// callback func.
{
  bool gain_res_flag = false;
  switch (cmd.data) {
  case ROS_ROLL_PITCH_PD_P_UP:
    conf.pid[PIDLEVEL].P8 +=2;
    gain_res_flag = true;
    break;
  case ROS_ROLL_PITCH_PD_P_DOWN: 
    conf.pid[PIDLEVEL].P8 -=2;
    gain_res_flag = true;
    break;
  case ROS_ROLL_PD_D_UP:
    conf.pid[ROLL].P8 +=2;
    gain_res_flag = true;
    break;
  case ROS_ROLL_PD_D_DOWN: 
    conf.pid[ROLL].P8 -=2;
    gain_res_flag = true;
    break;
  case ROS_PITCH_PD_D_UP:
    conf.pid[PITCH].P8 +=2;
    gain_res_flag = true;
    break;
  case ROS_PITCH_PD_D_DOWN: 
    conf.pid[PITCH].P8 -=2;
    gain_res_flag = true;
    break;
  case ROS_YAW_D_UP:
    conf.pid[YAW].P8 +=2;
    gain_res_flag = true;
    break;
  case ROS_YAW_D_DOWN: 
    conf.pid[YAW].P8 -=2;
    gain_res_flag = true;
    break;

  case MSP_ACC_CALIBRATION:
    if(!f.ARMED) calibratingA=512;
    //     headSerialReply(0);
    break;
  case MSP_MAG_CALIBRATION:
    if(!f.ARMED) f.CALIBRATE_MAG = 1;
    //     headSerialReply(0);
    break;
  case ROS_ARM_ON_CMD: //arming by ros
    if(f.ARMED)
      {
        armingAck.data = 1;
        pubArmingAck.publish(&armingAck);
      }  
    f.ACC_CALIBRATED = 1;
    if (conf.activate[BOXARM] == 0 ) go_arm();
    break;
  case ROS_ARM_OFF_CMD: //disarming by ros
    if(!f.ARMED)
      {
        armingAck.data = 0;
        pubArmingAck.publish(&armingAck);
        rcSet.roll = MIDRC;
        rcSet.pitch = MIDRC;
        rcSet.yaw = MIDRC;
        rcSet.throttle = MINCOMMAND;
        rcData[ROLL] = rcSet.roll;
        rcData[PITCH] = rcSet.pitch;
        rcData[YAW] = rcSet.yaw;
        rcData[THROTTLE] = rcSet.throttle;
      }
    if (conf.activate[BOXARM] == 0 && f.ARMED) 
      {
         go_disarm();
         rcSet.roll = MIDRC;
         rcSet.pitch = MIDRC;
         rcSet.yaw = MIDRC;
         rcSet.throttle = MINCOMMAND;
         rcData[ROLL] = rcSet.roll;
         rcData[PITCH] = rcSet.pitch;
         rcData[YAW] = rcSet.yaw;
         rcData[THROTTLE] = rcSet.throttle;
      }
  case ROS_DROP_CMD: //disarming by ros
    drop_flag = 1;
    break;
  default:
    break;
  }

  if(gain_res_flag)
    {
      pdGainRes.roll_pitch_p = conf.pid[PIDLEVEL].P8;
      pdGainRes.roll_d = conf.pid[ROLL].P8;
      pdGainRes.pitch_d = conf.pid[PITCH].P8;
      pdGainRes.yaw_d = conf.pid[YAW].P8;
      pubPDGain.publish(&pdGainRes);
    }


}
ros::Subscriber<std_msgs::UInt16> subCmd("kduino/msp_cmd", &cbCmd );


// setup for ROS
void ros_setup()
{
  
  pinMode(31, OUTPUT); //30, 31
  //digitalWrite(31, HIGH);   // blink the led
  // initialize
  ros_loop_cnt = 0;
  arming_on_cnt = 0;
  arming_off_cnt = 0;
  drop_flag = 0;
  router_num = 0;
  drop_time = 0;

  // node initialize
  nh.getHardware()->setBaud(ROS_BAUD);
  nh.initNode();

  nh.advertise(pubKduinoImu);
  nh.advertise(pubArmingAck);
  nh.advertise(pubPDGain);

  // msg(subcriber)
  nh.subscribe(subRc);
  nh.subscribe(subCmd);

  servo[1] = 1060; //left  1000(long rod stop mode) , 2000(short rod push mode)
  servo[2] = 2000; //right, mirror side
}

// loop func for ROS
void ros_loop()
{
  //servo
  //for drop
  if(drop_flag == 2)
    {
      if(currentTime  > drop_time) //1.5sec
        {
          if(router_num %2 == 0) servo[1] = 1060;
          else servo[2] = 2000;
          drop_flag = 0;
          router_num ++;
        }
    }
  if(drop_flag == 1)
    {
      if(router_num %2 == 0) servo[1] = 2000;
      else  servo[2] = 1100;
      drop_flag ++;
      drop_time = currentTime + 1800000;  // 1.5sec
    }
    
    
  // rcData
 #if defined(RCSERIAL) //use serial RC control
  rcData[ROLL]     = rcSet.roll;
  rcData[PITCH]    = rcSet.pitch;
  rcData[YAW]      = rcSet.yaw;
  rcData[THROTTLE] = rcSet.throttle;
  #endif

  // msg(publisher)
  //30-> 10Hz, 3 -> 100Hz
  if ((ros_loop_cnt%3)==1) 
    {
      kduinoImu.stamp = nh.now();
      //acceleration
      kduinoImu.accData[0] =  imu.accSmooth[0];
      kduinoImu.accData[1] =  imu.accSmooth[1];
      kduinoImu.accData[2] =  imu.accSmooth[2];
      kduinoImu.gyroData[0] =  imu.gyroData[0];
      kduinoImu.gyroData[1] =  imu.gyroData[1];
      kduinoImu.gyroData[2] =  imu.gyroData[2];

      /*
      kduinoImu.magData[0] =  imu.gyroADC[0];
      kduinoImu.magData[1] =  imu.gyroADC[1];
      kduinoImu.magData[2] =  imu.gyroADC[2];
      */
      /*
      kduinoImu.magData[0] =  PTermTemp[1];
      kduinoImu.magData[1] =  ITermTemp[1];
      kduinoImu.magData[2] =  DTermTemp[1];
      */
      kduinoImu.magData[0] =  EstG32.A[0];
      kduinoImu.magData[1] =  EstG32.A[1];
      kduinoImu.magData[2] =  EstG32.A[2];


      /*
      kduinoImu.magData[0] =  imu.magADC[0];
      kduinoImu.magData[1] =  imu.magADC[1];
      kduinoImu.magData[2] =  imu.magADC[2];
      */
      //angles
      kduinoImu.angle[0] = att.angle[0]; //10 times
      kduinoImu.angle[1] = att.angle[1]; //10 times
      kduinoImu.angle[2] = att.heading; 
 
     //debug
#if 1
      kduinoImu.altitude = validAcc;  //cm
      //kduinoImu.altitude = alt.EstAlt;  //cm
      //kduinoImu.altitude = laser_altitude;  //cm
#else
      kduinoImu.altitude = cycleTime;
#endif

      int debug;
      debug = pubKduinoImu.publish(&kduinoImu);
    }

  //debug
  int debug;
  debug = nh.spinOnce();  
  //if(debug == -5)
  //  digitalWrite(13, HIGH-digitalRead(13));   // blink the led  
  ros_loop_cnt++;
}
#endif// USE_ROS




