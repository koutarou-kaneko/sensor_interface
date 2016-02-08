#ifdef USE_ROS
// Rosnode.h
#include <ros.h>

// msg

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int8.h>
#include <jsk_quadcopter_common/KduinoSimpleImu.h>
#include <jsk_quadcopter_common/RcData.h> //baffur for virtual rc
#include <hydra/HydraParam.h>
#include <hydra/TransformRes.h>

#define ARMING_COUNT 50

// etc
unsigned long ros_loop_cnt;
unsigned int arming_on_cnt;
unsigned int arming_off_cnt;
unsigned int drop_flag;
unsigned int router_num;
unsigned long drop_cmd_time;
uint32_t drop_time;
jsk_quadcopter_common::RcData rcSet; //buffer for virtual rc command

// nodeHandle
ros::NodeHandle nh;

// msg(publisher)
jsk_quadcopter_common::KduinoSimpleImu kduinoImu;
ros::Publisher pubKduinoImu("kduino/simple_imu", &kduinoImu);

std_msgs::Int8 armingAck;
ros::Publisher pubArmingAck("kduino/arming_ack", &armingAck);

jsk_quadcopter_common::RcData rcRes;
ros::Publisher pubRC("kduino/rc_res", &rcRes);

hydra::HydraParam hyRes;
ros::Publisher pubHy("kduino/hy_res", &hyRes);

hydra::TransformRes hyTransformRes;
ros::Publisher pubHyTransform("kduino/hy_transform_res", &hyTransformRes);


// msg(subscriber)
void cbRc( const jsk_quadcopter_common::RcData &cmd_msg) 
{
  rcSet.roll     = MIDRC + cmd_msg.roll;
  rcSet.pitch    = MIDRC + cmd_msg.pitch;
  rcSet.yaw      = MIDRC + cmd_msg.yaw;
  // rcSet.roll     = MIDRC;
  // rcSet.pitch    = MIDRC;
  // rcSet.yaw      = MIDRC;
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
ros::Subscriber<jsk_quadcopter_common::RcData> subRc("kduino/rc_cmd", &cbRc);

void cbCmd( const std_msgs::UInt16& cmd)		// callback func.
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
        rcSet.roll = 0;
        rcSet.pitch = 0;
        rcSet.yaw = 0;
        rcSet.throttle = MINCOMMAND;
        rcData[ROLL] = rcSet.roll;
        rcData[PITCH] = rcSet.pitch;
        rcData[YAW] = rcSet.yaw;
        rcData[THROTTLE] = rcSet.throttle;
      }
    if (conf.activate[BOXARM] == 0 && f.ARMED) 
      {
         go_disarm();
         rcSet.roll = 0;
         rcSet.pitch = 0;
         rcSet.yaw = 0;
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
}
ros::Subscriber<std_msgs::UInt16> subCmd("kduino/msp_cmd", &cbCmd );

void cbHydraParam(const hydra::HydraParam& param)
{
  //roll: 0, 1, 2, 3
  //pitch: 4, 5, 6, 7
  //yaw: 8, 9, 10, 11
  // x 1024(<<10factor)
  memcpy(&qMatrix, &(param.q_matrix[0]), 24); 
  //memcpy(&iPrincipalRate, &(param.i_principal_rate), 6); 
  memcpy(&rotateAngle, &(param.rotate_angle), 4);
  
  //for  pid modification
  /* 
 dynamic_gain.pid_att[ROLL].P8 = (uint8_t)(((uint32_t)conf.pid[PIDLEVEL].P8 * iPrincipalRate[0]) >> 10);
  dynamic_gain.pid_att[ROLL].I8 = (uint8_t)(((uint32_t)conf.pid[PIDLEVEL].I8 * iPrincipalRate[0]) >> 10);
  if(iPrincipalRate[0] > 1024) 
    dynamic_gain.pid_att[ROLL].D8 = (uint8_t)(((uint32_t)conf.pid[PIDLEVEL].D8 * iPrincipalRate[0]) >> 10);

  dynamic_gain.pid_att[PITCH].P8 = (uint8_t)(((uint32_t)conf.pid[PIDLEVEL].P8 * iPrincipalRate[1]) >> 10);
  dynamic_gain.pid_att[PITCH].I8 = (uint8_t)(((uint32_t)conf.pid[PIDLEVEL].I8 * iPrincipalRate[1]) >> 10);
  if(iPrincipalRate[1] > 1024) 
    dynamic_gain.pid_att[PITCH].D8 = (uint8_t)(((uint32_t)conf.pid[PIDLEVEL].D8 * iPrincipalRate[1]) >> 10);

  dynamic_gain.pid_gyro[ROLL].P8 = (uint8_t)(((uint32_t)conf.pid[ROLL].P8 * iPrincipalRate[0]) >> 10);
  dynamic_gain.pid_gyro[ROLL].I8 = (uint8_t)(((uint32_t)conf.pid[ROLL].I8 * iPrincipalRate[0]) >> 10);
  dynamic_gain.pid_gyro[ROLL].D8 = (uint8_t)(((uint32_t)conf.pid[ROLL].D8 * iPrincipalRate[0]) >> 10);

  dynamic_gain.pid_gyro[PITCH].P8 = (uint8_t)(((uint32_t)conf.pid[PITCH].P8 * iPrincipalRate[1]) >> 10);
  dynamic_gain.pid_gyro[PITCH].I8 = (uint8_t)(((uint32_t)conf.pid[PITCH].I8 * iPrincipalRate[1]) >> 10);
  dynamic_gain.pid_gyro[PITCH].D8 = (uint8_t)(((uint32_t)conf.pid[PITCH].D8 * iPrincipalRate[1]) >> 10);

  dynamic_gain.pid_gyro[YAW].P8 = (uint8_t)(((uint32_t)conf.pid[YAW].P8 * iPrincipalRate[2]) >> 10);
  dynamic_gain.pid_gyro[YAW].I8 = (uint8_t)(((uint32_t)conf.pid[YAW].I8 * iPrincipalRate[2]) >> 10);
  dynamic_gain.pid_gyro[YAW].D8 = (uint8_t)(((uint32_t)conf.pid[YAW].D8 * iPrincipalRate[2]) >> 10);
*/
}

ros::Subscriber<hydra::HydraParam> subHy("kduino/hydra_param", &cbHydraParam);

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
  nh.advertise(pubRC);
  nh.advertise(pubHy);
  nh.advertise(pubHyTransform);
  nh.advertise(pubArmingAck);

  // msg(subcriber)
  nh.subscribe(subRc);
  nh.subscribe(subCmd);
  nh.subscribe(subHy);

  servo[1] = 1060; //left  1000(long rod stop mode) , 2000(short rod push mode)
  servo[2] = 2000; //right, mirror side

  iPrincipalRate[0] = 1;   iPrincipalRate[1] = 1; iPrincipalRate[2] = 1;
}

// loop func for ROS
void ros_loop()
{
    
    
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
      kduinoImu.accData[0] =  imu.accSmooth[0]; //shoulde change to transformed one
      kduinoImu.accData[1] =  imu.accSmooth[1]; //shoulde change to transformed one
      kduinoImu.accData[2] =  imu.accSmooth[2]; //shoulde change to transformed one

      //angles
      //kduinoImu.angle[0] = att.angle[0]; //10 times
      //kduinoImu.angle[1] = att.angle[0]; //10 times
      kduinoImu.angle[0] = att_cog.angle[0]; //10 times
      kduinoImu.angle[1] = att_cog.angle[1]; //10 times

      kduinoImu.angle[2] = att.heading; 

      
      kduinoImu.roll_control[0] = PTermTemp[0]; 
      kduinoImu.roll_control[1] = ITermTemp[0]; 
      kduinoImu.roll_control[2] = DTermTemp[0]; 
      kduinoImu.pitch_control[0] = PTermTemp[1]; 
      kduinoImu.pitch_control[1] = ITermTemp[1]; 
      kduinoImu.pitch_control[2] = DTermTemp[1]; 
      kduinoImu.yaw_control[0] = PTermTemp[2]; 
      kduinoImu.yaw_control[1] = ITermTemp[2]; 
      kduinoImu.yaw_control[2] = DTermTemp[2]; 
      

      pubKduinoImu.publish(&kduinoImu);
    }
  if ((ros_loop_cnt%50)==-1)  //-1: no output
    {
      // pubRC
      rcRes.roll = rcData[ROLL];
      rcRes.pitch = rcData[PITCH];
      rcRes.yaw = rcData[YAW];
      rcRes.throttle = rcData[THROTTLE];
      pubRC.publish(&rcRes);

      //pubHydra
      //for(int i = 0; i < 16 ; i ++)
      //hyRes.q_matrix[i] = 0;
      //hyRes.i_principal[0] = 0;   hyRes.i_principal[1] = 0; hyRes.i_principal[2] = 0;
      //hyRes.rotate_angle[0] = 0;   hyRes.rotate_angle[1] = 0;

      memcpy(&(hyRes.q_matrix), &qMatrix, 24); 
      //memcpy(&(hyRes.i_principal_rate), &iPrincipalRate, 6); 
      memcpy(&(hyRes.rotate_angle), &rotateAngle, 4); 
      pubHy.publish(&hyRes);
    }
  if ((ros_loop_cnt%50)==-1)  //-1: no output
    {
      // transform test
      int16_t z_roll_pitch_yaw[4] = {1600, -200, 300, -400};
      //int32_t motor_value1 =  (int32_t)((int32_t)z_roll_pitch_yaw[0] + (int32_t)z_roll_pitch_yaw[1] * qMatrix[0] + (int32_t)z_roll_pitch_yaw[2] * qMatrix[4] + YAW_DIRECTION * (int32_t)z_roll_pitch_yaw[3] * qMatrix[8]) >> 10;
      int32_t motor_value1 =  (int32_t)z_roll_pitch_yaw[0] + (int32_t)(((int32_t)z_roll_pitch_yaw[1] * qMatrix[0] + (int32_t)z_roll_pitch_yaw[2] * qMatrix[4] + YAW_DIRECTION * (int32_t)z_roll_pitch_yaw[3] * qMatrix[8]) >> 10);

      int32_t motor_value2 =  (int32_t)z_roll_pitch_yaw[0] + (int32_t)(((int32_t)z_roll_pitch_yaw[1] * qMatrix[1] + (int32_t)z_roll_pitch_yaw[2] * qMatrix[5] + YAW_DIRECTION * (int32_t)z_roll_pitch_yaw[3] * qMatrix[9]) >> 10);
      int32_t motor_value3 =  (int32_t)z_roll_pitch_yaw[0] + (int32_t)(((int32_t)z_roll_pitch_yaw[1] * qMatrix[2] + (int32_t)z_roll_pitch_yaw[2] * qMatrix[6] + YAW_DIRECTION * (int32_t)z_roll_pitch_yaw[3] * qMatrix[10])>>10);
      int32_t motor_value4 =  (int32_t)z_roll_pitch_yaw[0] + (int32_t)(((int32_t)z_roll_pitch_yaw[1] * qMatrix[3] + (int32_t)z_roll_pitch_yaw[2] * qMatrix[7] + YAW_DIRECTION * (int32_t)z_roll_pitch_yaw[3] * qMatrix[11])>>10);
      
      hyTransformRes.motor_value[0] = motor_value1;
      hyTransformRes.motor_value[1] = motor_value2;
      hyTransformRes.motor_value[2] = motor_value3;
      hyTransformRes.motor_value[3] = motor_value4;
      /*
      hyTransformRes.pid_att_roll[0] = dynamic_gain.pid_att[ROLL].P8;
      hyTransformRes.pid_att_roll[1] = dynamic_gain.pid_att[ROLL].I8;
      hyTransformRes.pid_att_roll[2] = dynamic_gain.pid_att[ROLL].D8;
      hyTransformRes.pid_att_pitch[0] = dynamic_gain.pid_att[PITCH].P8;
      hyTransformRes.pid_att_pitch[1] = dynamic_gain.pid_att[PITCH].I8;
      hyTransformRes.pid_att_pitch[2] = dynamic_gain.pid_att[PITCH].D8;

      hyTransformRes.pid_gyro_roll[0] = dynP8[ROLL];
      hyTransformRes.pid_gyro_roll[1] = dynamic_gain.pid_gyro[ROLL].I8;
      hyTransformRes.pid_gyro_roll[2] = dynD8[ROLL];
      hyTransformRes.pid_gyro_pitch[0] = dynP8[PITCH];
      hyTransformRes.pid_gyro_pitch[1] = dynamic_gain.pid_gyro[PITCH].I8;
      hyTransformRes.pid_gyro_pitch[2] = dynD8[PITCH];
      hyTransformRes.pid_gyro_yaw[0] = dynP8[YAW];
      hyTransformRes.pid_gyro_yaw[1] = dynamic_gain.pid_gyro[YAW].I8;
      hyTransformRes.pid_gyro_yaw[2] = dynD8[YAW];
      */
      pubHyTransform.publish(&hyTransformRes);
    }


  //debug
  int debug;
  debug = nh.spinOnce();  
  //if(debug == -5)
  //  digitalWrite(13, HIGH-digitalRead(13));   // blink the led  
  ros_loop_cnt++;
}
#endif// USE_ROS




