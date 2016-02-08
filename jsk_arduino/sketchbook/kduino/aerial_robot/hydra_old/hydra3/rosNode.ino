#ifdef USE_ROS
// Rosnode.h
#include <ros.h>

// msg

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int8.h>
#include <aerial_robot_msgs/KduinoSimpleImu.h>
#include <aerial_robot_msgs/FourAxisCommand.h> 
#include <aerial_robot_msgs/ITermBias.h>
#include <aerial_robot_msgs/MotorBias.h>
#include <aerial_robot_msgs/AttitudePDGain.h>
#include <aerial_robot_msgs/AttGains.h>
#include <hydra_transform_control/HydraParam.h>
#include <hydra_transform_control/TransformRes.h>

#define ARMING_COUNT 50

// etc
unsigned long ros_loop_cnt;
unsigned int arming_on_cnt;
unsigned int arming_off_cnt;
aerial_robot_msgs::FourAxisCommand rcSet; //buffer for virtual rc command

// nodeHandle
ros::NodeHandle nh;

// msg(publisher)
aerial_robot_msgs::KduinoSimpleImu kduinoImu;
ros::Publisher pubKduinoImu("kduino/simple_imu", &kduinoImu);

std_msgs::Int8 armingAck;
ros::Publisher pubArmingAck("kduino/arming_ack", &armingAck);

aerial_robot_msgs::AttitudePDGain pdGainRes;
ros::Publisher pubPDGain("kduino/pd_gain", &pdGainRes);

hydra_transform_control::HydraParam hyRes;
ros::Publisher pubHy("kduino/hy_res", &hyRes);

hydra_transform_control::TransformRes hyTransformRes;
ros::Publisher pubHyTransform("kduino/hy_transform_res", &hyTransformRes);

aerial_robot_msgs::MotorBias motorBiasRes;
ros::Publisher pubMotorBias("kduino/motor_bias_res", &motorBiasRes);


// msg(subscriber)
void cbFourAxisCommand( const aerial_robot_msgs::FourAxisCommand &cmd_msg) 
{
  rcSet.roll     =  cmd_msg.roll;
  rcSet.pitch    =  cmd_msg.pitch;
  rcSet.yaw      =  cmd_msg.yaw;
  rcSet.throttle = cmd_msg.throttle; 
}
ros::Subscriber<aerial_robot_msgs::FourAxisCommand> subRc("kduino/rc_cmd", &cbFourAxisCommand);

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
        motorBias[0] = 0;
        motorBias[1] = 0;
        motorBias[2] = 0;
        motorBias[3] = 0;
        biasSetFlag = false;

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

        motorBias[0] = 0;
        motorBias[1] = 0;
        motorBias[2] = 0;
        motorBias[3] = 0;
        biasSetFlag = false;

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

void cbITermBias( const aerial_robot_msgs::ITermBias& msg)
{
  //temporari for expo func and pid gain
  int16_t i_term_bias_tmp[2], i_term_bias[2];
  uint16_t tmp,tmp2;
  int16_t tmp3;
  uint8_t axis;
  for(axis=0;axis<2;axis++) { 
    tmp = abs(msg.bias[axis]);
    tmp2 = tmp>>7;
    i_term_bias_tmp[axis] = lookupPitchRollRC[tmp2] + ((tmp-(tmp2<<7)) * (lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2])>>7); 
    if (msg.bias[axis]<0) i_term_bias_tmp[axis] = -i_term_bias_tmp[axis];
    tmp3 = constrain((i_term_bias_tmp[axis]<<1),-500,+500);
    i_term_bias[axis] = ((int32_t)tmp3*conf.pid[PIDLEVEL].P8)>>7;
  }

  motorBias[0] = qMatrix[0] * (int32_t)(i_term_bias[ROLL] + ITermTemp[ROLL]) + qMatrix[4] * (int32_t)(i_term_bias[PITCH] + ITermTemp[PITCH]);
  motorBias[1] = qMatrix[1] * (int32_t)(i_term_bias[ROLL] + ITermTemp[ROLL]) + qMatrix[5] * (int32_t)(i_term_bias[PITCH] + ITermTemp[PITCH]);
  motorBias[2] = qMatrix[2] * (int32_t)(i_term_bias[ROLL] + ITermTemp[ROLL]) + qMatrix[6] * (int32_t)(i_term_bias[PITCH] + ITermTemp[PITCH]);
  motorBias[3] = qMatrix[3] * (int32_t)(i_term_bias[ROLL] + ITermTemp[ROLL]) + qMatrix[7] * (int32_t)(i_term_bias[PITCH] + ITermTemp[PITCH]);

  motorBiasRes.motor_bias[0] = motorBias[0];
  motorBiasRes.motor_bias[1] = motorBias[1];
  motorBiasRes.motor_bias[2] = motorBias[2];
  motorBiasRes.motor_bias[3] = motorBias[3];
  pubMotorBias.publish(&motorBiasRes);

  biasSetFlag = true;
}
ros::Subscriber<aerial_robot_msgs::ITermBias> subITerm("kduino/i_term_bias", &cbITermBias );

void cbAttGains( const aerial_robot_msgs::AttGains& msg)
{
  conf.pid[ROLL].P8 = msg.roll_d_gain;
  conf.pid[PITCH].P8 = msg.pitch_d_gain;
}
ros::Subscriber<aerial_robot_msgs::AttGains> subAttGains("kduino/att_gains", &cbAttGains);



void cbHydraParam(const hydra_transform_control::HydraParam& param)
{
  //roll: 0, 1, 2, 3
  //pitch: 4, 5, 6, 7
  //yaw: 8, 9, 10, 11
  // x 1024(<<10factor)
  memcpy(&qMatrix, &(param.q_matrix[0]), 24); 
  //memcpy(&iPrincipalRate, &(param.i_principal_rate), 6); 
  memcpy(&rotateAngle, &(param.rotate_angle), 4);
  
}

ros::Subscriber<hydra_transform_control::HydraParam> subHy("kduino/hydra_param", &cbHydraParam);

// setup for ROS
void ros_setup()
{
  
  pinMode(31, OUTPUT); //30, 31
  //digitalWrite(31, HIGH);   // blink the led
  // initialize
  ros_loop_cnt = 0;
  arming_on_cnt = 0;
  arming_off_cnt = 0;

  // node initialize
  nh.getHardware()->setBaud(ROS_BAUD);
  nh.initNode();

  nh.advertise(pubKduinoImu);
  nh.advertise(pubHy);
  nh.advertise(pubHyTransform);
  nh.advertise(pubArmingAck);
  nh.advertise(pubMotorBias);
  nh.advertise(pubPDGain);
  // msg(subcriber)
  nh.subscribe(subRc);
  nh.subscribe(subCmd);
  nh.subscribe(subHy);
  nh.subscribe(subITerm);
  nh.subscribe(subAttGains);

  servo[1] = 1060; //left  1000(long rod stop mode) , 2000(short rod push mode)
  servo[2] = 2000; //right, mirror side

  iPrincipalRate[0] = 1;   iPrincipalRate[1] = 1; iPrincipalRate[2] = 1;
 

  //for hydra3, this is no d_gyro control mode
  conf.pid[PIDLEVEL].P8 = 90; 
  conf.pid[ROLL].P8 = 31; 
  conf.pid[PITCH].P8 = 31; 
  conf.pid[YAW].P8 = 80; 


   rcSet.roll     = 0;
   rcSet.pitch    = 0;
   rcSet.yaw      = 0;
   rcSet.throttle      = MINCOMMAND;

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

      
       // kduinoImu.roll_control[0] = PTermTemp[0]; 
       // kduinoImu.roll_control[1] = ITermTemp[0]; 
       // kduinoImu.roll_control[2] = DTermTemp[0]; 
       // kduinoImu.pitch_control[0] = PTermTemp[1]; 
       // kduinoImu.pitch_control[1] = ITermTemp[1]; 
       // kduinoImu.pitch_control[2] = DTermTemp[1]; 
       // kduinoImu.yaw_control[0] = PTermTemp[2]; 
       // kduinoImu.yaw_control[1] = ITermTemp[2]; 
       // kduinoImu.yaw_control[2] = DTermTemp[2]; 
      

      pubKduinoImu.publish(&kduinoImu);
    }
  if ((ros_loop_cnt%50)==1)  //-1: no output
    {
      hyTransformRes.motor_value[0] = motor[0];
      hyTransformRes.motor_value[1] = motor[1];
      hyTransformRes.motor_value[2] = motor[2];
      hyTransformRes.motor_value[3] = motor[3];
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




