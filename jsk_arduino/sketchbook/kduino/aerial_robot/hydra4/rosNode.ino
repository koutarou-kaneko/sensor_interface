#ifdef USE_ROS
// Rosnode.h
#include <ros.h>

// msg

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <aerial_robot_msgs/SimpleImu.h>
#include <aerial_robot_msgs/FourAxisCommand.h> 
#include <aerial_robot_msgs/RollPitchYawGain.h> 
#include <aerial_robot_msgs/MotorValue.h> 
#include <aerial_robot_msgs/ControlTerm.h>

#define ARMING_COUNT 50

//for ros start/stop cmd
#define ROS_ARM_ON_CMD           0x00   //arming on, old: 150
#define ROS_ARM_OFF_CMD          0x01   //arming off, old: 151
#define ROS_INTEGRATE_CMD        160   //integrate flag, old: 160


// etc
unsigned long ros_loop_cnt;
unsigned int arming_on_cnt;
unsigned int arming_off_cnt;
aerial_robot_msgs::FourAxisCommand rcSet; //buffer for virtual rc command

// nodeHandle
ros::NodeHandle nh;

// msg(publisher)
aerial_robot_msgs::SimpleImu kduinoImu;
ros::Publisher pubKduinoImu("imu", &kduinoImu);

std_msgs::UInt8 armingAck;
ros::Publisher pubArmingAck("flight_config_ack", &armingAck);

aerial_robot_msgs::MotorValue motorValue;
ros::Publisher pubMotorValues("motor_values", &motorValue);

//debug
aerial_robot_msgs::ControlTerm controlTerm;
ros::Publisher pubControlTerms("control_terms", &controlTerm);

// msg(subscriber)
void cbFourAxisCommand( const aerial_robot_msgs::FourAxisCommand &cmd_msg) 
{
  command_angles[0] = cmd_msg.angles[0];
  command_angles[1] = cmd_msg.angles[1];
  
  uint8_t i;
  for(i = 0; i < NUMBER_MOTOR; i++)
    {
      yaw_pi_term[i] = cmd_msg.yaw_pi_term[i];
      throttle_pid_term[i] = cmd_msg.throttle_pid_term[i];
    }
}
ros::Subscriber<aerial_robot_msgs::FourAxisCommand> subRc("aerial_robot_control", &cbFourAxisCommand);

void cbCmd( const std_msgs::UInt8& cmd)		// callback func.
{
  bool gain_res_flag = false;
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
        resetCommand();

        armingAck.data = ROS_ARM_ON_CMD;
        pubArmingAck.publish(&armingAck);
      }  
    //f.ACC_CALIBRATED = 1; //should check this!!
    go_arm();
    break;
  case ROS_ARM_OFF_CMD: //disarming by ros
    if(!f.ARMED)
      {
         resetCommand();
        armingAck.data = ROS_ARM_OFF_CMD;
        pubArmingAck.publish(&armingAck);

      }
    if ( f.ARMED) 
      {
         go_disarm();
         resetCommand();
      }
  case ROS_INTEGRATE_CMD:
    integrate_flag = 1;
  default:
    break;
  }
}
ros::Subscriber<std_msgs::UInt8> subCmd("flight_config_cmd", &cbCmd );


void cbRPYGain(const aerial_robot_msgs::RollPitchYawGain& hydra_param)
{
  int i, axis;

  rotateAngle[0] = hydra_param.angle_cos; //cos
  rotateAngle[1] = hydra_param.angle_sin; //cos
#if 1
  for(i = 0; i < NUMBER_MOTOR; i++)
    {
      p_gain[i][ROLL] = hydra_param.roll_p_gain[i];
      i_gain[i][ROLL] = hydra_param.roll_i_gain[i];
      d_gain[i][ROLL] = hydra_param.roll_d_gain[i];

      p_gain[i][PITCH] = hydra_param.pitch_p_gain[i];
      i_gain[i][PITCH] = hydra_param.pitch_i_gain[i];
      d_gain[i][PITCH] = hydra_param.pitch_d_gain[i];

      d_gain[i][YAW] = hydra_param.yaw_d_gain[i];
    }
#endif
}
ros::Subscriber<aerial_robot_msgs::RollPitchYawGain> subRPYGain("rpy_gain", &cbRPYGain);


// setup for ROS
void ros_setup()
{
  // initialize
  ros_loop_cnt = 0;
  arming_on_cnt = 0;
  arming_off_cnt = 0;

  // node initialize
  nh.getHardware()->setBaud(ROS_BAUD);
  nh.initNode();

  nh.advertise(pubKduinoImu);
  nh.advertise(pubArmingAck);
  nh.advertise(pubMotorValues);

  //debug
  nh.advertise(pubControlTerms);

  // msg(subcriber)
  nh.subscribe(subCmd);
  nh.subscribe(subRc);
  nh.subscribe(subRPYGain);
}

// loop func for ROS
void ros_loop()
{
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

      kduinoImu.debug = cycleTime;


      pubKduinoImu.publish(&kduinoImu);
    }
  if ((ros_loop_cnt%50)==1)  //-1: no output
    {
      motorValue.stamp = nh.now();
      motorValue.motor_value[0] = motor[0];
      motorValue.motor_value[1] = motor[1];
      motorValue.motor_value[2] = motor[2];
      motorValue.motor_value[3] = motor[3];
      pubMotorValues.publish(&motorValue);
    }
  if((ros_loop_cnt%50)==2){
    for(int i = 0; i < NUMBER_MOTOR; i++)
      {
        controlTerm.roll_p_term[i] = roll_p_term[i];
        controlTerm.roll_i_term[i] = roll_i_term[i];
        controlTerm.roll_d_term[i] = roll_d_term[i];

        controlTerm.pitch_p_term[i] = pitch_p_term[i];
        controlTerm.pitch_i_term[i] = pitch_i_term[i];
        controlTerm.pitch_d_term[i] = pitch_d_term[i];

        controlTerm.yaw_d_term[i] = yaw_d_term[i];

      }
    pubControlTerms.publish(&controlTerm);
  }
  
  nh.spinOnce();  
  ros_loop_cnt++;
}
#endif// USE_ROS




