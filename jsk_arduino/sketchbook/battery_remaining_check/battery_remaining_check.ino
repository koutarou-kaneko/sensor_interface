/* 
 * rosserial ADC Example
 * 
 * This is a poor man's Oscilloscope.  It does not have the sampling 
 * rate or accuracy of a commerical scope, but it is great to get
 * an analog value into ROS in a pinch.
 */

/*
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
*/
#include <ros.h>
#include <rosserial_arduino/Adc.h>
#include "std_msgs/Float64.h"

#define BATTERY_CAPACITY 15.0 //[Ah]
#define INITIAL_ADC 507

ros::NodeHandle nh;

rosserial_arduino::Adc adc_msg;
std_msgs::Float64 current_msg;
std_msgs::Float64 uci_msg; //used current integration
std_msgs::Float64 brc_msg; //battery remaining capacity
std_msgs::Float64 brp_msg; //battery remaining percentage

ros::Publisher p("Adc", &adc_msg);
ros::Publisher current_pub("Current", &current_msg);
ros::Publisher uci_pub("UsedCurrentIntegration", &uci_msg);
ros::Publisher brc_pub("BatteryRemainingCapacity", &brc_msg);
ros::Publisher brp_pub("BatteryRemainingPercentage", &brp_msg);

static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;
static float prev_uci = 0.0; //prevous current integration value

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  
  nh.advertise(p);
  nh.advertise(current_pub);
  nh.advertise(uci_pub);
  nh.advertise(brc_pub);
  nh.advertise(brp_pub);

}

//We average the analog reading to elminate some of the noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

//long adc_timer;

void loop()
{
  current_msg.data = 0.0;
  uci_msg.data = 0.0;

  adc_msg.adc0 = averageAnalog(0);
  current_msg.data = (adc_msg.adc0 - (float)INITIAL_ADC) * 5.0 / 1024.0 / 0.019; // analog to current[A]. 5[V] / 1024 / 0.019[A/V]で最小分解能は0.25699[A].
  uci_msg.data = prev_uci + current_msg.data * (float)cycleTime / 1000 / 1000; //消費電流を積算.battery capacity [As]
  brc_msg.data = BATTERY_CAPACITY * 3600 - uci_msg.data; //バッテリー残量[As]
  brp_msg.data = brc_msg.data / (BATTERY_CAPACITY * 3600) * 100; // バッテリ残量[As]/バッテリ総量[As]. 割合

//  rt_msg.hour = rt_msg.data / 3600;        // [h]
//  rt_msg.min  = (rt_msg.hour % 3600) / 60; // [min]
//  rt_msg.s    = (rt_msg.hour % 3600) % 60; // [s]


    
  p.publish(&adc_msg);
  current_pub.publish(&current_msg);
  uci_pub.publish(&uci_msg);
  brc_pub.publish(&brc_msg);
  brp_pub.publish(&brp_msg);

  nh.spinOnce();

  prev_uci = uci_msg.data;
  currentTime = micros();
  cycleTime = currentTime - previousTime; //[micros]
  previousTime = currentTime;
}

