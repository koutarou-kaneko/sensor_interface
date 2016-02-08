#include <ros.h>
#include <rosserial_arduino/Adc.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <rosserial_arduino/Test.h>
#include <jsk_tools/BatteryParameters.h>

using jsk_tools::BatteryParameters;

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

// http://www.mlabo.com/mcs_md_series.html
float threshold = 1024 / 2;
float capacity = 15;
float ratio = 23;

void callback(const BatteryParameters::Request & req, BatteryParameters::Response & res){
  threshold = req.threshold;
  capacity = req.capacity;
  ratio = req.ratio;
}

ros::ServiceServer<BatteryParameters::Request, BatteryParameters::Response> service("set_battery_params", &callback);

float averageAnalog(int pin, int num){
  float v = 0;
  for(int i=0; i<num; i++) {
    v+= analogRead(pin);
  }
  return v / num;
}

void setup() {
  Serial.begin(57600);
  // Serial.begin(9600);
  // ros
  nh.initNode();
  nh.advertiseService(service);
  nh.advertise(p);
  nh.advertise(current_pub);
  nh.advertise(uci_pub);
  nh.advertise(brc_pub);
  nh.advertise(brp_pub);
  // initialize
  threshold = averageAnalog(0, 20);
}



void loop() {
  Serial.println(cycleTime);

  adc_msg.adc0 = averageAnalog(0, 4); // [bit]
  current_msg.data = (adc_msg.adc0 - threshold) * (5.0 / 1024.0) / (ratio * 0.001); // current [A] : [bit] * [V/bit] / [V/A]
  uci_msg.data = prev_uci + current_msg.data * ((float)cycleTime / 1000 / 1000); // used [As]
  brc_msg.data = capacity * 60 * 60 - uci_msg.data; // remaining [As]
  brp_msg.data = brc_msg.data / (capacity * 60 * 60) * 100; // percentage [%]

  p.publish(&adc_msg);
  current_pub.publish(&current_msg);
  uci_pub.publish(&uci_msg);
  brc_pub.publish(&brc_msg);
  brp_pub.publish(&brp_msg);

  nh.spinOnce();
  delay(500);
  prev_uci = uci_msg.data;
  currentTime = micros();
  cycleTime = currentTime - previousTime; //[micros]
  previousTime = currentTime;
}

