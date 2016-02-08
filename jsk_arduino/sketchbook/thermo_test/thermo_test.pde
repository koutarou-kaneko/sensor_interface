#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>

#include <WProgram.h>
#include <Servo.h>

#define ThermoStandardT (25.0 + 273.0) /* [K] */
#define ThermoStandardR (10.0 * 1e3) /* [ohm] */
#define ThermoB         3435.0 /* [K] */
#define ThermoResisterDividor 5600.0 /* [ohm] */
#define ThermoVcc 5.0 /* [V] */

#define AD2thermoR(data) (ThermoResisterDividor * ((ThermoVcc / (ThermoVcc * (data / 1024.0))) - 1.0)) /* [ohm] */
#define AD2thermo(data) ((1.0 / ((1.0 / ThermoStandardT) + (((log( AD2thermoR(data) ) ) - ( log(ThermoStandardR) )) / ThermoB))) - 273.0) /* [Degree Celsius] */

ros::NodeHandle nh;

std_msgs::Float32MultiArray thermo;
ros::Publisher thermo_pub("thermo", &thermo);

char dim0_label[] = "thermo";
void setup()
{
  nh.initNode();
  thermo.layout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension) * 1);
  thermo.layout.dim[0].label = dim0_label;
  thermo.layout.dim[0].size = 8;
  thermo.layout.dim[0].stride = 1*8;
  thermo.layout.data_offset = 0;
  thermo.data = (float *)malloc(sizeof(float)*8);
  thermo.data_length = 8;
  nh.advertise(thermo_pub);
}

void loop()
{
  for(int i = 0; i < 8; i++){
    thermo.data[i] = AD2thermo(analogRead(i));
  }
  
  thermo_pub.publish( &thermo );
  
  nh.spinOnce();

  delay(500);
}
