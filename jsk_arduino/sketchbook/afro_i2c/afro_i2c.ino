// Wire Master Writer
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Writes data to an I2C/TWI slave device
// Refer to the "Wire Slave Receiver" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>
#include <ros.h>
#include <std_msgs/UInt8.h>

// nodeHandle
ros::NodeHandle nh;


uint8_t motor = 0;
#define MOTOR_ADRS 0x52

void sendCmd()
{
  Wire.beginTransmission(MOTOR_ADRS); // transmit to device #4
  Wire.write(motor);        // sends five bytes
  Wire.endTransmission();    // stop transmitting
}

void motorCmd( const std_msgs::UInt8& cmd)// callback func.
{
  motor = cmd.data;
}
ros::Subscriber<std_msgs::UInt8> subCmd("motor_cmd", &motorCmd );


void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(subCmd);

  Wire.begin(); // join i2c bus (address optional for master)
 #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz, for the Due                                                                                               
    #else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz                                                                                               
    #endif


  delay(100);

}



void loop()
{
  sendCmd();
  //motor++; 
 delay(10);
  nh.spinOnce();  
}
