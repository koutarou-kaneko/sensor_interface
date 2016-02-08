/*
 Circuit:
 SCP1000 sensor attached to pins 7, 10 - 13:
 CSB: pin 7
 MOSI: pin 11
 MISO: pin 12
 SCK: pin 13
 
 created 31 July 2010
 modified 14 August 2010
 by Tom Igoe
 */

// the sensor communicates using SPI, so include the library:
#include <SPI.h>

#define MPU6050_DLPF_CFG   0
const int chipSelectPin = 7;

#define MAG_ADDRESS 0x0C
#define MAG_DATA_REGISTER 0x03


void setup() {
  Serial.begin(9600);

  
  // start the SPI library:
  SPI.begin();

  // initalize the  data ready and chip select pins:
  pinMode(chipSelectPin, OUTPUT);

  // give the sensor time to set up:
  delay(100);
  writeRegister(0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(5);
  writeRegister(0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0;
  writeRegister(0x1A, MPU6050_DLPF_CFG);
  writeRegister(0x1B, 0x18);
  delay(100);
  writeRegister(0x1C, 0x10);
  delay(100);

  //mag
  writeRegister( 0x6A, 0b00100000); //USER_CTRL -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
  delay(10);
  writeRegister( 0x37, 0x00); //INT_PIN_CFG -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
  delay(10);
  writeRegister( 0x24, 0x0D);

  delay(1);
  writeRegister( 0x25, 0x0C);
  delay(1);

  writeRegister( 0x26, 0x0B);
  delay(1);
  writeRegister( 0x63, 0x01);
  delay(1);
  writeRegister( 0x27, 0x81);
  delay(1);

  writeRegister( 0x26, 0x0A);
  delay(1);
  writeRegister( 0x63, 0x12);
  delay(1);
  writeRegister( 0x27, 0x81);

  delay(1);

  writeRegister( 0x25, 0x80|MAG_ADDRESS);
  delay(1);
   writeRegister( 0x26, 0x03);
   delay(1);
   writeRegister( 0x27, 0x87);



  // delay(10);
  // writeRegister( 0x25, 0x80|MAG_ADDRESS);
  // delay(10);
  // writeRegister( 0x26, MAG_DATA_REGISTER);
  // delay(10);
  // writeRegister( 0x27, 0x86);

  // delay(10);
  // writeRegister( 0x31, MAG_ADDRESS);
  // delay(10);
  // writeRegister( 0x32, 0x0a);
  // delay(10);
  // writeRegister( 0x33, 0x01);

  // delay(100);
  // writeRegister( 0x34, 0x80); 
  // delay(200);

}

void loop() {
int acc_x = 0, acc_y =0, acc_z = 0;
  byte temp = 0;

  /*
  //test whi am i
  readRegister(0x75, 1);



#if 0
  
  temp = readRegister(0x3B, 1);
  acc_x = temp << 8;
  temp = readRegister(0x3C, 1);
  acc_x = acc_x | temp;
  temp = readRegister(0x3D, 1);
  acc_y = temp << 8;
  temp = readRegister(0x3E, 1);
  acc_y = acc_y | temp;
  temp = readRegister(0x3F, 1);
  acc_z = temp << 8;
  temp = readRegister(0x40, 1);
  acc_z = acc_z | temp;


#else
  byte dataToSend = 0x3B | 0x80;
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(dataToSend);  
  temp = SPI.transfer(0x00);
  acc_x = temp << 8;
  temp = SPI.transfer(0x00);
  acc_x = acc_x | temp;
  temp = SPI.transfer(0x00);
  acc_y = temp << 8;
  temp = SPI.transfer(0x00);
  acc_y = acc_y | temp;
  temp = SPI.transfer(0x00);
  acc_z = temp << 8;
  temp = SPI.transfer(0x00);
  acc_z = acc_z | temp;
  digitalWrite(chipSelectPin, HIGH);
#endif

  Serial.println(String(acc_x) + "; " + String(acc_y) + "; " + String(acc_z));

  delay(100);
  
  delay(100);
  temp = readRegister(0x1c, 1);
    Serial.println(String(temp));
    */
  //   writeRegister(0x1C, 0x10);



   // writeRegister( 0x26, 0x00);
   // delay(1);
   // writeRegister( 0x27, 0x81);
   // temp = readRegister(0x49, 1);
   // Serial.println(String(temp));


  byte dataToSend = 0x49 | 0x80;
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(dataToSend);  
  temp = SPI.transfer(0x00);
  acc_x = temp;
  temp = SPI.transfer(0x00);
  acc_x = acc_x | (temp << 8);
  temp = SPI.transfer(0x00);
  acc_y = temp;
  temp = SPI.transfer(0x00);
  acc_y = acc_y | (temp << 8);
  temp = SPI.transfer(0x00);
  acc_z = temp;
  temp = SPI.transfer(0x00);
  acc_z = acc_z | (temp << 8);
  temp = SPI.transfer(0x00);

  digitalWrite(chipSelectPin, HIGH);
  Serial.println(String(acc_x) + "; " + String(acc_y) + "; " + String(acc_z));




     delay(100);

}

//Read from or write to register from the SCP1000:
unsigned int readRegister(byte thisRegister, int bytesToRead ) {
  byte inByte = 0;           // incoming byte from the SPI

  // now combine the address and the command into one byte
  byte dataToSend = thisRegister | 0x80;
  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);
  // send the device the register you want to read:
  SPI.transfer(dataToSend);
  // send a value of 0 to read the first byte returned:
  inByte = SPI.transfer(0x00);
  // decrement the number of bytes left to read:
  bytesToRead--;
  // if you still have another byte to read:
  if (bytesToRead > 0) {
    // shift the first byte left, then get the second byte:
    inByte = SPI.transfer(0x00);
    // decrement the number of bytes left to read:
    bytesToRead--;
  }
  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
  // return the result:
  return(inByte);
}



void writeRegister(byte thisRegister, byte thisValue) {

  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(thisRegister); //Send register location
  SPI.transfer(thisValue);  //Send value to record into register

  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
}

