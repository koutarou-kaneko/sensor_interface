
/*
MultiWiiCopter by Alexandre Dubus
 www.multiwii.com
 March  2013     V2.2
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 */

#include <avr/io.h>

#include "config.h"
#include "def.h"

#include <avr/pgmspace.h>
#define  VERSION  221  

enum rc {
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
  AUX1,
  AUX2,
  AUX3,
  AUX4
};

static uint32_t loopCount = 0;
static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0; // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint16_t calibratingA = 0; // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint16_t calibratingB = 0; // baro calibration = get new ground pressure value
static uint16_t calibratingG;     // Gyro calibration
static uint8_t  vbatMin = VBATNOMINAL;  // lowest battery voltage in 0.1V steps

// **************
// gyro+acc IMU
// **************
int16_t acc1G[3] =  { 0, 0, ACC_1G };
static int16_t gyroZero[3] = { 0,0,0};
static int16_t angle[2] = { 0,0}; // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
static struct {
  int16_t accSmooth[3];
  int16_t gyroData[3];
  int16_t magADC[3];
  int16_t gyroADC[3];
  int16_t accADC[3];
} 
imu;
static struct {
  uint8_t vbat; // battery voltage in 0.1V steps
  uint16_t intPowerMeterSum;
  uint16_t rssi; // range: [0;1023]
} 
analog;
static struct {
  int32_t EstAlt; // in cm
  int16_t vario; // variometer in cm/s
} 
alt;
static struct {
  int16_t angle[2];
  int16_t heading; // variometer in cm/s
} 
att;
static int16_t debug[4];
static int16_t sonarAlt; //to think about the unit
static int32_t BaroAlt; // in cm

struct flags_struct {
uint8_t OK_TO_ARM :    1 ;
uint8_t ARMED :    1 ;
uint8_t I2C_INIT_DONE :    1 ; // For i2c gps we have to now when i2c init is done, so we can update parameters to the i2cgps from eeprom (at startup it is done in setup())
uint8_t ACC_CALIBRATED :   1 ;
uint8_t NUNCHUKDATA :    1 ;
uint8_t ANGLE_MODE :    1 ;
uint8_t HORIZON_MODE :    1 ;
uint8_t MAG_MODE :    1 ;
uint8_t BARO_MODE :    1 ;
uint8_t GPS_HOME_MODE :    1 ;
uint8_t GPS_HOLD_MODE :    1 ;
uint8_t HEADFREE_MODE :    1 ;
uint8_t PASSTHRU_MODE :    1 ;
uint8_t GPS_FIX :    1 ;
uint8_t GPS_FIX_HOME :    1 ;
uint8_t SMALL_ANGLES_25 :    1 ;
uint8_t CALIBRATE_MAG :    1 ;
uint8_t VARIO_MODE :    1;
}
f;
//for log
static int16_t i2c_errors_count = 0;
static int16_t annex650_overrun_count = 0;
// **********************
//Automatic ACC Offset Calibration
// **********************
// **********************
// power meter
// **********************
static uint16_t intPowerTrigger1;

// **********************
// ROS
// **********************
/*
#ifdef USE_ROS
static int16_t  RosData[RC_CHANS];
static uint16_t RosDataCnt=0;
static int16_t  RosServo[NUMBER_SERVO];
static int16_t  RosServoSpeedMax[NUMBER_SERVO];
static uint16_t RosServoCnt[NUMBER_SERVO];
static int16_t  RosMotor[NUMBER_SERVO];
static int16_t  RosMotorSpeedMax[NUMBER_SERVO];
static uint16_t RosMotorCnt[NUMBER_SERVO];
#endif// USE_ROS
*/

static int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

// *************************
// motor and servo functions
// *************************
static int16_t motor[8];
static int16_t servo[8];

// ************************
// EEPROM Layout definition
// ************************
static uint8_t dynP8[3], dynD8[3];
static struct {
  uint8_t currentSet;
  int16_t accZero[3];
  int16_t magZero[3];
  uint16_t flashsum;
  uint8_t checksum; // MUST BE ON LAST POSITION OF STRUCTURE ! 
} 
global_conf;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Serial GPS only variables
//navigation mode
#define NAV_MODE_NONE          0
#define NAV_MODE_POSHOLD       1
#define NAV_MODE_WP            2
static uint8_t nav_mode = NAV_MODE_NONE; // Navigation mode
static uint8_t alarmArray[16]; // array
static int32_t baroPressure;
static int32_t baroTemperature;
static int32_t baroPressureSum;

void annexCode() { // this code is excetuted at each loop and won't interfere wbith control loop if it lasts less than 650 microseconds
  static uint32_t calibratedAccTime;
  if ( currentTime > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 100000;
    } 
    else {
      f.ACC_CALIBRATED = 1;
    }
  }
}

void setup() {
  // Serial
  SerialOpen(ROS_PORT,ROS_BAUD);
  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  BUZZERPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_OFF;

  initOutput();
  readGlobalSet();
  uint16_t i = 65000; // only first ~64K for mega board due to pgm_read_byte limitation
  uint16_t flashsum = 0;
  uint8_t pbyt;
  while(i--) {
    pbyt =  pgm_read_byte(i);        // calculate flash checksum
    flashsum += pbyt;
    flashsum ^= (pbyt<<8);
  }
  readGlobalSet(); // reload global settings for get last profile number
  if(flashsum != global_conf.flashsum) {
    global_conf.flashsum = flashsum; // new flash sum
    writeGlobalSet(1); // update flash sum in global config
  }

  blinkLED(2,40,global_conf.currentSet+1);
  initSensors();
  previousTime = micros();
  calibratingG = 512;
  calibratingB = 200; // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
  /************************************/
  /************************************/
  f.OK_TO_ARM = 0;
  f.ARMED = 0;
  f.I2C_INIT_DONE = 0;
  f.ACC_CALIBRATED = 0;
  f.SMALL_ANGLES_25=1; // important for gyro only conf

  
  //初期値がばらばらなので注意
  servo[0] = 1350;
  servo[1] = 1800;
  servo[2] = 1500;
  servo[3] = 1500;

#ifdef USE_CTRL
  ctrl_setup();
#endif// USE_CTRL

#ifdef USE_ROS
  ros_setup();
#endif// USE_ROS


}

// ******** Main Loop *********
void loop () {
/*
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  static uint8_t rcSticks; // this hold sticks position for command combos
  uint8_t axis,i;
  int16_t error,errorAngle;
  int16_t delta,deltaSum;
  int16_t PTerm,ITerm,DTerm;
  int16_t PTermACC = 0 , ITermACC = 0 , PTermGYRO = 0 , ITermGYRO = 0;
  static int16_t lastGyro[3] = {
    0,0,0        };
  static int16_t delta1[3],delta2[3];
  static int16_t errorGyroI[3] = {
    0,0,0        };
  static int16_t errorAngleI[2] = {
    0,0        };
  static int16_t initialThrottleHold;
  static uint32_t timestamp_fixated = 0;
*/
  uint8_t i;
  static uint32_t rcTime = 0;
  if (currentTime > rcTime ) { // 50Hz
    rcTime = currentTime + 20000;

    if (1) {
      for (i=0; i<NUMBER_MOTOR; i++) {
        motor[i] = constrain(motor[i], MINMOTOR, MAXMOTOR);
      }
      for (i=0; i<NUMBER_SERVO; i++) {
        servo[i] = constrain(servo[i], MINSERVO, MAXSERVO);
      }
    }

    loopCount++;
  } 
  else { // not in rc loop
    static uint8_t taskOrder=0; // never call all functions in the same loop, to avoid high delay spikes
    if(taskOrder>4) taskOrder-=5;
    switch (taskOrder) {
    case 0:
      taskOrder++;
      if (Mag_getADC()) break; // max 350  (HMC5883) // only break when we actually did something
    case 1:
      taskOrder++;
      if (Baro_update() != 0 ) break;
    case 2:
      taskOrder++;
      if (getEstimatedAltitude() !=0) break;
    case 3:
      taskOrder++;
    case 4:
      taskOrder++;
      break;
    }
  }

  // IMU update
  computeIMU();

  // Control
#ifdef USE_CTRL
  ctrl_loop();
#endif// USE_CTRL

  // Ros update
  ros_loop();

  // Measure loop rate just afer reading the sensors
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;

  // Motor & Servo
  writeServos();
  //writeMotors();
}
