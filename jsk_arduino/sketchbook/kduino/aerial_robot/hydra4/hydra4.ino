/*
  MultiWiiCopterd by Alexandre Dubus
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

//for four axis command
#include <aerial_robot_msgs/FourAxisCommand.h> 

/*********** RC alias *****************/
enum rc {
  ROLL,
  PITCH,
  YAW,  
  THROTTLE,
};


enum box {
  BOXARM,
#if ACC  
  BOXANGLE,
  BOXHORIZON,
#endif
#if MAG
  BOXMAG,
  BOXHEADFREE,
  BOXHEADADJ, // acquire heading for HEADFREE mode
#endif
#if defined(LED_FLASHER)
  BOXLEDMAX, // we want maximum illumination
  BOXLEDLOW, // low/no lights
#endif
#if defined(LANDING_LIGHTS_DDR)
  BOXLLIGHTS, // enable landing lights at any altitude
#endif
  CHECKBOXITEMS
};

const char boxnames[] PROGMEM = // names for dynamic generation of config GUI
  "ARM;"
#if ACC
  "ANGLE;"
  "HORIZON;"
#endif
#if MAG
  "MAG;"
  "HEADFREE;"
  "HEADADJ;"  
#endif
#if defined(LED_FLASHER)
  "LEDMAX;"
  "LEDLOW;"
#endif
  ;

const uint8_t boxids[] PROGMEM = {// permanent IDs associated to boxes. This way, you can rely on an ID number to identify a BOX function.
  0, //"ARM;"
#if ACC
  1, //"ANGLE;"
  2, //"HORIZON;"
#endif
#if MAG
  5, //"MAG;"
  6, //"HEADFREE;"
  7, //"HEADADJ;"  
#endif
#if defined(LED_FLASHER)
  14, //"LEDMAX;"
  15, //"LEDLOW;"
#endif
#if defined(LANDING_LIGHTS_DDR)
  16, //"LLIGHTS;"
#endif
};


static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint16_t calibratingG;

// **************
// gyro+acc IMU
// **************
static int16_t gyroZero[3] = {0,0,0};
static int16_t angle[2]    = {0,0};  // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

static struct {
  int16_t  accSmooth[3];
  int16_t  gyroData[3];
  int16_t  magADC[3];
  int16_t  gyroADC[3];
  int16_t  accADC[3];
  int16_t  accSmoothAnother[3];
} imu;

static struct {
  uint8_t  vbat;               // battery voltage in 0.1V steps
  uint16_t intPowerMeterSum;
  uint16_t rssi;               // range: [0;1023]
} analog;

static struct {
  int32_t  EstAlt;             // in cm
  int16_t  vario;              // variometer in cm/s
} alt;

static struct {
  int16_t angle[2];
  int16_t heading;              // variometer in cm/s
} att;

static struct { //bakui
  int16_t angle[2];
} att_cog;

static struct { //bakui
  int16_t  data[3];
} gyro_cog;


#if defined(ARMEDTIMEWARNING)
static uint32_t  ArmedTimeWarningMicroSeconds = 0;
#endif

static int16_t  debug[4];

struct flags_struct {
  uint8_t OK_TO_ARM :1 ;
  uint8_t ARMED :1 ;
  uint8_t I2C_INIT_DONE :1 ; // For i2c gps we have to now when i2c init is done, so we can update parameters to the i2cgps from eeprom (at startup it is done in setup())
  uint8_t ACC_CALIBRATED :1 ;
  uint8_t NUNCHUKDATA :1 ;
  uint8_t ANGLE_MODE :1 ;
  uint8_t HORIZON_MODE :1 ;
  uint8_t MAG_MODE :1 ;
  uint8_t BARO_MODE :1 ;
  uint8_t GPS_HOME_MODE :1 ;
  uint8_t GPS_HOLD_MODE :1 ;
  uint8_t HEADFREE_MODE :1 ;
  uint8_t PASSTHRU_MODE :1 ;
  uint8_t GPS_FIX :1 ;
  uint8_t GPS_FIX_HOME :1 ;
  uint8_t SMALL_ANGLES_25 :1 ;
  uint8_t CALIBRATE_MAG :1 ;
  uint8_t VARIO_MODE :1;
} f;


static int16_t  i2c_errors_count = 0;
static int16_t  annex650_overrun_count = 0;



// ******************
// rc functions
// ******************
#define MINCHECK 1100
#define MAXCHECK 1900


static int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;



// *************************
// motor and servo functions
// *************************
//static int16_t motor[8]; //by bakui, increase the size of motor balue
static uint16_t motor[8]; 
static int16_t servo[8] = {2000,1060,2000,1500,1500,1500,1500,1500}; //changed by bakui


static struct {
  uint8_t currentSet;
  int16_t accZero[3];
  int16_t magZero[3];
  uint16_t flashsum; 
  uint8_t checksum;      // MUST BE ON LAST POSITION OF STRUCTURE ! 
} global_conf;

struct servo_conf_ {  // this is a generic way to configure a servo, every multi type with a servo should use it
  int16_t min;        // minimum value, must be more than 1020 with the current implementation
  int16_t max;        // maximum value, must be less than 2000 with the current implementation
  int16_t middle;     // default should be 1500
  int8_t  rate;       // range [-100;+100] ; can be used to ajust a rate 0-100% and a direction
};

static struct {
  //  uint8_t checkNewConf;   not used anymore, should be removed
  servo_conf_ servoConf[8];
#if defined (FAILSAFE)
  int16_t failsafe_throttle;
#endif
#ifdef CYCLETIME_FIXATED
  uint16_t cycletime_fixated;
#endif
#ifdef ARMEDTIMEWARNING
  uint16_t armedtimewarning;
#endif
  int16_t minthrottle;
  uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE ! 
} conf;



static uint8_t alarmArray[16];           // array

//for hydra, lqi
//#define F_PWM_RATE 5.4522 // 0.3029 * 18
//#define F_PWM_OFFSET -21.196
static int16_t command_angles[2] = {0,0};
static int32_t yaw_pi_term[4] = {0,0,0,0};
static uint16_t throttle_pid_term[4] = {MINCOMMAND,MINCOMMAND,MINCOMMAND,MINCOMMAND};
//gain
static  int32_t p_gain[4][2]; //each motor, rpy
static  int32_t i_gain[4][2]; //each motor, rpy
static  int32_t d_gain[4][3]; //each motor, rpy

static int32_t motor_rpy_force[4] = {0,0,0,0};
static uint8_t integrate_flag = 0;

#if 1
static  int32_t roll_p_term[4] = {0,0,0,0};
static  int32_t roll_i_term[4] = {0,0,0,0};
static  int32_t roll_d_term[4] = {0,0,0,0};
static  int32_t pitch_p_term[4] = {0,0,0,0};
static  int32_t pitch_i_term[4] = {0,0,0,0};
static  int32_t pitch_d_term[4] = {0,0,0,0};
// static  int32_t yaw_p_term[4] = {0,0,0,0};
// static  int32_t yaw_i_term[4] = {0,0,0,0};
static  int32_t yaw_d_term[4] = {0,0,0,0};
#endif


static float error_angle_i[2] = {0,0};

//angle
static int16_t rotateAngle[2] = {1024, 0}; //cos_theta, sin_theta



void annexCode() { // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t calibratedAccTime;
  uint8_t axis;

  if ( (calibratingA>0 && ACC ) || (calibratingG>0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {LEDPIN_OFF;}
    if (f.ARMED) {LEDPIN_ON;}
  }

#if defined(LED_RING)
  static uint32_t LEDTime;
  if ( currentTime > LEDTime ) {
    LEDTime = currentTime + 50000;
    i2CLedRingState();
  }
#endif

#if defined(LED_FLASHER)
  auto_switch_led_flasher();
#endif

  if ( currentTime > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 100000;
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }

#if !(defined(SPEKTRUM) && defined(PROMINI))  //Only one serial port on ProMini.  Skip serial com if Spektrum Sat in use. Note: Spek code will auto-call serialCom if GUI data detected on serial0.
  serialCom();
#endif

  if (f.ARMED)  
    {
#if defined(ARMEDTIMEWARNING)
      armedTime += (uint32_t)cycleTime;
#endif
    }
}


void setup() {
#if !defined(GPS_PROMINI)
  SerialOpen(SERIAL_COM_PORT,SERIAL0_COM_SPEED);
  //    SerialOpen(0,SERIAL0_COM_SPEED);
#if defined(MEGA)
  SerialOpen(1,SERIAL1_COM_SPEED);
  SerialOpen(2,SERIAL2_COM_SPEED);
  SerialOpen(3,SERIAL3_COM_SPEED);
#endif
#endif
  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_OFF;
  initOutput();
  readGlobalSet();

  uint16_t i = 65000;                             // only first ~64K for mega board due to pgm_read_byte limitation
  uint16_t flashsum = 0;
  uint8_t pbyt;
  while(i--) {
    pbyt =  pgm_read_byte(i);        // calculate flash checksum
    flashsum += pbyt;
    flashsum ^= (pbyt<<8);
  }
#ifdef MULTIPLE_CONFIGURATION_PROFILES
  global_conf.currentSet=2;
#else
  global_conf.currentSet=0;
#endif
  while(1) {                                                    // check settings integrity
    if(readEEPROM()) {                                          // check current setting integrity
      if(flashsum != global_conf.flashsum) update_constants();  // update constants if firmware is changed and integrity is OK
    }
    if(global_conf.currentSet == 0) break;                      // all checks is done
    global_conf.currentSet--;                                   // next setting for check
  }
  readGlobalSet();                            // reload global settings for get last profile number
  if(flashsum != global_conf.flashsum) {
    global_conf.flashsum = flashsum;          // new flash sum
    writeGlobalSet(1);                        // update flash sum in global config
  }
  readEEPROM();                               // load setting data from last used profile
  blinkLED(2,40,global_conf.currentSet+1);          

  initSensors();
  previousTime = micros();

  calibratingG = 512;

  f.SMALL_ANGLES_25=1; // important for gyro only conf


  ros_setup();

  //lqi
  uint8_t axis;
  for(i=0; i< NUMBER_MOTOR; i++)
    {
      for(axis = 0; axis < 3; axis++)
        {
          if(axis < 2)
            {
              p_gain[i][axis] = 0;
              i_gain[i][axis] = 0;
            }
          d_gain[i][axis] = 0;
        }
    }
}

void go_arm() {
  if(calibratingG == 0 && f.ACC_CALIBRATED 
     ) {
    if(!f.ARMED) { // arm now!
      f.ARMED = 1;
    }
  } else if(!f.ARMED) { 
    blinkLED(2,255,1);
  }
}
void go_disarm() {
  if (f.ARMED)  f.ARMED = 0;
}
void servos2Neutral() {
}

// ******** Main Loop *********
void loop () {
  uint8_t axis,i;//, rotor;

  //control term
  //for lqi
  int32_t lqi_p_term = 0;
  int32_t lqi_i_term = 0;
  int32_t lqi_d_term = 0;


  static uint32_t rcTime  = 0;
  static uint32_t timestamp_fixated = 0;


  if (currentTime > rcTime ) { // 50Hz
    rcTime = currentTime + 20000;
  } else { // not in rc loop
    static uint8_t taskOrder=0; // never call all functions in the same loop, to avoid high delay spikes
    if(taskOrder>4) taskOrder-=5;
    switch (taskOrder) {
    case 0:
      taskOrder++;
#if MAG
      if (Mag_getADC()) break; // max 350 µs (HMC5883) // only break when we actually did something
#endif
    case 1:
      taskOrder++;
    case 2:
      taskOrder++;
    case 3:
      taskOrder++;
    case 4:
      taskOrder++;
      break;
    }
  }


  ros_loop();//ros

  computeIMU();
  // Measure loop rate just afer reading the sensors
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;

#ifdef CYCLETIME_FIXATED
  if (conf.cycletime_fixated) {
    if ((micros()-timestamp_fixated)>conf.cycletime_fixated) {
    } else {
      while((micros()-timestamp_fixated)<conf.cycletime_fixated) ; // waste away
    }
    timestamp_fixated=micros();
  }
#endif

  //lqi
  if(f.ARMED)
    {

      for(i = 0; i < NUMBER_MOTOR; i++)
        {
          motor_rpy_force[i] = yaw_pi_term[i]; //x10000

          for(axis=0; axis < 3; axis++)
            {
              if(axis < 2)
                {
                  //error_angle_i
                  if(i == 0 && integrate_flag == 1)
                    error_angle_i[axis] += ((float)(command_angles[axis] - att_cog.angle[axis]) * (float)cycleTime / 1000000.0f);

                  lqi_p_term = att_cog.angle[axis] * p_gain[i][axis];//x10000

                  lqi_i_term = (int32_t)(error_angle_i[axis] * (float)i_gain[i][axis]);//x10000
                }
              else
                { lqi_p_term = 0; lqi_i_term = 0; }

              //lqi_d_term = gyro_cog.data[axis] * d_gain[i][axis]; //x10000
              lqi_d_term = constrain(gyro_cog.data[axis] * d_gain[i][axis], -1500000,1500000); //can be more strict


              motor_rpy_force[i] += (lqi_p_term + lqi_i_term + lqi_d_term);

#if 1
              //debug
              if(axis == 0)
                {
                  roll_p_term[i] = lqi_p_term;
                  roll_i_term[i] = lqi_i_term;
                  roll_d_term[i] = lqi_d_term;
                }
              if(axis == 1)
                {
                  pitch_p_term[i] = lqi_p_term;
                  pitch_i_term[i] = lqi_i_term;
                  pitch_d_term[i] = lqi_d_term;
                }
              if(axis == 2)
                yaw_d_term[i] = motor_rpy_force[i];//lqi_d_term;
#endif
            }
          motor[i] = (uint16_t)((int16_t)throttle_pid_term[i] + (int16_t)(motor_rpy_force[i]/10000));
        }
    }

  motorValueCheck();
  writeMotors();
}


/********************************************************************/
/****                         LED Handling                       ****/
/********************************************************************/
//Beware!! Is working with delays, do not use inflight!

void blinkLED(uint8_t num, uint8_t ontime,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
#if defined(LED_FLASHER)
      switch_led_flasher(1);
#endif
#if defined(LANDING_LIGHTS_DDR)
      switch_landing_lights(1);
#endif
      LEDPIN_TOGGLE; // switch LEDPIN state
      delay(ontime);
#if defined(LED_FLASHER)
      switch_led_flasher(0);
#endif
#if defined(LANDING_LIGHTS_DDR)
      switch_landing_lights(0);
#endif
    }
    delay(60); //wait 60 ms
  }
}


void resetCommand()
{
  command_angles[ROLL] = 0;
  command_angles[PITCH] = 0;
  error_angle_i[ROLL] = 0;
  error_angle_i[PITCH] = 0;

  int i;
  for(i = 0; i < NUMBER_MOTOR; i++)
    {
      yaw_pi_term[i] = 0;
      throttle_pid_term[i] = MINCOMMAND;
    }

  integrate_flag = 0;
}
