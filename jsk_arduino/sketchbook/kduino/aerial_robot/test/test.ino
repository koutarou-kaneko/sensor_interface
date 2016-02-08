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

/*********** RC alias *****************/
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

enum pid {
  PIDROLL,
  PIDPITCH,
  PIDYAW,
  PIDALT,
  PIDPOS,
  PIDPOSR,
  PIDNAVR,
  PIDLEVEL,
  PIDMAG,
  PIDVEL,     // not used currently
  PIDITEMS
};

const char pidnames[] PROGMEM =
  "ROLL;"
  "PITCH;"
  "YAW;"
  "ALT;"
  "Pos;"
  "PosR;"
  "NavR;"
  "LEVEL;"
  "MAG;"
  "VEL;"
;

enum box {
  BOXARM,
  #if ACC  
    BOXANGLE,
    BOXHORIZON,
  #endif
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    BOXBARO,
  #endif
  #ifdef VARIOMETER
    BOXVARIO,
  #endif
  #if MAG
    BOXMAG,
    BOXHEADFREE,
    BOXHEADADJ, // acquire heading for HEADFREE mode
  #endif
  #if defined(_TILT) || defined(GIMBAL)  || defined(SERVO_MIX_TILT)
    BOXCAMSTAB,
  #endif
  #if defined(CAMTRIG)
    BOXCAMTRIG,
  #endif
  #if GPS
    BOXGPSHOME,
    BOXGPSHOLD,
  #endif
  #if defined(FIXEDWING) || defined(HELICOPTER)
    BOXPASSTHRU,
  #endif
  #if defined(BUZZER)
    BOXBEEPERON,
  #endif
  #if defined(LED_FLASHER)
    BOXLEDMAX, // we want maximum illumination
    BOXLEDLOW, // low/no lights
  #endif
  #if defined(LANDING_LIGHTS_DDR)
    BOXLLIGHTS, // enable landing lights at any altitude
  #endif
  #ifdef INFLIGHT_ACC_CALIBRATION
    BOXCALIB,
  #endif
  #ifdef GOVERNOR_P
    BOXGOV,
  #endif
  #ifdef OSD_SWITCH
    BOXOSD,
  #endif
  CHECKBOXITEMS
};

const char boxnames[] PROGMEM = // names for dynamic generation of config GUI
  "ARM;"
  #if ACC
    "ANGLE;"
    "HORIZON;"
  #endif
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    "BARO;"
  #endif
  #ifdef VARIOMETER
    "VARIO;"
  #endif
  #if MAG
    "MAG;"
    "HEADFREE;"
    "HEADADJ;"  
  #endif
  #if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
    "CAMSTAB;"
  #endif
  #if defined(CAMTRIG)
    "CAMTRIG;"
  #endif
  #if GPS
    "GPS HOME;"
    "GPS HOLD;"
  #endif
  #if defined(FIXEDWING) || defined(HELICOPTER)
    "PASSTHRU;"
  #endif
  #if defined(BUZZER)
    "BEEPER;"
  #endif
  #if defined(LED_FLASHER)
    "LEDMAX;"
    "LEDLOW;"
  #endif
  #if defined(LANDING_LIGHTS_DDR)
    "LLIGHTS;"
  #endif
  #ifdef INFLIGHT_ACC_CALIBRATION
    "CALIB;"
  #endif
  #ifdef GOVERNOR_P
    "GOVERNOR;"
  #endif
  #ifdef OSD_SWITCH
    "OSD SW;"
  #endif
;

const uint8_t boxids[] PROGMEM = {// permanent IDs associated to boxes. This way, you can rely on an ID number to identify a BOX function.
  0, //"ARM;"
  #if ACC
    1, //"ANGLE;"
    2, //"HORIZON;"
  #endif
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    3, //"BARO;"
  #endif
  #ifdef VARIOMETER
    4, //"VARIO;"
  #endif
  #if MAG
    5, //"MAG;"
    6, //"HEADFREE;"
    7, //"HEADADJ;"  
  #endif
  #if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
    8, //"CAMSTAB;"
  #endif
  #if defined(CAMTRIG)
    9, //"CAMTRIG;"
  #endif
  #if GPS
    10, //"GPS HOME;"
    11, //"GPS HOLD;"
  #endif
  #if defined(FIXEDWING) || defined(HELICOPTER)
    12, //"PASSTHRU;"
  #endif
  #if defined(BUZZER)
    13, //"BEEPER;"
  #endif
  #if defined(LED_FLASHER)
    14, //"LEDMAX;"
    15, //"LEDLOW;"
  #endif
  #if defined(LANDING_LIGHTS_DDR)
    16, //"LLIGHTS;"
  #endif
  #ifdef INFLIGHT_ACC_CALIBRATION
    17, //"CALIB;"
  #endif
  #ifdef GOVERNOR_P
    18, //"GOVERNOR;"
  #endif
  #ifdef OSD_SWITCH
    19, //"OSD_SWITCH;"
  #endif
};


static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint16_t calibratingB = 0;  // baro calibration = get new ground pressure value
static uint16_t calibratingG;
static int16_t  magHold,headFreeModeHold; // [-180;+180]
static uint8_t  vbatMin = VBATNOMINAL;  // lowest battery voltage in 0.1V steps
static uint8_t  rcOptions[CHECKBOXITEMS];
static int32_t  BaroAlt,AltHold; // in cm
static int16_t  BaroPID = 0;
static int16_t  errorAltitudeI = 0;
static uint16_t  laser_altitude = 0;

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

#if defined(ARMEDTIMEWARNING)
  static uint32_t  ArmedTimeWarningMicroSeconds = 0;
#endif

static int16_t  debug[4];
static int16_t  sonarAlt; //to think about the unit

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


// **********************
// power meter
// **********************
static uint16_t intPowerTrigger1;


// ******************
// rc functions
// ******************
#define MINCHECK 1100
#define MAXCHECK 1900

#define ROL_LO  (1<<(2*ROLL))
#define ROL_CE  (3<<(2*ROLL))
#define ROL_HI  (2<<(2*ROLL))
#define PIT_LO  (1<<(2*PITCH))
#define PIT_CE  (3<<(2*PITCH))
#define PIT_HI  (2<<(2*PITCH))
#define YAW_LO  (1<<(2*YAW))
#define YAW_CE  (3<<(2*YAW))
#define YAW_HI  (2<<(2*YAW))
#define THR_LO  (1<<(2*THROTTLE))
#define THR_CE  (3<<(2*THROTTLE))
#define THR_HI  (2<<(2*THROTTLE))

static int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

static int16_t rcData[RC_CHANS];    // interval [1000;2000]
static int16_t rcCommand[4];        // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 
static int16_t lookupPitchRollRC[5];// lookup table for expo & RC rate PITCH+ROLL
static int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE

#if defined(SPEKTRUM)
  volatile uint8_t  spekFrameFlags;
  volatile uint32_t spekTimeLast;
#endif

#if defined(OPENLRSv2MULTI)
  static uint8_t pot_P,pot_I; // OpenLRS onboard potentiometers for P and I trim or other usages
#endif


// *************************
// motor and servo functions
// *************************
static int16_t axisPID[3];
static int16_t motor[8];
static int16_t servo[8] = {2000,1060,2000,1500,1500,1500,1500,1500}; //changed by bakui

// ************************
// EEPROM Layout definition
// ************************
static uint8_t dynP8[3], dynD8[3];

static struct {
  uint8_t currentSet;
  int16_t accZero[3];
  int16_t magZero[3];
  uint16_t flashsum; 
  uint8_t checksum;      // MUST BE ON LAST POSITION OF STRUCTURE ! 
} global_conf;

struct pid_ {
  uint8_t P8;
  uint8_t I8;
  uint8_t D8;
};

struct servo_conf_ {  // this is a generic way to configure a servo, every multi type with a servo should use it
  int16_t min;        // minimum value, must be more than 1020 with the current implementation
  int16_t max;        // maximum value, must be less than 2000 with the current implementation
  int16_t middle;     // default should be 1500
  int8_t  rate;       // range [-100;+100] ; can be used to ajust a rate 0-100% and a direction
};

static struct {
//  uint8_t checkNewConf;   not used anymore, should be removed
  pid_    pid[PIDITEMS];
  uint8_t rcRate8;
  uint8_t rcExpo8;
  uint8_t rollPitchRate;
  uint8_t yawRate;
  uint8_t dynThrPID;
  uint8_t thrMid8;
  uint8_t thrExpo8;
  int16_t angleTrim[2];
  uint16_t activate[CHECKBOXITEMS];
  uint8_t powerTrigger1;
  servo_conf_ servoConf[8];
  #if defined(GYRO_SMOOTHING)
    uint8_t Smoothing[3];
  #endif
  #if defined (FAILSAFE)
    int16_t failsafe_throttle;
  #endif
  int16_t minthrottle;
  #ifdef GOVERNOR_P
   int16_t governorP;
   int16_t governorD;
   int8_t  governorR;
  #endif
  uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE ! 
} conf;


// **********************
// GPS common variables
// **********************
  static int16_t  GPS_angle[2] = { 0, 0};                      // the angles that must be applied for GPS correction
  static int32_t  GPS_coord[2];
  static int32_t  GPS_home[2];
  static int32_t  GPS_hold[2];
  static uint8_t  GPS_numSat;
  static uint16_t GPS_distanceToHome;                          // distance to home  - unit: meter
  static int16_t  GPS_directionToHome;                         // direction to home - unit: degree
  static uint16_t GPS_altitude;                                // GPS altitude      - unit: meter
  static uint16_t GPS_speed;                                   // GPS speed         - unit: cm/s
  static uint8_t  GPS_update = 0;                              // a binary toogle to distinct a GPS position update
  static uint16_t GPS_ground_course = 0;                       //                   - unit: degree*10
  static uint8_t  GPS_Present = 0;                             // Checksum from Gps serial
  static uint8_t  GPS_Enable  = 0;

  #define LAT  0
  #define LON  1
  // The desired bank towards North (Positive) or South (Negative) : latitude
  // The desired bank towards East (Positive) or West (Negative)   : longitude
  static int16_t  nav[2];
  static int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

  // default POSHOLD control gains
  #define POSHOLD_P              .11
  #define POSHOLD_I              0.0
  #define POSHOLD_IMAX           20        // degrees

  #define POSHOLD_RATE_P         2.0
  #define POSHOLD_RATE_I         0.08      // Wind control
  #define POSHOLD_RATE_D         0.045     // try 2 or 3 for POSHOLD_RATE 1
  #define POSHOLD_RATE_IMAX      20        // degrees

  // default Navigation PID gains
  #define NAV_P                  1.4
  #define NAV_I                  0.20      // Wind control
  #define NAV_D                  0.08      //
  #define NAV_IMAX               20        // degrees

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Serial GPS only variables
  //navigation mode
  #define NAV_MODE_NONE          0
  #define NAV_MODE_POSHOLD       1
  #define NAV_MODE_WP            2
  static uint8_t nav_mode = NAV_MODE_NONE; // Navigation mode

  static uint8_t alarmArray[16];           // array
 
#if BARO
  static int32_t baroPressure;
  static int32_t baroTemperature;
  static int32_t baroPressureSum;
#endif

void annexCode() {
#ifdef USE_ROS
  ros_loop();
#endif
// USE_ROS

}

void setup() {
  #if !defined(GPS_PROMINI)
    SerialOpen(SERIAL_COM_PORT,SERIAL0_COM_SPEED);
//    SerialOpen(0,SERIAL0_COM_SPEED);
    #if defined(PROMICRO)
      SerialOpen(1,SERIAL1_COM_SPEED);
    #endif
    #if defined(MEGA)
      SerialOpen(1,SERIAL1_COM_SPEED);
      SerialOpen(2,SERIAL2_COM_SPEED);
      SerialOpen(3,SERIAL3_COM_SPEED);
    #endif
  #endif
  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  BUZZERPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_OFF;

  readGlobalSet();
  #if defined(MEGA)
    uint16_t i = 65000;                             // only first ~64K for mega board due to pgm_read_byte limitation
  #else
    uint16_t i = 32000;
  #endif
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
  // while(1) {                                                    // check settings integrity
  //   if(readEEPROM()) {                                          // check current setting integrity
  //     if(flashsum != global_conf.flashsum) update_constants();  // update constants if firmware is changed and integrity is OK
  //   }
  //   if(global_conf.currentSet == 0) break;                      // all checks is done
  //   global_conf.currentSet--;                                   // next setting for check
  // }
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
  calibratingB = 200;  // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
  /************************************/

  #ifdef FASTER_ANALOG_READS
    ADCSRA |= _BV(ADPS2) ; ADCSRA &= ~_BV(ADPS1); ADCSRA &= ~_BV(ADPS0); // this speeds up analogRead without loosing too much resolution: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11
  #endif

  f.SMALL_ANGLES_25=1; // important for gyro only conf

#ifdef USE_ROS
  ros_setup();
#endif// USE_ROS

#if 1  //add by bakui , for angle control(acc mode)
      conf.activate[BOXANGLE] = 0xffff;
#endif
  
}

void go_arm() {
  if(calibratingG == 0 && f.ACC_CALIBRATED 
  #if defined(FAILSAFE)
    && failsafeCnt < 2
  #endif
    ) {
    if(!f.ARMED) { // arm now!
      f.ARMED = 1;
      headFreeModeHold = att.heading;
    }
  } else if(!f.ARMED) { 
    blinkLED(2,255,1);
    alarmArray[8] = 1;
  }
}
void go_disarm() {
  if (f.ARMED) {
    f.ARMED = 0;
    #ifdef LOG_PERMANENT
      plog.disarm++;        // #disarm events
      plog.armed_time = armedTime ;   // lifetime in seconds
      if (failsafeEvents) plog.failsafe++;      // #acitve failsafe @ disarm
      if (i2c_errors_count > 10) plog.i2c++;           // #i2c errs @ disarm
      plog.running = 0;       // toggle @ arm & disarm to monitor for clean shutdown vs. powercut
      // write now.
      writePLog();
    #endif
  }
}

// ******** Main Loop *********
void loop () {
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  static uint8_t rcSticks;       // this hold sticks position for command combos
  uint8_t axis,i;
  int16_t error,errorAngle;
  int16_t delta,deltaSum;
  int16_t PTerm,ITerm,DTerm;
  int16_t PTermACC = 0 , ITermACC = 0 , PTermGYRO = 0 , ITermGYRO = 0;
  static int16_t lastGyro[3] = {0,0,0};
  static int16_t delta1[3],delta2[3];
  static int16_t errorGyroI[3] = {0,0,0};
  static int16_t errorAngleI[2] = {0,0};
  static uint32_t rcTime  = 0;
  static int16_t initialThrottleHold;
  static uint32_t timestamp_fixated = 0;



  if (currentTime > rcTime ) { // 50Hz
    rcTime = currentTime + 20000;


    // note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAFE_DELAY is always false
    #if ACC
      if ( rcOptions[BOXANGLE] || (failsafeCnt > 5*FAILSAFE_DELAY) ) { 
        // bumpless transfer to Level mode
        if (!f.ANGLE_MODE) {
          errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
          f.ANGLE_MODE = 1;
        }  
      } else {
        // failsafe support
        f.ANGLE_MODE = 0;
      }
      if ( rcOptions[BOXHORIZON] ) {
        f.ANGLE_MODE = 0;
        if (!f.HORIZON_MODE) {
          errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
          f.HORIZON_MODE = 1;
        }
      } else {
        f.HORIZON_MODE = 0;
      }
    #endif

    if (rcOptions[BOXARM] == 0) f.OK_TO_ARM = 1;
    #if !defined(GPS_LED_INDICATOR)
      if (f.ANGLE_MODE || f.HORIZON_MODE) {STABLEPIN_ON;} else {STABLEPIN_OFF;}
    #endif

    #if BARO
      #if (!defined(SUPPRESS_BARO_ALTHOLD))
        if (rcOptions[BOXBARO]) {
            if (!f.BARO_MODE) {
              f.BARO_MODE = 1;
              AltHold = alt.EstAlt;
              initialThrottleHold = rcCommand[THROTTLE];
              errorAltitudeI = 0;
              BaroPID=0;
            }
        } else {
            f.BARO_MODE = 0;
        }
      #endif
    #endif
    #if MAG
      if (rcOptions[BOXMAG]) {
        if (!f.MAG_MODE) {
          f.MAG_MODE = 1;
          magHold = att.heading;
        }
      } else {
        f.MAG_MODE = 0;
      }
      if (rcOptions[BOXHEADFREE]) {
        if (!f.HEADFREE_MODE) {
          f.HEADFREE_MODE = 1;
        }
      } else {
        f.HEADFREE_MODE = 0;
      }
      if (rcOptions[BOXHEADADJ]) {
        headFreeModeHold = att.heading; // acquire new heading
      }
    #endif
 
  } else { // not in rc loop
  }
 

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

 
}

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
