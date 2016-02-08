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
  #if defined(SERVO_TILT) || defined(GIMBAL)  || defined(SERVO_MIX_TILT)
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

//for log
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY)
  static uint16_t cycleTimeMax = 0;       // highest ever cycle timen
  static uint16_t cycleTimeMin = 65535;   // lowest ever cycle timen
  static uint16_t powerMax = 0;           // highest ever current;
  static int32_t  BAROaltMax;         // maximum value
#endif
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING)  || defined(LOG_PERMANENT)
  static uint32_t armedTime = 0;
#endif

static int16_t  i2c_errors_count = 0;
static int16_t  annex650_overrun_count = 0;



#if defined(THROTTLE_ANGLE_CORRECTION)
  static int16_t throttleAngleCorrection = 0;	// correction of throttle in lateral wind,
  static int8_t  cosZ = 100;					// cos(angleZ)*100
#endif



// **********************
//Automatic ACC Offset Calibration
// **********************
#if defined(INFLIGHT_ACC_CALIBRATION)
  static uint16_t InflightcalibratingA = 0;
  static int16_t AccInflightCalibrationArmed;
  static uint16_t AccInflightCalibrationMeasurementDone = 0;
  static uint16_t AccInflightCalibrationSavetoEEProm = 0;
  static uint16_t AccInflightCalibrationActive = 0;
#endif

// **********************
// power meter
// **********************
#if defined(POWERMETER) || ( defined(LOG_VALUES) && (LOG_VALUES >= 3) )
#define PMOTOR_SUM 8                     // index into pMeter[] for sum
  static uint32_t pMeter[PMOTOR_SUM + 1];  // we use [0:7] for eight motors,one extra for sum
  static uint8_t pMeterV;                  // dummy to satisfy the paramStruct logic in ConfigurationLoop()
  static uint32_t pAlarm;                  // we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
  static uint16_t powerValue = 0;          // last known current
#endif
static uint16_t intPowerTrigger1;

// **********************
// telemetry
// **********************
#if defined(LCD_TELEMETRY)
  static uint8_t telemetry = 0;
  static uint8_t telemetry_auto = 0;
#endif
#ifdef LCD_TELEMETRY_STEP
  static char telemetryStepSequence []  = LCD_TELEMETRY_STEP;
  static uint8_t telemetryStepIndex = 0;
#endif


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
  #ifdef FLYING_WING
    uint16_t wing_left_mid;
    uint16_t wing_right_mid;
  #endif
  #ifdef TRI
    uint16_t tri_yaw_middle;
  #endif
  #if defined HELICOPTER || defined(AIRPLANE)|| defined(SINGLECOPTER)|| defined(DUALCOPTER)
    int16_t servoTrim[8];
  #endif
  #if defined(GYRO_SMOOTHING)
    uint8_t Smoothing[3];
  #endif
  #if defined (FAILSAFE)
    int16_t failsafe_throttle;
  #endif
  #ifdef VBAT
    uint8_t vbatscale;
    uint8_t vbatlevel_warn1;
    uint8_t vbatlevel_warn2;
    uint8_t vbatlevel_crit;
  #endif
  #ifdef POWERMETER
    uint16_t psensornull;
    uint16_t pleveldivsoft;
    uint16_t pleveldiv;
    uint8_t pint2ma;
  #endif
  #ifdef CYCLETIME_FIXATED
    uint16_t cycletime_fixated;
  #endif
  #ifdef MMGYRO
    uint8_t mmgyro;
  #endif
  #ifdef ARMEDTIMEWARNING
    uint16_t armedtimewarning;
  #endif
  int16_t minthrottle;
  #ifdef GOVERNOR_P
   int16_t governorP;
   int16_t governorD;
   int8_t  governorR;
  #endif
  uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE ! 
} conf;

#ifdef LOG_PERMANENT
static struct {
  uint16_t arm;           // #arm events
  uint16_t disarm;        // #disarm events
  uint16_t start;         // #powercycle/reset/initialize events
  uint32_t armed_time ;   // copy of armedTime @ disarm
  uint32_t lifetime;      // sum (armed) lifetime in seconds
  uint16_t failsafe;      // #failsafe state @ disarm
  uint16_t i2c;           // #i2c errs state @ disarm
  uint8_t  running;       // toggle on arm & disarm to monitor for clean shutdown vs. powercut
  uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE !
} plog;
#endif

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
  static int32_t baroGroundPressure;
  static int32_t baroTemperature;
  static int32_t baroPressureSum;
#endif


/// this code is excetuted at each loop and
///   won't interfere with control loop if it lasts less than 650 microseconds
void annexCode() {
  static uint32_t calibratedAccTime;
  uint16_t tmp,tmp2;
  uint8_t axis,prop1,prop2;

#if defined(BUZZER)
 // external buzzer routine that handles buzzer events globally now
  alarmHandler();
#endif

  // Calibration phasis
  if ((calibratingA > 0 && ACC) || (calibratingG > 0)) {
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {
      LEDPIN_OFF;
    }
    if (f.ARMED) {
      LEDPIN_ON;
    }
  }

#if defined(LED_RING)
  static uint32_t LEDTime;
  if (currentTime > LEDTime) {
    LEDTime = currentTime + 50000;
    i2CLedRingState();
  }
#endif

#if defined(LED_FLASHER)
  auto_switch_led_flasher();
#endif

  if (currentTime > calibratedAccTime) {
    if (!f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 100000;
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }

  // Only one serial port on ProMini.
  // Skip serial com if Spektrum Sat in use.
  // Note: Spek code will auto-call serialCom if GUI data detected on serial0.
#if !(defined(SPEKTRUM) && defined(PROMINI))
#if defined(GPS_PROMINI)
  if(GPS_Enable == 0) {
    serialCom();
  }
#else
  serialCom();
#endif
#endif

#ifdef USE_ROS
	ros_loop();
#endif  // USE_ROS

#if defined(POWERMETER)
  analog.intPowerMeterSum = (pMeter[PMOTOR_SUM] / conf.pleveldiv);
  intPowerTrigger1 = conf.powerTrigger1 * PLEVELSCALE; 
#endif

#ifdef LCD_TELEMETRY_AUTO
  static char telemetryAutoSequence []  = LCD_TELEMETRY_AUTO;
  static uint8_t telemetryAutoIndex = 0;
  static uint16_t telemetryAutoTimer = 0;
  if ((telemetry_auto) &&
      (!(++telemetryAutoTimer % LCD_TELEMETRY_AUTO_FREQ))) {
    toggle_telemetry(
        telemetryAutoSequence[
            ++telemetryAutoIndex % strlen(telemetryAutoSequence)]);
    // LCDclear();  // make sure to clear away remnants
  }
#endif
  
#ifdef LCD_TELEMETRY
  static uint16_t telemetryTimer = 0;
  if (!(++telemetryTimer % LCD_TELEMETRY_FREQ)) {
#if (LCD_TELEMETRY_DEBUG + 0 > 0)
    telemetry = LCD_TELEMETRY_DEBUG;
#endif
    if (telemetry) lcd_telemetry();
  }
#endif

  // modified by MIS to use STABLEPIN LED for number of sattelites indication
#if GPS & defined(GPS_LED_INDICATOR)
  // - No GPS FIX -> LED blink at speed of incoming GPS frames
  static uint32_t GPSLEDTime;
  // - Fix and sat no. bellow 5 -> LED off
  static uint8_t blcnt;

  // - Fix and sat no. >= 5 -> LED blinks,
  //   one blink for 5 sat,
  //   two blinks for 6 sat,
  //   three for 7 ...
  if (currentTime > GPSLEDTime) {
    if (f.GPS_FIX && GPS_numSat >= 5) {
      if (++blcnt > 2 * GPS_numSat) blcnt = 0;
      GPSLEDTime = currentTime + 150000;
      if(blcnt >= 10 && ((blcnt%2) == 0)) {
        STABLEPIN_ON;
      } else {
        STABLEPIN_OFF;
      }
    }else{
      if((GPS_update == 1) && !f.GPS_FIX) {
        STABLEPIN_ON;
      } else {
        STABLEPIN_OFF;
      }
      blcnt = 0;
    }
  }
#endif  // GPS & defined(GPS_LED_INDICATOR)

#if defined(LOG_VALUES) && (LOG_VALUES >= 2)
  if (cycleTime > cycleTimeMax)
    cycleTimeMax = cycleTime;  // remember highscore
  if (cycleTime < cycleTimeMin)
    cycleTimeMin = cycleTime;  // remember lowscore
#endif
  
  if (f.ARMED) {
#if defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING) || defined(LOG_PERMANENT)
    armedTime += (uint32_t)cycleTime;
#endif
    
#if defined(VBAT)
    if ((analog.vbat > NO_VBAT) && (analog.vbat < vbatMin))
      vbatMin = analog.vbat;
#endif

#ifdef LCD_TELEMETRY
#if BARO
    if (BaroAlt > BAROaltMax)
      BAROaltMax = BaroAlt;
#endif
#endif
  }
}


void setup() {
#if !defined(GPS_PROMINI)
  SerialOpen(SERIAL_COM_PORT,SERIAL0_COM_SPEED);
  // SerialOpen(0,SERIAL0_COM_SPEED);

#if defined(PROMICRO)
  SerialOpen(1,SERIAL1_COM_SPEED);
#endif

#if defined(MEGA)
  SerialOpen(1,SERIAL1_COM_SPEED);
  SerialOpen(2,SERIAL2_COM_SPEED);
  SerialOpen(3,SERIAL3_COM_SPEED);
#endif
#endif  // GPS_PROMINI

  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  BUZZERPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_OFF;
  readGlobalSet();

#if defined(MEGA)
  // only first ~64K for mega board due to pgm_read_byte limitation
  uint16_t i = 65000;
#else
  uint16_t i = 32000;
#endif

  uint16_t flashsum = 0;
  uint8_t pbyt;
  while (i--) {
    // calculate flash checksum
    pbyt =  pgm_read_byte(i);
    flashsum += pbyt;
    flashsum ^= (pbyt << 8);
  }
  
#ifdef MULTIPLE_CONFIGURATION_PROFILES
  global_conf.currentSet = 2;
#else
  global_conf.currentSet = 0;
#endif

  // check settings integrity
  while (1) {
    // check current setting integrity
    if (readEEPROM()) {
      // update constants if firmware is changed and integrity is OK
      if(flashsum != global_conf.flashsum)
        update_constants();
    }

    // all checks is done
    if (global_conf.currentSet == 0)
      break;

    // next setting for check
    global_conf.currentSet--;
  }

  // reload global settings for get last profile number
  readGlobalSet();
  
  if (flashsum != global_conf.flashsum) {
    // new flash sum
    global_conf.flashsum = flashsum;
    // update flash sum in global config
    writeGlobalSet(1);
  }

  // load setting data from last used profile
  readEEPROM();
  blinkLED(2, 40, global_conf.currentSet + 1);          

  initSensors();

#if defined(I2C_GPS) || defined(GPS_SERIAL) || defined(GPS_FROM_OSD)
  GPS_set_pids();
#endif

  previousTime = micros();

#if defined(GIMBAL)
  calibratingA = 512;
#endif
  calibratingG = 512;
  // 10 seconds init_delay + 200 * 25 ms
  //   = 15 seconds before ground pressure settles
  calibratingB = 200;

#if defined(POWERMETER)
  for(uint8_t i = 0; i <= PMOTOR_SUM; i++)
    pMeter[i] = 0;
#endif
 
#if defined(I2C_GPS) || defined(TINY_GPS) || defined(GPS_FROM_OSD)
  GPS_Enable = 1;
#endif
  
#if defined(LCD_ETPP) || defined(LCD_LCD03) || defined(OLED_I2C_128x64) || defined(LCD_TELEMETRY_STEP)
  initLCD();
#endif

#ifdef LCD_TELEMETRY_DEBUG
  telemetry_auto = 1;
#endif

#ifdef LCD_CONF_DEBUG
  configurationLoop();
#endif

#ifdef LANDING_LIGHTS_DDR
  init_landing_lights();
#endif

#ifdef FASTER_ANALOG_READS
  // this speeds up analogRead without loosing too much resolution:
  //   http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11
  ADCSRA |= _BV(ADPS2);
  ADCSRA &= ~_BV(ADPS1);
  ADCSRA &= ~_BV(ADPS0);
#endif

#if defined(LED_FLASHER)
  init_led_flasher();
  led_flasher_set_sequence(LED_FLASHER_SEQUENCE);
#endif

  f.SMALL_ANGLES_25=1;  // important for gyro only conf

#ifdef LOG_PERMANENT
  // read last stored set
  readPLog();
  plog.lifetime += plog.armed_time / 1000000;
  plog.start++;  // #powercycle/reset/initialize events

#ifdef LOG_PERMANENT_SHOW_AT_STARTUP
  // dump plog data to terminal
  dumpPLog(0);
#endif

  // lifetime in seconds
  plog.armed_time = 0;
  // toggle on arm & disarm to monitor for clean shutdown vs. powercut
  // plog.running = 0;
#endif  // LOG_PERMANENT

#ifdef USE_ROS
  ros_setup();
#endif  // USE_ROS

  debugmsg_append_str("initialization completed\n");
}


// ******** Main Loop *********
void loop () {
  static uint32_t rcTime  = 0;
  static uint32_t timestamp_fixated = 0;

  // never call all functions in the same loop, to avoid high delay spikes
  static uint8_t taskOrder = 0;
  
  if (taskOrder > 4) taskOrder -= 5;

  switch (taskOrder) {
  case 0:
    taskOrder++;
#if MAG
    // max 350 µs (HMC5883)
    // only break when we actually did something
    if (Mag_getADC()) break;
#endif
  case 1:
    taskOrder++;
#if BARO
    if (Baro_update() != 0) break;
#endif
  case 2:
    taskOrder++;
#if BARO
    if (getEstimatedAltitude() != 0) break;
#endif    
  case 3:
    taskOrder++;
  case 4:
    taskOrder++;
    break;
  }

  computeIMU();  // defined in IMU.ino

  // Measure loop rate just afer reading the sensors
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;

#ifdef CYCLETIME_FIXATED
  if (conf.cycletime_fixated) {
    if ((micros()-timestamp_fixated)>conf.cycletime_fixated) {
    } else {
      // waste away
      while ((micros() - timestamp_fixated) < conf.cycletime_fixated);
    }
    timestamp_fixated=micros();
  }
#endif

}
