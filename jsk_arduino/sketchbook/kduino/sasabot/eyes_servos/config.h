/*************************************************************************************************/
/****           CONFIGURABLE PARAMETERS                                                       ****/
/*************************************************************************************************/

/* this file consists of several sections
 * to create a working combination you must at least make your choices in section 1.
 * 1 - BASIC SETUP - you must select an option in every block.
 *      this assumes you have 4 channels connected to your board with standard ESCs and servos.
 * 2 - COPTER TYPE SPECIFIC OPTIONS - you likely want to check for options for your copter type
 * 3 - RC SYSTEM SETUP
 * 4 - ALTERNATE CPUs & BOARDS - if you have
 * 5 - ALTERNATE SETUP - select alternate RX (SBUS, PPM, etc.), alternate ESC-range, etc. here
 * 6 - OPTIONAL FEATURES - enable nice to have features here (FlightModes, LCD, telemetry, battery monitor etc.)
 * 7 - TUNING & DEVELOPER - if you know what you are doing; you have been warned
 */


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  0 - JSK SETUP                                                *******/
/*****************                                                                 ***************/
/*************************************************************************************************/
#define JSK_IMU		// JSK IMU
#define USE_ROS           // rosserial
#define USE_CTRL
#define USE_TEST

#define SERVO
#define NUMBER_MOTOR 0
#define NUMBER_SERVO 8

//#define MOTOR_TEST  // motor test
//#define SERVO_TEST  // servo test

//#define ROS_CB_SET  // ROS set servo and motor in Callback

//    +---LED---+      ^ X
//    |         |      |
//    |         USB  Z x -> Y
//    |         |
//    +---------+
//#define AXIS_ORIENTATION(v)  AXIS_ORIENTATION_Xm90(v)  // usb up to sky
//#define AXIS_ORIENTATION(v)  AXIS_ORIENTATION_Xp90(v)  // usb down to ground
//#define AXIS_ORIENTATION(v)  AXIS_ORIENTATION_Yp90(v)  // led up to sky
//#define AXIS_ORIENTATION(v)  AXIS_ORIENTATION_Ym90(v)  // led down to ground
//#define AXIS_ORIENTATION(v)  AXIS_ORIENTATION_X180(v)  // upside down

// Serial Port (0:USB,3:XBEE)
// Serial Port (0:USB,2:ICS,3:XBEE)
#define ICS_PORT    2
#define ICS_BAUD    115200
//#define USB_BAUD    57600
#define USB_BAUD    115200
#define XBEE_BAUD   57600

#if 1
// MSP:XBEE
// ROS:USB
#define ROS_PORT 0
#define ROS_BAUD   USB_BAUD
#define MSP_PORT 3
#define MSP_BAUD   XBEE_BAUD
#else
// MSP:USB
// ROS:XBEE
#define ROS_PORT 3
#define ROS_BAUD   XBEE_BAUD
#define MSP_PORT  0
#define MSP_BAUD   USB_BAUD
#endif

#define MINMOTOR 1000
#define MIDMOTOR 1500
#define MAXMOTOR 2000
#define INIMOTOR MIDMOTOR

#define MINSERVO 1000
#define MIDSERVO 1500
#define MAXSERVO 2000
#define INISERVO MIDSERVO

// Axis Convert MultiWii - Right Hand System
#define AXIS_CONV_ACC_M2R(v)   {int16_t tmp=v[0];v[0]=v[1];v[1]=tmp;}
#define AXIS_CONV_ACC_R2M(v)   {int16_t tmp=v[1];v[1]=v[0];v[0]=tmp;}
#define AXIS_CONV_GYRO_M2R(v)  {v[1]=-v[1];}
#define AXIS_CONV_GYRO_R2M(v)  {v[1]=-v[1];}
#define AXIS_CONV_MAG_M2R(v)   {int16_t tmp=v[0];v[0]=v[1];v[1]=tmp;}
#define AXIS_CONV_MAG_R2M(v)   {int16_t tmp=v[1];v[1]=v[0];v[0]=tmp;}

// check
#if ((NUMBER_SERVO+NUMBER_MOTOR)>13)
#error "Error:(NUMBER_SERVO+NUMBER_MOTOR)>13"
#endif

#ifdef MOTOR_TEST
#undef ROS_CB_SET
#endif// MOTOR_TEST
#ifdef SERVO_TEST
#undef ROS_CB_SET
#endif// SERVO_TEST

  /**************************************************************************************/
  /********                              Gyro filters                ********************/
  /**************************************************************************************/

    /*********************    Lowpass filter for some gyros    ****************************/
      /* ITG3200 & ITG3205 Low pass filter setting. In case you cannot eliminate all vibrations to the Gyro, you can try
         to decrease the LPF frequency, only one step per try. As soon as twitching gone, stick with that setting.
         It will not help on feedback wobbles, so change only when copter is randomly twiching and all dampening and
         balancing options ran out. Uncomment only one option!
         IMPORTANT! Change low pass filter setting changes PID behaviour, so retune your PID's after changing LPF.*/
      //#define ITG3200_LPF_256HZ     // This is the default setting, no need to uncomment, just for reference
      //#define ITG3200_LPF_188HZ
      //#define ITG3200_LPF_98HZ
#define ITG3200_LPF_42HZ
      //#define ITG3200_LPF_20HZ
      //#define ITG3200_LPF_10HZ      // Use this only in extreme cases, rather change motors and/or props

      /* MPU6050 Low pass filter setting. In case you cannot eliminate all vibrations to the Gyro, you can try
         to decrease the LPF frequency, only one step per try. As soon as twitching gone, stick with that setting.
         It will not help on feedback wobbles, so change only when copter is randomly twiching and all dampening and
         balancing options ran out. Uncomment only one option!
         IMPORTANT! Change low pass filter setting changes PID behaviour, so retune your PID's after changing LPF.*/
      //#define MPU6050_LPF_256HZ     // This is the default setting, no need to uncomment, just for reference
      //#define MPU6050_LPF_188HZ
      //#define MPU6050_LPF_98HZ
#define MPU6050_LPF_42HZ
      //#define MPU6050_LPF_20HZ
      //#define MPU6050_LPF_10HZ
      //#define MPU6050_LPF_5HZ       // Use this only in extreme cases, rather change motors and/or props

    /******                Gyro smoothing    **********************************/
      /* GYRO_SMOOTHING. In case you cannot reduce vibrations _and_ _after_ you have tried the low pass filter options, you
         may try this gyro smoothing via averaging. Not suitable for multicopters!
         Good results for helicopter, airplanes and flying wings (foamies) with lots of vibrations.*/
      //#define GYRO_SMOOTHING {20, 20, 3}    // separate averaging ranges for roll, pitch, yaw

    /************************    Moving Average Gyros    **********************************/
      //#define MMGYRO 10                      // Active Moving Average Function for Gyros
      //#define MMGYROVECTORLENGTH 15          // Length of Moving Average Vector (maximum value for tunable MMGYRO
      /* Moving Average ServoGimbal Signal Output */
      //#define MMSERVOGIMBAL                  // Active Output Moving Average Function for Servos Gimbal
      //#define MMSERVOGIMBALVECTORLENGHT 32   // Lenght of Moving Average Vector

  /************************    Analog Reads              **********************************/
    /* if you want faster analog Reads, enable this. It may result in less accurate results, especially for more than one analog channel */
    //#define FASTER_ANALOG_READS


    /* Get your magnetic decliniation from here : http://magnetic-declination.com/
       Convert the degree+minutes into decimal degree by ==> degree+minutes*(1/60)
       Note the sign on declination it could be negative or positive (WEST or EAST) */
    //#define MAG_DECLINIATION  3.96f              //For Budapest Hungary.
    //#define MAG_DECLINIATION  0.0f
#define MAG_DECLINIATION  -7.1f

  /********************************************************************/
  /****           battery voltage monitoring                       ****/
  /********************************************************************/
    /* for V BAT monitoring
       after the resistor divisor we should get [0V;5V]->[0;1023] on analog V_BATPIN
       with R1=33k and R2=51k
       vbat = [0;1023]*16/VBATSCALE
       must be associated with #define BUZZER ! */
    //#define VBAT              // uncomment this line to activate the vbat code
    #define VBATSCALE       131 // change this value if readed Battery voltage is different than real voltage
    #define VBATNOMINAL     126 // 12,6V full battery nominal voltage - only used for lcd.telemetry
    #define VBATLEVEL_WARN1 107 // 10,7V
    #define VBATLEVEL_WARN2  99 // 9.9V
    #define VBATLEVEL_CRIT   93 // 9.3V - critical condition: if vbat ever goes below this value, permanent alarm is triggered
    #define NO_VBAT          16 // Avoid beeping without any battery


  /********************************************************************/
  /****           powermeter (battery capacity monitoring)         ****/
  /********************************************************************/

    /* enable monitoring of the power consumption from battery (think of mAh)
       allows to set alarm value in GUI or via LCD
      Full description and howto here http://www.multiwii.com/wiki/index.php?title=Powermeter
       Two options:
       1 - hard: - (uses hardware sensor, after configuration gives very good results)
       2 - soft: - (good results +-5% for plush and mystery ESCs @ 2S and 3S, not good with SuperSimple ESC)    */
    //#define POWERMETER_SOFT
    //#define POWERMETER_HARD
    /* PLEVELSCALE is the step size you can use to set alarm */
    #define PLEVELSCALE 50 // if you change this value for other granularity, you must search for comments in code to change accordingly
    /* larger PLEVELDIV will get you smaller value for power (mAh equivalent) */
    #define PLEVELDIV 5000 // default for soft - if you lower PLEVELDIV, beware of overrun in uint32 pMeter
    //#define PLEVELDIV 36000 // fixed value for hard - do not tune
    #define PLEVELDIVSOFT PLEVELDIV // for soft always equal to PLEVELDIV
    //#define PLEVELDIVSOFT 5000 // for hard fixed to 5000
    #define PSENSORNULL 510 // set to analogRead() value for zero current; for I=0A my sensor gives 1/2 Vss; that is approx 2.49Volt;
    #define PINT2mA 132 // one integer step on arduino analog translates to mA (example 4.9 / 37 * 1000

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  7 - TUNING & DEVELOPER                                  **************/
/*****************                                                                 ***************/
/*************************************************************************************************/

/************ Experimental: force a stable, fixated (high) cycle time       **********/
/* when activated, the displayed cycle time in GUI will not be correct.
 * Tunable via LCD config menu.
 * value of 0 turns the feature off.
 */
//#define CYCLETIME_FIXATED 9000

/**************************************************************************************/
/********   special ESC with extended range [0-2000] microseconds  ********************/
/**************************************************************************************/
//#define EXT_MOTOR_RANGE

/**************************************************************************************/
/***********************     motor, servo and other presets     ***********************/
/**************************************************************************************/
/* motors will not spin when the throttle command is in low position
 this is an alternative method to stop immediately the motors */
//#define MOTOR_STOP

/* some radios have not a neutral point centered on 1500. can be changed here */
//#define MIDRC 1550
#define MIDRC 1500
//#define MIDRC 1520
#define MINRC (MIDRC-500)
#define MAXRC (MIDRC+500)

/***********************         Servo Refreshrates            ***********************/
/* Default 50Hz Servo refresh rate*/
#define SERVO_RFR_50HZ

/* up to 160Hz servo refreshrate .. works with the most analog servos*/
//#define SERVO_RFR_160HZ

/* up to 300Hz refreshrate it is as fast as possible (100-300Hz depending on the cound of used servos and the servos state).
 for use with digital servos
 dont use it with analog servos! thay may get damage. (some will work but be careful) */
//#define SERVO_RFR_300HZ

/***********************             HW PWM Servos             ***********************/
/* HW PWM Servo outputs for Arduino Mega.. moves:
 Pitch   = pin 44
 Roll    = pin 45
 CamTrig = pin 46
 SERVO4  = pin 11 (aileron left for fixed wing)
 SERVO5  = pin 12 (aileron right for fixed wing)
 SERVO6  = pin 6   (rudder for fixed wing)
 SERVO7  = pin 7   (elevator for fixed wing)
 SERVO8  = pin 8   (motor for fixed wing)       */

//#define MEGA_HW_PWM_SERVOS
#define SERVO_RFR_RATE  50    // In Hz, you can set it from 20 to 400Hz, used only in HW PWM mode

/*************************************************************************************************/
/****           END OF CONFIGURABLE PARAMETERS                                                ****/
/*************************************************************************************************/
