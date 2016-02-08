/*************************************************************************************************/
/****           CONFIGURABLE PARAMETERS                                                       ****/
/*************************************************************************************************/

#define USE_ROS           // rosserial on Port0
#define USE_XBEE          // XBEE for MultiWiiConf on Port3
#define USE_HARDWARESERIAL  // for normal arduino lib
#define SUPPRESS_ALL_SERIAL_MSP // saves approx 2700 bytes
#define ROS_BAUD   115200 // also for bluetooh device connection

//for servo
#define SERVO
#define NUMBER_SERVO 3
#define NUMBER_SERVO 4
#define MIDSERVO 1500

#ifdef USE_XBEE
#define SERIAL_COM_PORT 3
#else// USE_XBEE
#define SERIAL_COM_PORT 0
#endif// USE_XBEE


#ifdef USE_ROS
// not use custom serial, use HardwareSerial
//#define USE_HARDWARESERIAL
/*
YOU NEED TO EDIT HardwareSerial.h and HardWareSerial.cpp which is arduino standard library.
#define USE_HARDWARESERIAL0
//#define USE_HARDWARESERIAL1
//#define USE_HARDWARESERIAL2
//#define USE_HARDWARESERIAL3
*/
#endif// USE_ROS

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  1 - BASIC SETUP                                                *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /**************************    The type of multicopter    ****************************/
    //#define GIMBAL
    //#define BI
    //#define TRI
    //#define QUADP
#define QUADX
    //#define Y4
    //#define Y6
    //#define HEX6
    //#define HEX6X
    //#define HEX6H  // New Model
    //#define OCTOX8
    //#define OCTOFLATP
    //#define OCTOFLATX
    //#define FLYING_WING
    //#define VTAIL4
    //#define AIRPLANE
    //#define SINGLECOPTER
    //#define DUALCOPTER
    //#define HELI_120_CCPM
    //#define HELI_90_DEG

  /****************************    Motor minthrottle    *******************************/
    /* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
       This is the minimum value that allow motors to run at a idle speed  */
// #define MINTHROTTLE 1220 // for Turnigy Plush ESCs 10A 1220 is default
    //#define MINTHROTTLE 1180 // this is for JSK_ROOK_PROTOTYPE
    //#define MINTHROTTLE 1120 // for Super Simple ESCs 10A
    #define MINTHROTTLE 1064 // special ESC (simonk)
    //#define MINTHROTTLE 1050 // for brushed ESCs like ladybird
    //#define MINTHROTTLE 1150 // 

  /****************************    Motor maxthrottle    *******************************/
    /* this is the maximum value for the ESCs at full power, this value can be increased up to 2000 */
    #define MAXTHROTTLE 1900

  /****************************    Mincommand          *******************************/
    /* this is the value for the ESCs when they are not armed
       in some cases, this value must be lowered down to 900 for some specific ESCs, otherwise they failed to initiate */
#define MINCOMMAND  1030  //1000

  /**********************************    I2C speed   ************************************/
    #define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP
    //#define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones

  /***************************    Internal i2c Pullups   ********************************/
    /* enable internal I2C pull ups (in most cases it is better to use external pullups) */
    //#define INTERNAL_I2C_PULLUPS

  /**************************************************************************************/
  /*****************          boards and sensor definitions            ******************/
  /**************************************************************************************/

#define JSK_IMU		// JSK IMU


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  2 - COPTER TYPE SPECIFIC OPTIONS                               *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /********************************    TRI    *********************************/
//#define YAW_DIRECTION 1 //by bakui, 1: z axis is downward
#define YAW_DIRECTION -1 // by bakui, -1: z axis is downward, is not good, the sign should be changed, and correction the direction of moment
    /* you can change the tricopter servo travel here */
      #define TRI_YAW_CONSTRAINT_MIN 1020
      #define TRI_YAW_CONSTRAINT_MAX 2000
//#define TRI_YAW_MIDDLE 1500 //  tail servo center pos. - use this for initial trim; later trim midpoint via LCD
#define TRI_YAW_MIDDLE MIDRC // (*) tail servo center pos. - use this for initial trim; later trim midpoint via LCD

  /********************************    BI    *********************************/
    /* you can change the bicopter servo travel direction here */     
    //#define BI_PITCH_DIRECTION 1
     #define BI_PITCH_DIRECTION -1

   /********************************    ARM/DISARM    *********************************/
   /* optionally disable stick combinations to arm/disarm the motors.
     * In most cases one of the two options to arm/disarm via TX stick is sufficient */
    #define ALLOW_ARM_DISARM_VIA_TX_YAW
    //#define ALLOW_ARM_DISARM_VIA_TX_ROLL


  /***********************          Airplane                       ***********************/
    //#define USE_THROTTLESERVO // For use of standard 50Hz servo on throttle.
    #define SERVO_RATES      {100, 100, 100, 100, 100, 100, 100, 100} // Rates in 0-100%
    #define SERVO_DIRECTION  { -1,   1,   1,   -1,  1,   1,   1,   1 } // Invert servos by setting -1


  /***********************      Common for Heli & Airplane         ***********************/

    #define SERVO_OFFSET     {  0,   0,   0,  0,   0,   0,  0,   0 } //  Adjust Servo MID Offset & Swash angles

  /***********************          Heli                           ***********************/
    /* Channel to control CollectivePitch */
    #define COLLECTIVE_PITCH      THROTTLE
    /* Set Maximum available movement for the servos. Depending on model */
    #define SERVO_ENDPOINT_HIGH {2000,2000,2000,2000,2000,2000,2000,2000};
    #define SERVO_ENDPOINT_LOW  {1020,1020,1020,1020,1020,1020,1020,1020};

    /* Limit the range of Collective Pitch. 100% is Full Range each way and position for Zero Pitch */
    #define COLLECTIVE_RANGE { 80, 0, 80 }   // {Min%, ZeroPitch offset from 1500, Max%}.
    #define YAW_CENTER             MIDRC      // Use servo[5] SERVO_ENDPOINT_HIGH/LOW for the endpoits.
    #define YAWMOTOR                 0       // If a motor is used as YAW Set to 1 else set to 0.

    /* Servo mixing for heli 120 Use 1/10 fractions (ex.5 = 5/10 = 1/2)
                         {Coll,Nick,Roll} */
    #define SERVO_NICK   { +10, -10, -0 }
    #define SERVO_LEFT   { +10, +5, +10 } 
    #define SERVO_RIGHT  { +10, +5, -10 } 

    /* Servo mixing for heli 90 
                            {Coll,Nick,Roll} */
    #define SERVO_DIRECTIONS { +1, -1, -1 } // -1 will invert servo

    /* Limit Maximum controll for Roll & Nick  in 0-100% */
    #define CONTROL_RANGE   { 100, 100 }      //  { ROLL,PITCH }

    /* use servo code to drive the throttle output. You want this for analog servo driving the throttle on IC engines.
       if inactive, throttle output will be treated as a motor output, so it can drive an ESC */
    //#define HELI_USE_SERVO_FOR_THROTTLE

  /***********************      Single and DualCopter Settings     ***********************/
    /* Change to -1 to reverse servomovement per axis
       Servosettings for SingleCopter */
    #define SINGLECOPTRER_YAW   {1, 1, 1, 1} // Left, Right,Front,Rear
    #define SINGLECOPTRER_SERVO {1,-1, 1,-1} // Pitch,Pitch,Roll, Roll    
  
    /* Servosettings for DualCopter */
     #define DUALCOPTER_SERVO {1,1} //Pitch,Roll
    /* Use  SERVO_OFFSET and SERVO_RATES in Heli and Airplane section for centering and endpoints */


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  3 - RC SYSTEM SETUP                                            *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /* note: no need to uncomment something in this section if you use a standard receiver */

  /**************************************************************************************/
  /********                       special receiver types             ********************/
  /**************************************************************************************/

    /****************************    PPM Sum Reciver    ***********************************/
      /* The following lines apply only for specific receiver with only one PPM sum signal, on digital PIN 2
         Select the right line depending on your radio brand. Feel free to modify the order in your PPM order is different */
      //#define SERIAL_SUM_PPM         PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum
#define SERIAL_SUM_PPM         ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Robe/Hitec/Futaba
      //#define SERIAL_SUM_PPM         ROLL,PITCH,YAW,THROTTLE,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Multiplex
      //#define SERIAL_SUM_PPM         PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For some Hitec/Sanwa/Others

      // Uncommenting following line allow to connect PPM_SUM receiver to standard THROTTLE PIN on MEGA boards (eg. A8 in CRIUS AIO)
#define PPM_ON_THROTTLE

    /**********************    Spektrum Satellite Reciver    *******************************/
      /* The following lines apply only for Spektrum Satellite Receiver
         Spektrum Satellites are 3V devices.  DO NOT connect to 5V!
         For MEGA boards, attach sat grey wire to RX1, pin 19. Sat black wire to ground. Sat orange wire to Mega board's 3.3V (or any other 3V to 3.3V source).
         For PROMINI, attach sat grey to RX0.  Attach sat black to ground. */
      //#define SPEKTRUM 1024
      //#define SPEKTRUM 2048
      //#define SPEK_SERIAL_PORT 1    // Forced to 0 on Pro Mini and single serial boards; Set to your choice of 0, 1, or 2 on any Mega based board (defaults to 1 on Mega).
      //**************************
      // Defines that allow a "Bind" of a Spektrum or Compatible Remote Receiver (aka Satellite) via Configuration GUI.
      //   Bind mode will be same as declared above, if your TX is capable.
      //   Ground, Power, and Signal must come from three adjacent pins. 
      //   By default, these are Ground=4, Power=5, Signal=6.  These pins are in a row on most MultiWii shield boards. Pins can be overriden below.  
      //   Normally use 3.3V regulator is needed on the power pin!!  If your satellite hangs during bind (blinks, but won't complete bind with a solid light), go direct 5V on all pins. 
      //**************************
      //   For Pro Mini, the connector for the Satellite that resides on the FTDI can be unplugged and moved to these three adjacent pins. 
      //#define SPEK_BIND             //Un-Comment for Spektrum Satellie Bind Support.  Code is ~420 bytes smaller without it. 
      //#define SPEK_BIND_GROUND 4
      //#define SPEK_BIND_POWER  5
      //#define SPEK_BIND_DATA   6

    /*******************************    SBUS RECIVER    ************************************/
      /* The following line apply only for Futaba S-Bus Receiver on MEGA boards at RX1 only (Serial 1).
         You have to invert the S-Bus-Serial Signal e.g. with a Hex-Inverter like IC SN74 LS 04 */
      //#define SBUS

    /******************* RC signal from the serial port via Multiwii Serial Protocol *********/
//add by bakui, use this define while using ros-serial-rc-control
#define RCSERIAL


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  5 - ALTERNATE SETUP                                            *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /******                Serial com speed    *********************************/
    /* This is the speed of the serial interfaces */
    #define SERIAL0_COM_SPEED 115200
    #define SERIAL1_COM_SPEED 115200
    #define SERIAL2_COM_SPEED 115200
    #define SERIAL3_COM_SPEED 115200

    /* interleaving delay in micro seconds between 2 readings WMP/NK in a WMP+NK config
       if the ACC calibration time is very long (20 or 30s), try to increase this delay up to 4000
       it is relevent only for a conf with NK */
    #define INTERLEAVING_DELAY 3000

    /* when there is an error on I2C bus, we neutralize the values during a short time. expressed in microseconds
       it is relevent only for a conf with at least a WMP */
    #define NEUTRALIZE_DELAY 100000


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

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  6 - OPTIONAL FEATURES                                          *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /************************        Angele throttle correction         ********************/
  /* Automatically increase throttle based on the angle of the copter
     Original idea by Kraut Rob, first implementation HAdrian							*/

  //#define THROTTLE_ANGLE_CORRECTION 40
  
 /*************************        Advanced Headfree Mode             ********************/
 /* In Advanced Headfree mode when the copter is farther than ADV_HEADFREE_RANGE meters then 
    the  bearing between home and copter position will become the control direction 
	IF copter come closer than ADV_HEADFREE_RANGE meters, then the control direction freezed to the 
	bearing between home and copter at the point where it crosses the ADV_HEADFREE_RANGE meter distance
	first implementation by HAdrian, mods by EOSBandi
 */

   //#define ADVANCED_HEADFREE									//Advanced headfree mode is enabled when this is uncommented
   //#define ADV_HEADFREE_RANGE 15								//Range where advanced headfree mode activated


  /************************        continuous gyro calibration        ********************/
  /* Gyrocalibration will be repeated if copter is moving during calibration. */
    //#define GYROCALIBRATIONFAILSAFE

  /************************        AP FlightMode        **********************************/
    /* Temporarily Disables GPS_HOLD_MODE to be able to make it possible to adjust the Hold-position when moving the sticks.*/
    #define AP_MODE 40  // Create a deadspan for GPS.
        
  /************************   Assisted AcroTrainer    ************************************/
    /* Train Acro with auto recovery. Value set the point where ANGLE_MODE takes over.
       Remember to activate ANGLE_MODE first!...
       A Value on 200 will give a very distinct transfer */
    //#define ACROTRAINER_MODE 200   // http://www.multiwii.com/forum/viewtopic.php?f=16&t=1944#p17437


  /********                          Failsafe settings                 ********************/
    /* Failsafe check pulses on four main control channels CH1-CH4. If the pulse is missing or bellow 985us (on any of these four channels) 
       the failsafe procedure is initiated. After FAILSAFE_DELAY time from failsafe detection, the level mode is on (if ACC or nunchuk is avaliable),
       PITCH, ROLL and YAW is centered and THROTTLE is set to FAILSAFE_THROTTLE value. You must set this value to descending about 1m/s or so
       for best results. This value is depended from your configuration, AUW and some other params.  Next, afrer FAILSAFE_OFF_DELAY the copter is disarmed, 
       and motors is stopped. If RC pulse coming back before reached FAILSAFE_OFF_DELAY time, after the small quard time the RC control is returned to normal. */
    //#define FAILSAFE                                // uncomment  to activate the failsafe function
    #define FAILSAFE_DELAY     10                     // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
    #define FAILSAFE_OFF_DELAY 200                    // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example
    #define FAILSAFE_THROTTLE  (MINTHROTTLE + 200)    // Throttle level used for landing - may be relative to MINTHROTTLE - as in this case


  /*****************                DFRobot LED RING    *********************************/
    /* I2C DFRobot LED RING communication */
    //#define LED_RING

  /********************************    LED FLASHER    ***********************************/
    //#define LED_FLASHER
    //#define LED_FLASHER_DDR DDRB
    //#define LED_FLASHER_PORT PORTB
    //#define LED_FLASHER_BIT PORTB4
    //#define LED_FLASHER_INVERT
    //#define LED_FLASHER_SEQUENCE        0b00000000      // leds OFF
    //#define LED_FLASHER_SEQUENCE_ARMED  0b00000101      // create double flashes
    //#define LED_FLASHER_SEQUENCE_MAX    0b11111111      // full illumination
    //#define LED_FLASHER_SEQUENCE_LOW    0b00000000      // no illumination


  /*******************************    Landing lights    *********************************/
  /* Landing lights
     Use an output pin to control landing lights.
     They can be switched automatically when used in conjunction
     with altitude data from a sonar unit. */
    //#define LANDING_LIGHTS_DDR DDRC
    //#define LANDING_LIGHTS_PORT PORTC
    //#define LANDING_LIGHTS_BIT PORTC0
    //#define LANDING_LIGHTS_INVERT

    /* altitude above ground (in cm) as reported by sonar */
    //#define LANDING_LIGHTS_AUTO_ALTITUDE 50

    /* adopt the flasher pattern for landing light LEDs */
    //#define LANDING_LIGHTS_ADOPT_LED_FLASHER_PATTERN

  /*************************    INFLIGHT ACC Calibration    *****************************/
    /* This will activate the ACC-Inflight calibration if unchecked */
    //#define INFLIGHT_ACC_CALIBRATION

  /**************************    Disable WMP power pin     *******************************/
    /* disable use of the POWER PIN
       (allready done if the option RCAUXPIN12 is selected) */
    //#define DISABLE_POWER_PIN

  /*******************************    OSD Switch    *************************************/
    // This adds a box that can be interpreted by OSD in activation status (to switch on/off the overlay for instance)
  //#define OSD_SWITCH

  /**************************************************************************************/
  /***********************                  TX-related         **************************/
  /**************************************************************************************/

    /* introduce a deadband around the stick center
       Must be greater than zero, comment if you dont want a deadband on roll, pitch and yaw */
#define DEADBAND 6

    /* defines the neutral zone of throttle stick during altitude hold, default setting is
       +/-40 uncommend and change the value below if you want to change it. */
#define ALT_HOLD_THROTTLE_NEUTRAL_ZONE 60


  /**************************************************************************************/
  /***********************                  GPS                **************************/
  /**************************************************************************************/

    /* GPS using a SERIAL port
       if enabled, define here the Arduino Serial port number and the UART speed
       note: only the RX PIN is used in case of NMEA mode, the GPS is not configured by multiwii
       in NMEA mode the GPS must be configured to output GGA and RMC NMEA sentences (which is generally the default conf for most GPS devices)
       at least 5Hz update rate. uncomment the first line to select the GPS serial port of the arduino */
    //#define GPS_SERIAL 2 // should be 2 for flyduino v2. It's the serial port number on arduino MEGA
    //#define GPS_BAUD   57600
    #define GPS_BAUD   115200


   /* GPS protocol 
       NMEA  - Standard NMEA protocol GGA, GSA and RMC  sentences are needed
       UBLOX - U-Blox binary protocol, use the ublox config file (u-blox-config.ublox.txt) from the source tree 
       MTK_BINARY16 and MTK_BINARY19 - MTK3329 chipset based GPS with DIYDrones binary firmware (v1.6 or v1.9)
       With UBLOX and MTK_BINARY you don't have to use GPS_FILTERING in multiwii code !!! */

    
    //#define NMEA
    //#define UBLOX
    //#define MTK_BINARY16
    //#define MTK_BINARY19
    //#define INIT_MTK_GPS        // initialize MTK GPS for using selected speed, 5Hz update rate and GGA & RMC sentence or binary settings

    //#define GPS_PROMINI_SERIAL    57600 // Will Autosense if GPS is connected when ardu boots
   
    /* I2C GPS device made with an independant arduino + GPS device
       including some navigation functions
       contribution from EOSBandi   http://code.google.com/p/i2c-gps-nav/ 
       You have to use at least I2CGpsNav code r33 */
    //#define I2C_GPS

    /* I2C GPS device made with an indeedent ATTiny[24]313 + GPS device and
       optional sonar device.    https://github.com/wertarbyte/tiny-gps/ */
    /* get GPS data from Tiny-GPS */
    //#define TINY_GPS
    /* get sonar data from Tiny-GPS */
    //#define TINY_GPS_SONAR

    /* GPS data readed from Misio-OSD - GPS module connected to OSD, and MultiWii read GPS data from OSD - tested and working OK ! */
    //#define GPS_FROM_OSD

    /* indicate a valid GPS fix with at least 5 satellites by flashing the LED  - Modified by MIS - Using stable LED (YELLOW on CRIUS AIO) led work as sat number indicator 
      - No GPS FIX -> LED blink at speed of incoming GPS frames
      - Fix and sat no. bellow 5 -> LED off
      - Fix and sat no. >= 5 -> LED blinks, one blink for 5 sat, two blinks for 6 sat, three for 7 ... */
    #define GPS_LED_INDICATOR

    //#define USE_MSP_WP                        //Enables the MSP_WP command, which is used by WinGUI to display and log Home and Poshold positions

    //#define DONT_RESET_HOME_AT_ARM             // HOME position is reset at every arm, uncomment it to prohibit it (you can set home position with GyroCalibration)

    /* GPS navigation can control the heading */
    
    #define NAV_CONTROLS_HEADING       true      // copter faces toward the navigation point, maghold must be enabled for it
    #define NAV_TAIL_FIRST             false     // true - copter comes in with tail first 
    #define NAV_SET_TAKEOFF_HEADING    true      // true - when copter arrives to home position it rotates it's head to takeoff direction
    
    
    /* Get your magnetic decliniation from here : http://magnetic-declination.com/
       Convert the degree+minutes into decimal degree by ==> degree+minutes*(1/60)
       Note the sign on declination it could be negative or positive (WEST or EAST) */
    //#define MAG_DECLINIATION  3.96f              //For Budapest Hungary.
    //#define MAG_DECLINIATION  0.0f
#define MAG_DECLINIATION  -7.1f
    #define GPS_LEAD_FILTER                      // Adds a forward predictive filterig to compensate gps lag. Code based on Jason Short's lead filter implementation
    
    //#define GPS_FILTERING                        // add a 5 element moving average filter to GPS coordinates, helps eliminate gps noise but adds latency comment out to disable
    #define GPS_WP_RADIUS              200       // if we are within this distance to a waypoint then we consider it reached (distance is in cm)
    #define NAV_SLEW_RATE              30        // Adds a rate control to nav output, will smoothen out nav angle spikes


  /**************************************************************************************/
  /***********************        LCD/OLED - display settings       *********************/
  /**************************************************************************************/

    /* http://www.multiwii.com/wiki/index.php?title=Extra_features#LCD_.2F_OLED */

    /*****************************   The type of LCD     **********************************/
      /* choice of LCD attached for configuration and telemetry, see notes below */
      //#define LCD_DUMMY       // No Physical LCD attached.  With this & LCD_CONF defined, TX sticks still work to set gains, by watching LED blink.  
      //#define LCD_SERIAL3W    // Alex' initial variant with 3 wires, using rx-pin for transmission @9600 baud fixed
      //#define LCD_TEXTSTAR    // SERIAL LCD: Cat's Whisker LCD_TEXTSTAR Module CW-LCD-02 (Which has 4 input keys for selecting menus)
      //#define LCD_VT100       // SERIAL LCD: vt100 compatible terminal emulation (blueterm, putty, etc.)
      //#define LCD_TTY         // SERIAL LCD: useful to tweak parameters over cable with arduino IDE 'serial monitor'
      //#define LCD_ETPP        // I2C LCD: Eagle Tree Power Panel LCD, which is i2c (not serial)
      //#define LCD_LCD03       // I2C LCD: LCD03, which is i2c
      //#define OLED_I2C_128x64 // I2C LCD: OLED http://www.multiwii.com/forum/viewtopic.php?f=7&t=1350

    /******************************   Display settings   ***********************************/
//      #define LCD_SERIAL_PORT 0    // must be 0 on Pro Mini and single serial boards; Set to your choice on any Mega based board
//#define LCD_SERIAL_PORT 0    // must be 0 on Pro Mini and single serial boards; Set to your choice on any Mega based board

      //#define SUPPRESS_OLED_I2C_128x64LOGO  // suppress display of OLED logo to save memory

    /* double font height for better readability. Reduces visible #lines by half.
     * The lower part of each page is accessible under the name of shifted keyboard letter :
     * 1 - ! , 2 - @ , 3 - # , 4 - $ , 5 - % , 6 - ^ , 7 - & , 8 - * , 9 - (
     * You must add both to your lcd.telemetry.* sequences
     */
      //#define DISPLAY_FONT_DSIZE //currently only aplicable for OLED_I2C_128x64

    /* style of display - AUTODETECTED via LCD_ setting - only activate to override defaults */
      //#define DISPLAY_2LINES
      //#define DISPLAY_MULTILINE
      //#define MULTILINE_PRE 2  // multiline configMenu # pref lines
      //#define MULTILINE_POST 6 // multiline configMenu # post lines
    /********************************    Navigation     ***********************************/
    /* keys to navigate the LCD menu */
      #define LCD_MENU_PREV 'p'
      #define LCD_MENU_NEXT 'n'
      #define LCD_VALUE_UP 'u'
      #define LCD_VALUE_DOWN 'd'

      #define LCD_MENU_SAVE_EXIT 's'
      #define LCD_MENU_ABORT 'x'

  /**************************************************************************************/
  /***********************      LCD configuration menu         **************************/
  /**************************************************************************************/

    /* uncomment this line if you plan to use a LCD or OLED for tweaking parameters
     * http://www.multiwii.com/wiki/index.php?title=Extra_features#Configuration_Menu */
      //#define LCD_CONF

    /* to include setting the aux switches for AUX1 -> AUX4 via LCD */
      //#define LCD_CONF_AUX

    /* optional exclude some functionality - uncomment to suppress some unwanted telemetry pages */
      //#define SUPPRESS_LCD_CONF_AUX34

  /**************************************************************************************/
  /***********************      LCD       telemetry            **************************/
  /**************************************************************************************/

    /* to monitor system values (battery level, loop time etc. with LCD 
     * http://www.multiwii.com/wiki/index.php?title=LCD_Telemetry */

    /********************************    Activation     ***********************************/
    //#define LCD_TELEMETRY

    /* to enable automatic hopping between a choice of telemetry pages uncomment this. */
    //#define LCD_TELEMETRY_AUTO "123452679" // pages 1 to 9 in ascending order
    //#define LCD_TELEMETRY_AUTO  "212232425262729" // strong emphasis on page 2

    /* manual stepping sequence; first page of the sequence gets loaded at startup to allow non-interactive display */
    //#define LCD_TELEMETRY_STEP "0123456789" // should contain a 0 to allow switching off.

    /* optional exclude some functionality - uncomment to suppress some unwanted telemetry pages */
    //#define SUPPRESS_TELEMETRY_PAGE_1
    //#define SUPPRESS_TELEMETRY_PAGE_2
    //#define SUPPRESS_TELEMETRY_PAGE_3
    //#define SUPPRESS_TELEMETRY_PAGE_4
    //#define SUPPRESS_TELEMETRY_PAGE_5
    //#define SUPPRESS_TELEMETRY_PAGE_6
    //#define SUPPRESS_TELEMETRY_PAGE_7
    //#define SUPPRESS_TELEMETRY_PAGE_8
    //#define SUPPRESS_TELEMETRY_PAGE_9

  /********************************************************************/
  /****                             RSSI                           ****/
  /********************************************************************/
    //#define RX_RSSI
    //#define RX_RSSI_PIN A3

  /********************************************************************/
  /****                             Buzzer                         ****/
  /********************************************************************/
    //#define BUZZER
    //#define RCOPTIONSBEEP         // uncomment this if you want the buzzer to beep at any rcOptions change on channel Aux1 to Aux4
    //#define ARMEDTIMEWARNING 330  // Trigger an alarm after a certain time of being armed [s] to save you lipo (if your TX does not have a countdown)
    //#define PILOTLAMP             // Uncomment if you are using a X-Arcraft Pilot Lamp

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

  /********************************************************************/
  /****           altitude hold                                    ****/
  /********************************************************************/

    /* uncomment to disable the altitude hold feature.
     * This is useful if all of the following apply
     * + you have a baro
     * + want altitude readout
     * + do not use altitude hold feature
     * + want to save memory space
     */
 //#define SUPPRESS_BARO_ALTHOLD

  /* Natural alt change for rapid pilots. It's temporary switch OFF the althold when throttle stick is out of deadband defined with ALT_HOLD_THROTTLE_NEUTRAL_ZONE
   * but if it's commented: Smooth alt change routine is activated, for slow auto and aerophoto modes (in general solution from alexmos). It's slowly increase/decrease 
   * altitude proportional to stick movement (+/-100 throttle gives about +/-50 cm in 1 second with cycle time about 3-4ms)
   */
  #define ALTHOLD_FAST_THROTTLE_CHANGE

  /********************************************************************/
  /****           altitude variometer                              ****/
  /********************************************************************/

    /* enable to get audio feedback upon rising/falling copter/plane.
     * Requires a working baro.
     * For now, Output gets sent to an enabled vt100 terminal program over the serial line.
     * choice of two methods (enable either one or both)
     * method 1 : use short term movement from baro ( bigger code size)
     * method 2 : use long term observation of altitude from baro (smaller code size)
     */
    //#define VARIOMETER 12            // possible values: 12 = methods 1 & 2 ; 1 = method 1 ; 2 = method 2
    //#define SUPPRESS_VARIOMETER_UP   // if no signaling for up movement is desired
    //#define SUPPRESS_VARIOMETER_DOWN // if no signaling for down movement is desired
    //#define VARIOMETER_SINGLE_TONE   // use only one tone (BEL); neccessary for non-patched vt100 terminals

  /********************************************************************/
  /****           baord naming                                     ****/
  /********************************************************************/

    /*
     * this name is displayed together with the MultiWii version number
     * upon powerup on the LCD.
     * If you are without a DISPLAYD then You may enable LCD_TTY and
     * use arduino IDE's serial monitor to view the info.
     *
     * You must preserve the format of this string!
     * It must be 16 characters total,
     * The last 4 characters will be overwritten with the version number.
     */
    #define BOARD_NAME "MultiWii   V-.--"
    //                  123456789.123456

  /*************      Support multiple configuration profiles in EEPROM     ************/
    //#define MULTIPLE_CONFIGURATION_PROFILES

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
    //#define MIDRC 1500
#define MIDRC 1520

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
 

  /********************************************************************/
  /****           ESCs calibration                                 ****/
  /********************************************************************/

    /* to calibrate all ESCs connected to MWii at the same time (useful to avoid unplugging/re-plugging each ESC)
       Warning: this creates a special version of MultiWii Code
       You cannot fly with this special version. It is only to be used for calibrating ESCs
       Read How To at http://code.google.com/p/multiwii/wiki/ESCsCalibration */
    #define ESC_CALIB_LOW  MINCOMMAND
    #define ESC_CALIB_HIGH 2000
    //#define ESC_CALIB_CANNOT_FLY  // uncomment to activate

  /****           internal frequencies                             ****/
    /* frequenies for rare cyclic actions in the main loop, depend on cycle time
       time base is main loop cycle time - a value of 6 means to trigger the action every 6th run through the main loop
       example: with cycle time of approx 3ms, do action every 6*3ms=18ms
       value must be [1; 65535] */
    #define LCD_TELEMETRY_FREQ 23       // to send telemetry data over serial 23 <=> 60ms <=> 16Hz (only sending interlaced, so 8Hz update rate)
    #define LCD_TELEMETRY_AUTO_FREQ 967 // to step to next telemetry page 967 <=> 3s
    #define PSENSOR_SMOOTH 16           // len of averaging vector for smoothing the PSENSOR readings; should be power of 2; set to 1 to disable
    #define VBAT_SMOOTH 16              // len of averaging vector for smoothing the VBAT readings; should be power of 2; set to 1 to disable
    #define RSSI_SMOOTH 16              // len of averaging vector for smoothing the RSSI readings; should be power of 2; set to 1 to disable


