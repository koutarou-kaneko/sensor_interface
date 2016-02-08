/*************************************************************************************************/
/****           CONFIGURABLE PARAMETERS                                                       ****/
/*************************************************************************************************/

#define USE_ROS           // rosserial on Port0
#define USE_HARDWARESERIAL  // for normal arduino lib
#define SUPPRESS_ALL_SERIAL_MSP // saves approx 2700 bytes
#define ROS_BAUD   115200 // also for bluetooh device connection

#define SERIAL_COM_PORT 3

//motor
#define NUMBER_MOTOR 4

//for servo
#define SERVO
#define NUMBER_SERVO 0
#define NUMBER_SERVO 4
#define MIDSERVO 1500


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  1 - BASIC SETUP                                                *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /****************************    Motor minthrottle    *******************************/
    /* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
       This is the minimum value that allow motors to run at a idle speed  */
#define MINTHROTTLE 1220 // for Turnigy Plush ESCs 10A 1220 is default
    //#define MINTHROTTLE 1180 // this is for JSK_ROOK_PROTOTYPE
    //#define MINTHROTTLE 1120 // for Super Simple ESCs 10A
    //#define MINTHROTTLE 1064 // special ESC (simonk)
    //#define MINTHROTTLE 1050 // for brushed ESCs like ladybird
    //#define MINTHROTTLE 1150 // 

  /****************************    Motor maxthrottle    *******************************/
    /* this is the maximum value for the ESCs at full power, this value can be increased up to 2000 */
    #define MAXTHROTTLE 1800

  /****************************    Mincommand          *******************************/
    /* this is the value for the ESCs when they are not armed
       in some cases, this value must be lowered down to 900 for some specific ESCs, otherwise they failed to initiate */
    #define MINCOMMAND  1000

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
#define YAW_DIRECTION -1 // by bakui, -1: z axis is downward, which is general
    /* you can change the tricopter servo travel here */


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



  /************************        AP FlightMode        **********************************/
    /* Temporarily Disables GPS_HOLD_MODE to be able to make it possible to adjust the Hold-position when moving the sticks.*/
#define AP_MODE 40  // Create a deadspan for GPS.
#define ALT_HOLD_THROTTLE_NEUTRAL_ZONE 60

    
    /* Get your magnetic decliniation from here : http://magnetic-declination.com/
       Convert the degree+minutes into decimal degree by ==> degree+minutes*(1/60)
       Note the sign on declination it could be negative or positive (WEST or EAST) */
    //#define MAG_DECLINIATION  3.96f              //For Budapest Hungary.
    //#define MAG_DECLINIATION  0.0f
#define MAG_DECLINIATION  -7.1f



#define MIDRC 1520

  /***********************         Servo Refreshrates            ***********************/
    /* Default 50Hz Servo refresh rate*/
    #define SERVO_RFR_50HZ

    #define SERVO_RFR_RATE  50    // In Hz, you can set it from 20 to 400Hz, used only in HW PWM mode
 
    #define ESC_CALIB_LOW  MINCOMMAND
    #define ESC_CALIB_HIGH 2000

