/*************************************************************************************************/
/****           CONFIGURABLE PARAMETERS                                                       ****/
/*************************************************************************************************/

#define JSK_IMU		// JSK IMU
#define USE_ROS           // rosserial on Port0
#define USE_XBEE          // XBEE for MultiWiiConf on Port3
#define USE_HARDWARESERIAL  // for normal arduino lib
#define SUPPRESS_ALL_SERIAL_MSP // saves approx 2700 bytes
//#define ROS_BAUD   57600
#define ROS_BAUD   115200  // bluetooth


#ifdef USE_XBEE
#define SERIAL_COM_PORT 3
#else// USE_XBEE
#define SERIAL_COM_PORT 0
#endif// USE_XBEE

  /**********************************    I2C speed   ************************************/
#define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP
    //#define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones

    /******************* RC signal from the serial port via Multiwii Serial Protocol *********/
//add by bakui, use this define while using ros-serial-rc-control
#define RCSERIAL

#define MPU6050_I2C_AUX_MASTER


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

    #define BOARD_NAME "MultiWii   V-.--"

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


