/*************************************************************************************************/
/****           CONFIGURABLE PARAMETERS                                                       ****/
/*************************************************************************************************/

#define USE_ROS           // rosserial on Port0
#define USE_HARDWARESERIAL  // for normal arduino lib
#define SUPPRESS_ALL_SERIAL_MSP // saves approx 2700 bytes
#define ROS_BAUD   115200 // also for bluetooh device connection
#define SERIAL_COM_PORT 3

//for servo
#define SERVO
#define NUMBER_SERVO 4
#define NUMBER_MOTOR     4
#define MIDSERVO 1500


#define MINTHROTTLE 1220 // for Turnigy Plush ESCs 10A 1220 is default
#define MAXTHROTTLE 1900
#define MINCOMMAND  1000
#define RCSERIAL


  /******                Serial com speed    *********************************/
    /* This is the speed of the serial interfaces */
    #define SERIAL0_COM_SPEED 115200
    #define SERIAL1_COM_SPEED 115200
    #define SERIAL2_COM_SPEED 115200
    #define SERIAL3_COM_SPEED 115200


/* Default 50Hz Servo refresh rate*/
#define SERVO_RFR_50HZ
//#define MEGA_HW_PWM_SERVOS
#define SERVO_RFR_RATE  50    // In Hz, you can set it from 20 to 400Hz, used only in HW PWM mode
#define ESC_CALIB_LOW  MINCOMMAND
#define ESC_CALIB_HIGH 2000
//#define ESC_CALIB_CANNOT_FLY  // uncomment to activate



