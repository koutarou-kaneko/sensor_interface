/**************************************************************************************/
/***************             test configurations                   ********************/
/**************************************************************************************/


/**************************************************************************************/
/***************             Proc specific definitions             ********************/
/**************************************************************************************/
// Proc auto detection
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
  #define MEGA
#endif


/**************************  all the Mega types  ***********************************/
#if defined(MEGA)
  #define LEDPIN_PINMODE             pinMode (13, OUTPUT);pinMode (30, OUTPUT);
  #define LEDPIN_TOGGLE              PINB  |= (1<<7); PINC  |= (1<<7);
  #define LEDPIN_ON                  PORTB |= (1<<7); PORTC |= (1<<7);
  #define LEDPIN_OFF                 PORTB &= ~(1<<7);PORTC &= ~(1<<7);
  #define BUZZERPIN_PINMODE          pinMode (32, OUTPUT);
  #if defined PILOTLAMP
    #define    PL_PIN_ON    PORTC |= 1<<5;
    #define    PL_PIN_OFF   PORTC &= ~(1<<5);
  #else
    #define BUZZERPIN_ON               PORTC |= 1<<5;
    #define BUZZERPIN_OFF              PORTC &= ~(1<<5);
  #endif 
    
  #if !defined(DISABLE_POWER_PIN)
    #define POWERPIN_PINMODE           pinMode (37, OUTPUT);
    #define POWERPIN_ON                PORTC |= 1<<0;
    #define POWERPIN_OFF               PORTC &= ~(1<<0);
  #else
    #define POWERPIN_PINMODE           ;
    #define POWERPIN_ON                ;
    #define POWERPIN_OFF               ;
  #endif
  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;       // PIN 20&21 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
  #define PINMODE_LCD                pinMode(0, OUTPUT);
  #define LCDPIN_OFF                 PORTE &= ~1; //switch OFF digital PIN 0
  #define LCDPIN_ON                  PORTE |= 1;
  #define STABLEPIN_PINMODE          pinMode (31, OUTPUT);
  #define STABLEPIN_ON               PORTC |= 1<<6;
  #define STABLEPIN_OFF              PORTC &= ~(1<<6);
  #if defined(PPM_ON_THROTTLE)
    //configure THROTTLE PIN (A8 pin) as input witch pullup and enabled PCINT interrupt
    #define PPM_PIN_INTERRUPT        DDRK &= ~(1<<0); PORTK |= (1<<0); PCMSK2 |= (1<<0); PCICR |= (1<<2);
  #else
    #define PPM_PIN_INTERRUPT        attachInterrupt(4, rxInt, RISING);  //PIN 19, also used for Spektrum satellite option
  #endif
  #if !defined(SPEK_SERIAL_PORT)
    #define SPEK_SERIAL_PORT         1
  #endif
  //RX PIN assignment inside the port //for PORTK
  #define THROTTLEPIN                0  //PIN 62 =  PIN A8
  #define ROLLPIN                    1  //PIN 63 =  PIN A9
  #define PITCHPIN                   2  //PIN 64 =  PIN A10
  #define YAWPIN                     3  //PIN 65 =  PIN A11
  #define AUX1PIN                    4  //PIN 66 =  PIN A12
  #define AUX2PIN                    5  //PIN 67 =  PIN A13
  #define AUX3PIN                    6  //PIN 68 =  PIN A14
  #define AUX4PIN                    7  //PIN 69 =  PIN A15
  #define V_BATPIN                   A0    // Analog PIN 0
  #define PSENSORPIN                 A2    // Analog PIN 2
  #define PCINT_PIN_COUNT            8
  #define PCINT_RX_BITS              (1<<2),(1<<4),(1<<5),(1<<6),(1<<7),(1<<0),(1<<1),(1<<3)
  #define PCINT_RX_PORT              PORTK
  #define PCINT_RX_MASK              PCMSK2
  #define PCIR_PORT_BIT              (1<<2)
  #define RX_PC_INTERRUPT            PCINT2_vect
  #define RX_PCINT_PIN_PORT          PINK


///////////////////////////////////////////////////////////////////////  
  
#ifdef USE_XBEE
  #define ISR_UART                   ISR(USART3_UDRE_vect)
#else// USE_XBEE
  #define ISR_UART                   ISR(USART0_UDRE_vect)
#endif// USE_XBEE
#endif


/**************************************************************************************/
/***************      IMU Orientations and Sensor definitions      ********************/
/**************************************************************************************/

#if defined(JSK_IMU) 
// MPU9150 = MPU6050 + AK8975
  #define MPU6050 
  #define AK8975
  #define MS561101BA 

#define AXES_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  =  Y; imu.accADC[PITCH]  = -X; imu.accADC[YAW]  =  Z;} 
#define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  =  Y; imu.magADC[PITCH]  = X; imu.magADC[YAW]  = -Z;} 

#undef INTERNAL_I2C_PULLUPS 
#define I2C_SPEED 400000L         //400kHz fast mode
#define ACC 1
#define MAG 1
#define GYRO 1
#define BARO 1
#define ACC_1G 512

#define ACC_25deg    (uint16_t)(ACC_1G * 0.423)
#define ACC_VelScale (9.80665f / 10000.0f / ACC_1G)

#endif// JSK_IMU


/**************************************************************************************/
/***************               override default pin assignments ?  ********************/
/**************************************************************************************/
#ifdef OVERRIDE_V_BATPIN
  #define V_BATPIN OVERRIDE_V_BATPIN
#endif
#ifdef OVERRIDE_PSENSORPIN
  #define PSENSORPIN OVERRIDE_PSENSORPIN
#endif
#ifdef OVERRIDE_LEDPIN_PINMODE
  #define LEDPIN_PINMODE OVERRIDE_LEDPIN_PINMODE
  #define LEDPIN_TOGGLE  OVERRIDE_LEDPIN_TOGGLE
  #define LEDPIN_OFF     OVERRIDE_LEDPIN_OFF
  #define LEDPIN_ON      OVERRIDE_LEDPIN_ON
#endif
#ifdef OVERRIDE_BUZZERPIN_PINMODE
  #define BUZZERPIN_PINMODE OVERRIDE_BUZZERPIN_PINMODE
  #define BUZZERPIN_ON      OVERRIDE_BUZZERPIN_ON
  #define BUZZERPIN_OFF     OVERRIDE_BUZZERPIN_OFF
#endif

/**************************************************************************************/
/********* enforce your sensors orientation - possibly overriding board defaults  *****/
/**************************************************************************************/
#ifdef FORCE_GYRO_ORIENTATION
  #define GYRO_ORIENTATION FORCE_GYRO_ORIENTATION
#endif
#ifdef FORCE_ACC_ORIENTATION
  #define ACC_ORIENTATION FORCE_ACC_ORIENTATION
#endif
#ifdef FORCE_MAG_ORIENTATION
  #define MAG_ORIENTATION FORCE_MAG_ORIENTATION
#endif

/**************************************************************************************/
/***************               Error Checking Section              ********************/
/**************************************************************************************/

#if (defined(LCD_DUMMY) || defined(LCD_SERIAL3W) || defined(LCD_TEXTSTAR) || defined(LCD_VT100) || defined(LCD_TTY) || defined(LCD_ETPP) || defined(LCD_LCD03) || defined(OLED_I2C_128x64) )
  #define HAS_LCD
#endif

#if (defined(LCD_CONF) || defined(LCD_TELEMETRY)) && !(defined(HAS_LCD) )
  #error "LCD_CONF or LCD_TELEMETRY defined, and choice of LCD not defined.  Uncomment one of LCD_SERIAL3W, LCD_TEXTSTAR, LCD_VT100, LCD_TTY or LCD_ETPP, LCD_LCD03, OLED_I2C_128x64"
#endif

#if defined(POWERMETER_SOFT) && !(defined(VBAT))
        #error "to use powermeter, you must also define and configure VBAT"
#endif

#if defined(LCD_TELEMETRY_AUTO) && !(defined(LCD_TELEMETRY))
        #error "to use automatic telemetry, you MUST also define and configure LCD_TELEMETRY"
#endif

#if defined(LCD_TELEMETRY_STEP) && !(defined(LCD_TELEMETRY))
        #error "to use single step telemetry, you MUST also define and configure LCD_TELEMETRY"
#endif
