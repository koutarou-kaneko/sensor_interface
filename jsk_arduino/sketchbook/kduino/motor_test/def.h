

/**************************************************************************************/
/***************             Proc specific definitions             ********************/
/**************************************************************************************/




/**************************  all the Mega types  ***********************************/
#define LEDPIN_PINMODE             pinMode (13, OUTPUT);pinMode (30, OUTPUT);
#define LEDPIN_TOGGLE              PINB  |= (1<<7); PINC  |= (1<<7);
#define LEDPIN_ON                  PORTB |= (1<<7); PORTC |= (1<<7);
#define LEDPIN_OFF                 PORTB &= ~(1<<7);PORTC &= ~(1<<7);


#undef INTERNAL_I2C_PULLUPS 
#define I2C_SPEED 400000L         //400kHz fast mode

#define SERVO_1_PINMODE            pinMode(44,OUTPUT);        // TILT_PITCH
#define SERVO_1_PIN_HIGH           PORTL |= 1<<5;
#define SERVO_1_PIN_LOW            PORTL &= ~(1<<5);
#define SERVO_2_PINMODE            pinMode(45,OUTPUT);        // TILT_ROLL 
#define SERVO_2_PIN_HIGH           PORTL |= 1<<4;
#define SERVO_2_PIN_LOW            PORTL &= ~(1<<4);
#define SERVO_3_PINMODE            pinMode(46,OUTPUT);        // CAM TRIG
#define SERVO_3_PIN_HIGH           PORTL |= 1<<3;
#define SERVO_3_PIN_LOW            PORTL &= ~(1<<3);
#define SERVO_4_PINMODE            pinMode(11,OUTPUT);        // SERVO4 , use hardware PWM
#define SERVO_5_PINMODE            pinMode(12,OUTPUT);        // SERVO5 , use hardware PWM




#ifndef NUMBER_MOTOR
        #error "NUMBER_MOTOR is not set, most likely you have not defined any type of multicopter"
#endif


#if defined(POWERMETER_SOFT) && !(defined(VBAT))
        #error "to use powermeter, you must also define and configure VBAT"
#endif

