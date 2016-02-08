/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/
// since we are uing the PWM generation in a direct way, the pin order is just to inizialie the right pins 
// its not possible to change a PWM output pin just by changing the order
uint8_t PWM_PIN[13] = {
//  3,5,6,2,7,8,9,10,11,12,44,45,46
  2,3,5,6,7,8,9,10,11,12,44,45,46
};
  
/**************************************************************************************/
/***************         Software PWM & Servo variables            ********************/
/**************************************************************************************/
#if defined(SERVO)
 volatile uint16_t atomicServo[8] = {8000,8000,8000,8000,8000,8000,8000,8000};
#endif

/**************************************************************************************/
/***************   Writes the Servos values to the needed format   ********************/
/**************************************************************************************/
void writeServos() {
  for(uint8_t i = 0; i < NUMBER_SERVO; i++){
//    atomicServo[i] = (servo[i]-1000)<<4;
    atomicServo[i] = (servo[i]-900)<<4;
  }
}

void writeAllServos(int16_t mc) { // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_SERVO;i++) {
    servo[i]=mc;
  }
  writeServos();
}

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors() { // [1000;2000] => [125;250]
  /****************  Specific PWM Timers & Registers for the MEGA's   *******************/
#ifdef MOTOR_50HZ
#define SET_MOTOR(OCR_REG,T_US) OCR_REG=(T_US);
#else// MOTOR_50HZ
#define SET_MOTOR(OCR_REG,T_US) OCR_REG=(T_US)<<3;
#endif// MOTOR_50HZ
  SET_MOTOR(OCR3B, motor[0]); //  pin 2
  SET_MOTOR(OCR3C, motor[1]); //  pin 3
  SET_MOTOR(OCR3A, motor[2]); //  pin 5
  SET_MOTOR(OCR4A, motor[3]); //  pin 6
  SET_MOTOR(OCR4B, motor[4]); //  pin 7
  SET_MOTOR(OCR4C, motor[5]); //  pin 8
//  OCR2B = motor[6]>>6; //  pin 9
//  OCR2A = motor[7]>>6; //  pin 10
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc) { // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
    motor[i]=mc;
  }
  writeMotors();
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void initOutput() {
  initializeMotor();
  delay(300);
#if defined(SERVO)
  initializeServo();
#endif
}

/**************************************************************************************/
/************                Initialize the PWM Motors               ******************/
/**************************************************************************************/
void initializeMotor() {
  /****************            mark all PWM pins as Output             ******************/
  for(uint8_t i=0;i<NUMBER_MOTOR;i++) {
    pinMode(PWM_PIN[i],OUTPUT);
  }

  /****************  Specific PWM Timers & Registers for the MEGA's    ******************/
#ifdef MOTOR_50HZ
#define ICMAX 0x4E20  // 20000/8/2 usec
#else// MOTOR_50HZ
#define ICMAX 0x3FFF  // 16383/8/2 usec
#endif// MOTOR_50HZ

  // init 16bit timer 3
  TCCR3A |= (1<<WGM31); // phase correct mode
  TCCR3A &= ~(1<<WGM30);
  TCCR3B |= (1<<WGM33);
#ifdef MOTOR_50HZ
  TCCR3B &= ~((1<<CS30)|(1<<CS31)|(1<<CS32)); // reset prescaler
  TCCR3B |= (1<<CS31); // 1/8 prescaler
#else// MOTOR_50HZ
  TCCR3B &= ~(1<<CS31); // no prescaler
#endif// MOTOR_50HZ
  ICR3   |= ICMAX; // TOP to 16383;      

  TCCR3A |= _BV(COM3C1); // connect pin 3 to timer 3 channel C
  TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A
  
  // init 16bit timer 4
  TCCR4A |= (1<<WGM41); // phase correct mode
  TCCR4A &= ~(1<<WGM40);
  TCCR4B |= (1<<WGM43);
#ifdef MOTOR_50HZ
  TCCR4B &= ~((1<<CS40)|(1<<CS41)|(1<<CS42)); // reset prescaler
  TCCR4B |= (1<<CS41); // 1/8 prescaler
#else// MOTOR_50HZ
  TCCR4B &= ~(1<<CS41); // no prescaler
#endif// MOTOR_50HZ
  ICR4   |= ICMAX; // TOP to 16383;    

  TCCR4A |= _BV(COM4A1); // connect pin 6 to timer 4 channel A
  TCCR3A |= _BV(COM3B1); // connect pin 2 to timer 3 channel B

#ifdef OMUSUBI_HEX
  TCCR4A |= _BV(COM4B1); // connect pin 7 to timer 4 channel B
  TCCR4A |= _BV(COM4C1); // connect pin 8 to timer 4 channel C
#endif// OMUSUBI_HEX

  // timer 2 is a 8bit timer so we cant change its range 
//  TCCR2A |= _BV(COM2B1); // connect pin 9 to timer 2 channel B
//  TCCR2A |= _BV(COM2A1); // connect pin 10 to timer 2 channel A

  writeAllMotors(MIDMOTOR);
}

/**************************************************************************************/
/************                Initialize the PWM Servos               ******************/
/**************************************************************************************/
void initializeServo() {
  SERVO_1_PINMODE;
  SERVO_2_PINMODE;
  SERVO_3_PINMODE;
  SERVO_4_PINMODE;
  SERVO_5_PINMODE;
  SERVO_6_PINMODE;
  SERVO_7_PINMODE;
  SERVO_8_PINMODE;

#if defined(SERVO_1_HIGH)  
  // uses timer 5 Comperator A (11 bit)
  TCCR5A &= ~(1<<WGM50) & ~(1<<WGM51); //normal counting & no prescaler
  TCCR5B &= ~(1<<WGM52) & ~(1<<CS51) & ~(1<<CS52) & ~(1<<WGM53);
  TCCR5B |= (1<<CS50);   
  TIMSK5 |= (1<<OCIE5A); // Enable CTC interrupt  
  #define SERVO_ISR TIMER5_COMPA_vect
  #define SERVO_CHANNEL OCR5A
  #define SERVO_900_US 14400 
//  #define SERVO_1K_US 16000 
  #define SERVO_1K_US SERVO_900_US 
#endif//SERVO_1_HIGH

  //writeAllServos(MIDSERVO); //bakui
  servo[0] = 1525;
  servo[1] = 1525;
  servo[2] = 1400;
  servo[3] = 1000;
  servo[4] = 1500;
  servo[5] = 1500;
  servo[6] = 1500;
  servo[7] = 1500;
   writeServos();
}

/**************************************************************************************/
/************              Servo software PWM generation             ******************/
/**************************************************************************************/
// prescaler is set by default to 64 on Timer0
// Duemilanove : 16MHz / 64 => 4 us
// 256 steps = 1 counter cycle = 1024 usb

//ISR x_  x_  x_ ... 
// 1  1111
// 2      1111
// 3          1111
// ...
// 1000usec + atomicServo[i]
#define SERVO_PULSE(PIN_HIGH,ACT_STATE,SERVO_NUM,LAST_PIN_LOW) \
    }else if(state == ACT_STATE){                                \
      LAST_PIN_LOW;                                              \
      PIN_HIGH;                                                  \
      SERVO_CHANNEL+=SERVO_1K_US;                                \
      state++;                                                   \
    }else if(state == ACT_STATE+1){                              \
      SERVO_CHANNEL+=atomicServo[SERVO_NUM];                     \
      state++;                                                   
  
  ISR(SERVO_ISR) {
    static uint8_t state = 0; // indicates the current state of the chain
    if (state==0) {
      SERVO_1_HIGH;
      SERVO_CHANNEL+=SERVO_1K_US;  // wait 1000us
      state++; // count up the state
    }else if(state==1){
      SERVO_CHANNEL+=atomicServo[SERVO_1_ARR_POS]; // load the servo's value (0-1000us)
      state++; // count up the state
    #if defined(SERVO_2_HIGH)
      SERVO_PULSE(SERVO_2_HIGH,2,SERVO_2_ARR_POS,SERVO_1_LOW); // the same here
    #endif
    #if defined(SERVO_3_HIGH)
      SERVO_PULSE(SERVO_3_HIGH,4,SERVO_3_ARR_POS,SERVO_2_LOW);
    #endif
    #if defined(SERVO_4_HIGH)
      SERVO_PULSE(SERVO_4_HIGH,6,SERVO_4_ARR_POS,SERVO_3_LOW);
    #endif
    #if defined(SERVO_5_HIGH)
      SERVO_PULSE(SERVO_5_HIGH,8,SERVO_5_ARR_POS,SERVO_4_LOW);
    #endif
    #if defined(SERVO_6_HIGH)
      SERVO_PULSE(SERVO_6_HIGH,10,SERVO_6_ARR_POS,SERVO_5_LOW);
    #endif
    #if defined(SERVO_7_HIGH)
      SERVO_PULSE(SERVO_7_HIGH,12,SERVO_7_ARR_POS,SERVO_6_LOW);
    #endif
    #if defined(SERVO_8_HIGH)
      SERVO_PULSE(SERVO_8_HIGH,14,SERVO_8_ARR_POS,SERVO_7_LOW);
    #endif
    }else{
      LAST_LOW;
      #if defined(SERVO_RFR_300HZ)
        #if defined(SERVO_3_HIGH)  // if there are 3 or more servos we dont need to slow it down
          SERVO_CHANNEL+=(SERVO_1K_US>>3); // 0 would be better but it causes bad jitter
          state=0; 
        #else // if there are less then 3 servos we need to slow it to not go over 300Hz (the highest working refresh rate for the digital servos for what i know..)
          SERVO_CHANNEL+=SERVO_1K_US;
          if(state<4){
            state+=2;
          }else{
            state=0;
          }
        #endif
      #endif
      #if defined(SERVO_RFR_160HZ)
        #if defined(SERVO_4_HIGH)  // if there are 4 or more servos we dont need to slow it down
          SERVO_CHANNEL+=(SERVO_1K_US>>3); // 0 would be better but it causes bad jitter
          state=0; 
        #else // if there are less then 4 servos we need to slow it to not go over ~170Hz (the highest working refresh rate for analog servos)
          SERVO_CHANNEL+=SERVO_1K_US;
          if(state<8){
            state+=2;
          }else{
            state=0;
          }
        #endif
      #endif   
      #if defined(SERVO_RFR_50HZ) // to have ~ 50Hz for all servos
        SERVO_CHANNEL+=SERVO_1K_US;
        if(state<30){
          state+=2;
        }else{
          state=0;
        }     
      #endif
    }
  } 

