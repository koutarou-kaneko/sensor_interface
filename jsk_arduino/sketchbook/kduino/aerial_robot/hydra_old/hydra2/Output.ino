/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/
// since we are uing the PWM generation in a direct way, the pin order is just to inizialie the right pins 
// its not possible to change a PWM output pin just by changing the order
#if defined(PROMINI)
  uint8_t PWM_PIN[8] = {9,10,11,3,6,5,A2,12};   //for a quad+: rear,right,left,front
#endif
#if defined(PROMICRO)
  #if !defined(HWPWM6)
    #if !defined(TEENSY20)
      uint8_t PWM_PIN[8] = {9,10,5,6,4,A2,SW_PWM_P3,SW_PWM_P4};   //for a quad+: rear,right,left,front
    #else
      uint8_t PWM_PIN[8] = {14,15,9,12,22,18,16,17};   //for a quad+: rear,right,left,front
    #endif
  #else
    #if !defined(TEENSY20)
      uint8_t PWM_PIN[8] = {9,10,5,6,11,13,SW_PWM_P3,SW_PWM_P4};   //for a quad+: rear,right,left,front
    #else
      uint8_t PWM_PIN[8] = {14,15,9,12,4,10,16,17};   //for a quad+: rear,right,left,front
    #endif
  #endif
#endif
#if defined(MEGA)
  uint8_t PWM_PIN[8] = {3,5,6,2,7,8,9,10};      //for a quad+: rear,right,left,front   //+ for y6: 7:under right  8:under left
#endif

/**************************************************************************************/
/***************         Software PWM & Servo variables            ********************/
/**************************************************************************************/
#if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6)) || (defined(MEGA) && defined(MEGA_HW_PWM_SERVOS))
  #if defined(SERVO)
    #if defined(AIRPLANE)|| defined(HELICOPTER)
      // To prevent motor to start at reset. atomicServo[7]=5 or 249 if reversed servo
      volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,5}; 
    #else
      volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,125};
    #endif
  #endif
  #if (NUMBER_MOTOR > 4)
    //for HEX Y6 and HEX6/HEX6X/HEX6H flat for promini
    volatile uint8_t atomicPWM_PIN5_lowState;
    volatile uint8_t atomicPWM_PIN5_highState;
    volatile uint8_t atomicPWM_PIN6_lowState;
    volatile uint8_t atomicPWM_PIN6_highState;
  #endif
  #if (NUMBER_MOTOR > 6)
    //for OCTO on promini
    volatile uint8_t atomicPWM_PINA2_lowState;
    volatile uint8_t atomicPWM_PINA2_highState;
    volatile uint8_t atomicPWM_PIN12_lowState;
    volatile uint8_t atomicPWM_PIN12_highState;
  #endif
#else
  #if defined(SERVO)
    #if defined(AIRPLANE)|| defined(HELICOPTER)
      // To prevent motor to start at reset. atomicServo[7]=5 or 249 if reversed servo
      volatile uint16_t atomicServo[8] = {8000,8000,8000,8000,8000,8000,8000,320}; 
    #else
      volatile uint16_t atomicServo[8] = {8000,8000,8000,8000,8000,8000,8000,8000};
    #endif
  #endif
  #if (NUMBER_MOTOR > 4)
    //for HEX Y6 and HEX6/HEX6X/HEX6H and for Promicro
    volatile uint16_t atomicPWM_PIN5_lowState;
    volatile uint16_t atomicPWM_PIN5_highState;
    volatile uint16_t atomicPWM_PIN6_lowState;
    volatile uint16_t atomicPWM_PIN6_highState;
  #endif
  #if (NUMBER_MOTOR > 6)
    //for OCTO on Promicro
    volatile uint16_t atomicPWM_PINA2_lowState;
    volatile uint16_t atomicPWM_PINA2_highState;
    volatile uint16_t atomicPWM_PIN12_lowState;
    volatile uint16_t atomicPWM_PIN12_highState;
  #endif
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
  #if defined(MEGA)// [1000:2000] => [8000:16000] for timer 3 & 4 for mega
    #if (NUMBER_MOTOR > 0) 
      #ifndef EXT_MOTOR_RANGE 
        OCR3C = motor[0]<<3; //  pin 3, rear right
      #else
        OCR3C = ((motor[0]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 1)
      #ifndef EXT_MOTOR_RANGE 
        OCR3A = motor[1]<<3; //  pin 5, front right
      #else
        OCR3A = ((motor[1]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 2)
      #ifndef EXT_MOTOR_RANGE 
        OCR4A = motor[2]<<3; //  pin 6, rear left
      #else
        OCR4A = ((motor[2]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      #ifndef EXT_MOTOR_RANGE 
        OCR3B = motor[3]<<3; //  pin 2, front left
      #else
        OCR3B = ((motor[3]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 4)
      #ifndef EXT_MOTOR_RANGE 
        OCR4B = motor[4]<<3; //  pin 7
        OCR4C = motor[5]<<3; //  pin 8
      #else
        OCR4B = ((motor[4]<<4) - 16000) + 128;
        OCR4C = ((motor[5]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 6)
      #ifndef EXT_MOTOR_RANGE 
        OCR2B = motor[6]>>3; //  pin 9
        OCR2A = motor[7]>>3; //  pin 10
      #else
        OCR2B = (motor[6]>>2) - 250;
        OCR2A = (motor[7]>>2) - 250;
      #endif
    #endif
  #endif
  
  /******** Specific PWM Timers & Registers for the atmega32u4 (Promicro)   ************/
  #if defined(PROMICRO)
    #if (NUMBER_MOTOR > 0) // Timer 1 A & B [1000:2000] => [8000:16000]
      #ifndef EXT_MOTOR_RANGE 
        OCR1A = motor[0]<<3; //  pin 9
      #else
        OCR1A = ((motor[0]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 1)
      #ifndef EXT_MOTOR_RANGE 
        OCR1B = motor[1]<<3; //  pin 10
      #else
        OCR1B = ((motor[1]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 2) // Timer 4 A & D [1000:2000] => [1000:2000]
      #if !defined(HWPWM6)
        // to write values > 255 to timer 4 A/B we need to split the bytes
        #ifndef EXT_MOTOR_RANGE 
          TC4H = (2047-motor[2])>>8; OCR4A = ((2047-motor[2])&0xFF); //  pin 5
        #else
          TC4H = 2047-(((motor[2]-1000)<<1)+16)>>8; OCR4A = (2047-(((motor[2]-1000)<<1)+16)&0xFF); //  pin 5
        #endif
      #else
        #ifndef EXT_MOTOR_RANGE 
          OCR3A = motor[2]<<3; //  pin 5
        #else
          OCR3A = ((motor[2]<<4) - 16000) + 128;
        #endif
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      #ifndef EXT_MOTOR_RANGE 
        TC4H = motor[3]>>8; OCR4D = (motor[3]&0xFF); //  pin 6
      #else
        TC4H = (((motor[3]-1000)<<1)+16)>>8; OCR4D = ((((motor[3]-1000)<<1)+16)&0xFF); //  pin 6
      #endif
    #endif    
    #if (NUMBER_MOTOR > 4)
      #if !defined(HWPWM6)
        #if (NUMBER_MOTOR == 6) && !defined(SERVO)
          atomicPWM_PIN5_highState = motor[4]<<3;
          atomicPWM_PIN5_lowState = 16383-atomicPWM_PIN5_highState;
          atomicPWM_PIN6_highState = motor[5]<<3;
          atomicPWM_PIN6_lowState = 16383-atomicPWM_PIN6_highState;      
        #else
          atomicPWM_PIN5_highState = ((motor[4]-1000)<<4)+320;
          atomicPWM_PIN5_lowState = 15743-atomicPWM_PIN5_highState;
          atomicPWM_PIN6_highState = ((motor[5]-1000)<<4)+320;
          atomicPWM_PIN6_lowState = 15743-atomicPWM_PIN6_highState;        
        #endif
      #else
        #ifndef EXT_MOTOR_RANGE 
          OCR1C = motor[4]<<3; //  pin 11
          TC4H = motor[5]>>8; OCR4A = (motor[5]&0xFF); //  pin 13  
        #else
          OCR1C = ((motor[4]<<4) - 16000) + 128;
          TC4H = (((motor[5]-1000)<<1)+16)>>8; OCR4A = ((((motor[5]-1000)<<1)+16)&0xFF); //  pin 13       
        #endif  
      #endif
    #endif
    #if (NUMBER_MOTOR > 6)
      #if !defined(HWPWM6)
        atomicPWM_PINA2_highState = ((motor[6]-1000)<<4)+320;
        atomicPWM_PINA2_lowState = 15743-atomicPWM_PINA2_highState;
        atomicPWM_PIN12_highState = ((motor[7]-1000)<<4)+320;
        atomicPWM_PIN12_lowState = 15743-atomicPWM_PIN12_highState;
      #else
        atomicPWM_PINA2_highState = ((motor[6]-1000)>>2)+5;
        atomicPWM_PINA2_lowState = 245-atomicPWM_PINA2_highState;
        atomicPWM_PIN12_highState = ((motor[7]-1000)>>2)+5;
        atomicPWM_PIN12_lowState = 245-atomicPWM_PIN12_highState;     
      #endif
    #endif
  #endif
  
  /********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
  #if defined(PROMINI)
    #if (NUMBER_MOTOR > 0)
      #ifndef EXT_MOTOR_RANGE 
        OCR1A = motor[0]>>3; //  pin 9
      #else
        OCR1A = ((motor[0]>>2) - 250);
      #endif
    #endif
    #if (NUMBER_MOTOR > 1)
      #ifndef EXT_MOTOR_RANGE 
        OCR1B = motor[1]>>3; //  pin 10
      #else
        OCR1B = ((motor[1]>>2) - 250);
      #endif
    #endif
    #if (NUMBER_MOTOR > 2)
      #ifndef EXT_MOTOR_RANGE
        OCR2A = motor[2]>>3; //  pin 11
      #else
        OCR2A = ((motor[2]>>2) - 250);
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      #ifndef EXT_MOTOR_RANGE
        OCR2B = motor[3]>>3; //  pin 3
      #else
        OCR2B = ((motor[3]>>2) - 250);
      #endif
    #endif
    #if (NUMBER_MOTOR > 4)
      #if (NUMBER_MOTOR == 6) && !defined(SERVO)
        #ifndef EXT_MOTOR_RANGE 
          atomicPWM_PIN6_highState = motor[4]>>3;
          atomicPWM_PIN5_highState = motor[5]>>3;
        #else
          atomicPWM_PIN6_highState = (motor[4]>>2) - 250;
          atomicPWM_PIN5_highState = (motor[5]>>2) - 250;
        #endif
        atomicPWM_PIN6_lowState  = 255-atomicPWM_PIN6_highState;
        atomicPWM_PIN5_lowState  = 255-atomicPWM_PIN5_highState; 
      #else //note: EXT_MOTOR_RANGE not possible here
        atomicPWM_PIN6_highState = ((motor[4]-1000)>>2)+5;
        atomicPWM_PIN6_lowState  = 245-atomicPWM_PIN6_highState;
        atomicPWM_PIN5_highState = ((motor[5]-1000)>>2)+5;
        atomicPWM_PIN5_lowState  = 245-atomicPWM_PIN5_highState;
      #endif
    #endif
    #if (NUMBER_MOTOR > 6) //note: EXT_MOTOR_RANGE not possible here
      atomicPWM_PINA2_highState = ((motor[6]-1000)>>2)+5;
      atomicPWM_PINA2_lowState  = 245-atomicPWM_PINA2_highState;
      atomicPWM_PIN12_highState = ((motor[7]-1000)>>2)+5;
      atomicPWM_PIN12_lowState  = 245-atomicPWM_PIN12_highState;
    #endif
  #endif
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
    motor[i]=mc;
  }
  writeMotors();
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void initOutput() {
  /****************            mark all PWM pins as Output             ******************/
  for(uint8_t i=0;i<NUMBER_MOTOR;i++) {
    pinMode(PWM_PIN[i],OUTPUT);
  }
    
  /****************  Specific PWM Timers & Registers for the MEGA's    ******************/
  #if defined(MEGA)
    #if (NUMBER_MOTOR > 0)
      // init 16bit timer 3
      TCCR3A |= (1<<WGM31); // phase correct mode
      TCCR3A &= ~(1<<WGM30);
      TCCR3B |= (1<<WGM33);
      TCCR3B &= ~(1<<CS31); // no prescaler
      ICR3   |= 0x3FFF; // TOP to 16383;      
      
      TCCR3A |= _BV(COM3C1); // connect pin 3 to timer 3 channel C
    #endif
    #if (NUMBER_MOTOR > 1)
      TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A
    #endif
    #if (NUMBER_MOTOR > 2)
      // init 16bit timer 4
      TCCR4A |= (1<<WGM41); // phase correct mode
      TCCR4A &= ~(1<<WGM40);
      TCCR4B |= (1<<WGM43);
      TCCR4B &= ~(1<<CS41); // no prescaler
      ICR4   |= 0x3FFF; // TOP to 16383;    
      
      TCCR4A |= _BV(COM4A1); // connect pin 6 to timer 4 channel A
    #endif
    #if (NUMBER_MOTOR > 3)
      TCCR3A |= _BV(COM3B1); // connect pin 2 to timer 3 channel B
    #endif
    #if (NUMBER_MOTOR > 4)
      TCCR4A |= _BV(COM4B1); // connect pin 7 to timer 4 channel B
      TCCR4A |= _BV(COM4C1); // connect pin 8 to timer 4 channel C
    #endif
    #if (NUMBER_MOTOR > 6)
      // timer 2 is a 8bit timer so we cant change its range 
      TCCR2A |= _BV(COM2B1); // connect pin 9 to timer 2 channel B
      TCCR2A |= _BV(COM2A1); // connect pin 10 to timer 2 channel A
    #endif
  #endif
  
  /******** Specific PWM Timers & Registers for the atmega32u4 (Promicro)   ************/
  #if defined(PROMICRO)
    #if (NUMBER_MOTOR > 0)
      TCCR1A |= (1<<WGM11); // phase correct mode & no prescaler
      TCCR1A &= ~(1<<WGM10);
      TCCR1B &= ~(1<<WGM12) &  ~(1<<CS11) & ~(1<<CS12);
      TCCR1B |= (1<<WGM13) | (1<<CS10); 
      ICR1   |= 0x3FFF; // TOP to 16383;     
      TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
    #endif
    #if (NUMBER_MOTOR > 1)
      TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
    #endif
    #if (NUMBER_MOTOR > 2)
      #if !defined(HWPWM6) // timer 4A
        TCCR4E |= (1<<ENHC4); // enhanced pwm mode
        TCCR4B &= ~(1<<CS41); TCCR4B |= (1<<CS42)|(1<<CS40); // prescaler to 16
        TCCR4D |= (1<<WGM40); TC4H = 0x3; OCR4C = 0xFF; // phase and frequency correct mode & top to 1023 but with enhanced pwm mode we have 2047
        TCCR4A |= (1<<COM4A0)|(1<<PWM4A); // connect pin 5 to timer 4 channel A 
      #else // timer 3A
        TCCR3A |= (1<<WGM31); // phase correct mode & no prescaler
        TCCR3A &= ~(1<<WGM30);
        TCCR3B &= ~(1<<WGM32) &  ~(1<<CS31) & ~(1<<CS32);
        TCCR3B |= (1<<WGM33) | (1<<CS30); 
        ICR3   |= 0x3FFF; // TOP to 16383;     
        TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A    
      #endif 
    #endif
    #if (NUMBER_MOTOR > 3)
      #if defined(HWPWM6) 
        TCCR4E |= (1<<ENHC4); // enhanced pwm mode
        TCCR4B &= ~(1<<CS41); TCCR4B |= (1<<CS42)|(1<<CS40); // prescaler to 16
        TCCR4D |= (1<<WGM40); TC4H = 0x3; OCR4C = 0xFF; // phase and frequency correct mode & top to 1023 but with enhanced pwm mode we have 2047
      #endif
      TCCR4C |= (1<<COM4D1)|(1<<PWM4D); // connect pin 6 to timer 4 channel D
    #endif
    #if (NUMBER_MOTOR > 4)
      #if defined(HWPWM6) 
        TCCR1A |= _BV(COM1C1); // connect pin 11 to timer 1 channel C
        TCCR4A |= (1<<COM4A1)|(1<<PWM4A); // connect pin 13 to timer 4 channel A 
      #else
        initializeSoftPWM();
      #endif
    #endif
    #if (NUMBER_MOTOR > 6)
      #if defined(HWPWM6) 
        initializeSoftPWM();
      #endif
    #endif
  #endif
  
  /********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
  #if defined(PROMINI)
    #if (NUMBER_MOTOR > 0)
      TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
    #endif
    #if (NUMBER_MOTOR > 1)
      TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
    #endif
    #if (NUMBER_MOTOR > 2)
      TCCR2A |= _BV(COM2A1); // connect pin 11 to timer 2 channel A
    #endif
    #if (NUMBER_MOTOR > 3)
      TCCR2A |= _BV(COM2B1); // connect pin 3 to timer 2 channel B
    #endif
    #if (NUMBER_MOTOR > 5)  // PIN 5 & 6 or A0 & A1
      initializeSoftPWM();
      #if defined(A0_A1_PIN_HEX) || (NUMBER_MOTOR > 6)
        pinMode(5,INPUT);pinMode(6,INPUT);     // we reactivate the INPUT affectation for these two PINs
        pinMode(A0,OUTPUT);pinMode(A1,OUTPUT);
      #endif
    #endif
  #endif

  /********  special version of MultiWii to calibrate all attached ESCs ************/
  #if defined(ESC_CALIB_CANNOT_FLY)
    writeAllMotors(ESC_CALIB_HIGH);
    delay(3000);
    writeAllMotors(ESC_CALIB_LOW);
    delay(500);
    while (1) {
      delay(5000);
      blinkLED(2,20, 2);
    #if defined(BUZZER)
      alarmArray[7] = 2;
    #endif
    }
    exit; // statement never reached
  #endif
  
  writeAllMotors(MINCOMMAND);
  delay(300);
  #if defined(SERVO)
    initializeServo();
  #endif
}


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

  //writeAllServos(MIDSERVO);
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

/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/
void mixTable() {
  int16_t maxMotor;
  uint8_t i;
  
  //yaw direction indicates the direction fo the force
  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

  #if NUMBER_MOTOR > 3
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));
  #endif
  /****************                   main Mix Table                ******************/
  #ifdef MY_PRIVATE_MIXING
    #include MY_PRIVATE_MIXING
  #else
    #ifdef QUADX
#if 0 //default code
      motor[0] = PIDMIX(-1,+1,-1); //REAR_R
      motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
      motor[2] = PIDMIX(+1,+1,+1); //REAR_L
      motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
#else //bakui edited
      motor[0] = (int32_t)rcCommand[THROTTLE] + (int32_t)(((int32_t)axisPID[ROLL]* qMatrix[0] + (int32_t)axisPID[PITCH]*qMatrix[4] + YAW_DIRECTION * (int32_t)axisPID[YAW]*qMatrix[8] + motorBias[0])>>10);
      motor[1] = (int32_t)rcCommand[THROTTLE] + (int32_t)(((int32_t)axisPID[ROLL]* qMatrix[1] + (int32_t)axisPID[PITCH]*qMatrix[5] + YAW_DIRECTION * (int32_t)axisPID[YAW]*qMatrix[9] + motorBias[1])>>10);
      motor[2] = (int32_t)rcCommand[THROTTLE] + (int32_t)(((int32_t)axisPID[ROLL]* qMatrix[2] + (int32_t)axisPID[PITCH]*qMatrix[6] + YAW_DIRECTION * (int32_t)axisPID[YAW]*qMatrix[10] + motorBias[2])>>10);
      motor[3] = (int32_t)rcCommand[THROTTLE] + (int32_t)(((int32_t)axisPID[ROLL]* qMatrix[3] + (int32_t)axisPID[PITCH]*qMatrix[7] + YAW_DIRECTION * (int32_t)axisPID[YAW]*qMatrix[11] + motorBias[3])>>10);

#endif
    #endif
    #ifdef HEX6
      motor[0] = PIDMIX(-7/8,+1/2,+1); //REAR_R
      motor[1] = PIDMIX(-7/8,-1/2,-1); //FRONT_R
      motor[2] = PIDMIX(+7/8,+1/2,+1); //REAR_L
      motor[3] = PIDMIX(+7/8,-1/2,-1); //FRONT_L
      motor[4] = PIDMIX(+0  ,-1  ,+1); //FRONT
      motor[5] = PIDMIX(+0  ,+1  ,-1); //REAR
    #endif
    #ifdef HEX6X
      motor[0] = PIDMIX(-1/2,+7/8,+1); //REAR_R
      motor[1] = PIDMIX(-1/2,-7/8,+1); //FRONT_R
      motor[2] = PIDMIX(+1/2,+7/8,-1); //REAR_L
      motor[3] = PIDMIX(+1/2,-7/8,-1); //FRONT_L
      motor[4] = PIDMIX(-1  ,+0  ,-1); //RIGHT
      motor[5] = PIDMIX(+1  ,+0  ,+1); //LEFT
    #endif
    #ifdef HEX6H
      motor[0] = PIDMIX(-1,+1,-1); //REAR_R
      motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
      motor[2] = PIDMIX(+ 1,+1,+1); //REAR_L
      motor[3] = PIDMIX(+ 1,-1,-1); //FRONT_L
      motor[4] = PIDMIX(0 ,0 ,0); //RIGHT
      motor[5] = PIDMIX(0 ,0 ,0); //LEFT
    #endif
    #ifdef OCTOX8
      motor[0] = PIDMIX(-1,+1,-1); //REAR_R
      motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
      motor[2] = PIDMIX(+1,+1,+1); //REAR_L
      motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
      motor[4] = PIDMIX(-1,+1,+1); //UNDER_REAR_R
      motor[5] = PIDMIX(-1,-1,-1); //UNDER_FRONT_R
      motor[6] = PIDMIX(+1,+1,-1); //UNDER_REAR_L
      motor[7] = PIDMIX(+1,-1,+1); //UNDER_FRONT_L
    #endif
    #ifdef OCTOFLATP
      motor[0] = PIDMIX(+7/10,-7/10,+1); //FRONT_L
      motor[1] = PIDMIX(-7/10,-7/10,+1); //FRONT_R
      motor[2] = PIDMIX(-7/10,+7/10,+1); //REAR_R
      motor[3] = PIDMIX(+7/10,+7/10,+1); //REAR_L
      motor[4] = PIDMIX(+0   ,-1   ,-1); //FRONT
      motor[5] = PIDMIX(-1   ,+0   ,-1); //RIGHT
      motor[6] = PIDMIX(+0   ,+1   ,-1); //REAR
      motor[7] = PIDMIX(+1   ,+0   ,-1); //LEFT
    #endif
    #ifdef OCTOFLATX
      motor[0] = PIDMIX(+1  ,-1/2,+1); //MIDFRONT_L
      motor[1] = PIDMIX(-1/2,-1  ,+1); //FRONT_R
      motor[2] = PIDMIX(-1  ,+1/2,+1); //MIDREAR_R
      motor[3] = PIDMIX(+1/2,+1  ,+1); //REAR_L
      motor[4] = PIDMIX(+1/2,-1  ,-1); //FRONT_L
      motor[5] = PIDMIX(-1  ,-1/2,-1); //MIDFRONT_R
      motor[6] = PIDMIX(-1/2,+1  ,-1); //REAR_R
      motor[7] = PIDMIX(+1  ,+1/2,-1); //MIDREAR_L
    #endif

    /************************************************************************************************************/
  #endif //MY_PRIVATE_MIXING
  /****************                    Cam trigger Sevo                ******************/
  /****************     neutralize Servos during calibration of gyro&acc   ******************/


  /****************                Filter the Motors values                ******************/
  #ifdef GOVERNOR_P
    if (rcOptions[BOXGOV] ) {
      static int8_t g[37] = { 0,3,5,8,11,14,17,19,22,25,28,31,34,38,41,44,47,51,54,58,61,65,68,72,76,79,83,87,91,95,99,104,108,112,117,121,126 };
      uint8_t v = constrain( VBATNOMINAL - constrain(analog.vbat, conf.vbatlevel_crit, VBATNOMINAL), 0, 36);
      for (i = 0; i < NUMBER_MOTOR; i++) {
        motor[i] += ( ( (int32_t)(motor[i]-1000) * (int32_t)g[v] ) * (int32_t)conf.governorR )/ 5000;
      }
    }
  #endif
  /****************                normalize the Motors values                ******************/

    maxMotor=motor[0];
    for(i=1; i< NUMBER_MOTOR; i++)
      if (motor[i]>maxMotor) maxMotor=motor[i];
    for(i=0; i< NUMBER_MOTOR; i++) {
      if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
        motor[i] -= maxMotor - MAXTHROTTLE;
      motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);
#if defined(ALTHOLD_FAST_THROTTLE_CHANGE)
      if (rcData[THROTTLE] < MINCHECK)
#else
        if ((rcData[THROTTLE] < MINCHECK) && !f.BARO_MODE)
#endif
#ifndef MOTOR_STOP
          motor[i] = conf.minthrottle;
#else
      motor[i] = MINCOMMAND;
#endif
      if (!f.ARMED)
        motor[i] = MINCOMMAND;
    }

}

