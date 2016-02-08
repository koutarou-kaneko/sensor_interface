/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/

uint8_t PWM_PIN[8] = {3,5,6,2,7,8,9,10};      

volatile uint16_t atomicServo[8] = {8000,8000,8000,8000,8000,8000,8000,8000};


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


/**************************************************************************************/
/***************   Writes the Servos values to the needed format   ********************/
/**************************************************************************************/
void writeServos() {
  for(uint8_t i = 0; i < NUMBER_SERVO; i++){
    atomicServo[i] = (servo[i]-900)<<4;
  }
}

void writeAllServos(int16_t mc) { // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_SERVO;i++) {
    servo[i]=mc;
  }
  writeServos();
}


void writeMotors() { // [1000;2000] => [8000;16000]
    #if (NUMBER_MOTOR > 0) 
      #ifndef EXT_MOTOR_RANGE 
        OCR3C = motor[0]<<3; //  pin 3
      #else
        OCR3C = ((motor[0]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 1)
      #ifndef EXT_MOTOR_RANGE 
        OCR3A = motor[1]<<3; //  pin 5
      #else
        OCR3A = ((motor[1]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 2)
      #ifndef EXT_MOTOR_RANGE 
        OCR4A = motor[2]<<3; //  pin 6
      #else
        OCR4A = ((motor[2]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      #ifndef EXT_MOTOR_RANGE 
        OCR3B = motor[3]<<3; //  pin 2
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

}

void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
    motor[i]=mc;
  }
  writeMotors();
}

void initOutput() {
  /****************            mark all PWM pins as Output             ******************/
  for(uint8_t i=0;i<NUMBER_MOTOR;i++) {
    pinMode(PWM_PIN[i],OUTPUT);
  }
    

    #if (NUMBER_MOTOR > 0)
      // init 16bit timer 3
      TCCR3A |= (1<<WGM31); // phase correct mode
      TCCR3A &= ~(1<<WGM30);
      TCCR3B |= (1<<WGM33);
      TCCR3B &= ~(1<<CS31); // no prescaler
      ICR3   |= 0x3E80; // TOP to 16000;      
      
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
      ICR4   |= 0x3E80; // TOP to 16383;    
      
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

  //writeAllMotors(MINCOMMAND);
  delay(300);
}



