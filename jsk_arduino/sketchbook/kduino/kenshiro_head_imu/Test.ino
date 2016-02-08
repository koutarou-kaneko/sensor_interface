/*
  Test
*/

#ifdef USE_TEST

unsigned long testLoopCount;

void test_setup()
{
  testLoopCount = 0;
}

void test_loop()
{
  static uint32_t testTime = 0;
  if (currentTime > testTime ) {
//  testTime = currentTime + 100000;  // 10Hz
    testTime = currentTime + 50000;  // 20Hz
//  testTime = currentTime + 20000;  // 50Hz
//  testTime = currentTime + 10000;  // 100Hz

    // Cyclic Process
#ifdef MOTOR_TEST  // motor test
    for (i=0;i<NUMBER_MOTOR;i++) {
      int j=(testLoopCount/50)%8;
      if (i==j) {
        motor[i] = 1000+10*(testLoopCount%50);
      }
      else {
        motor[i] = MIDRC;
      }
    }
#endif// MOTOR_TEST  // motor test

#ifdef SERVO_TEST
    for (i=0;i<NUMBER_SERVO;i++) {
      int j=(testLoopCount/50)%8;
      if (i==j) {
        servo[i] = 1500+10*(testLoopCount%50);
      }
      else {
        servo[i] = MIDRC;
      }
    }
#endif// SERVO_TEST
    
    // CountUp
    testLoopCount++;
  }else {
    // Non-Cyclic Process

  }
}

#endif// USE_TEST

