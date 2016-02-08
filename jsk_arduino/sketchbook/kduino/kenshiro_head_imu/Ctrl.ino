/*
  Controler
*/

#ifdef USE_CTRL
unsigned long ctrlLoopCount;

void ctrl_setup()
{
  ctrlLoopCount = 0;
}

void ctrl_loop()
{
  static uint32_t ctrlTime = 0;
  if (currentTime > ctrlTime ) {
//  ctrlTime = currentTime + 100000;  // 10Hz
    ctrlTime = currentTime + 50000;  // 20Hz
//  ctrlTime = currentTime + 20000;  // 50Hz
//  ctrlTime = currentTime + 10000;  // 100Hz

    // Cyclic Process
    
    // CountUp
    ctrlLoopCount++;
  }else {
    // Non-Cyclic Process

  }
}

#endif// USE_CTRL


