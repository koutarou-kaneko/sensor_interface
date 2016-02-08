// ************************************************************************************************************
// board orientation and setup
// ************************************************************************************************************

//    +---LED---+      ^ Y
//    |         |      |
//    |         USB  Z ・-> X
//    |         |
//    +---------+

/*** I2C address ***/
#if !defined(MPU6050_ADDRESS)
  #define MPU6050_ADDRESS     0x68 // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board
#endif

#if !defined(MS561101BA_ADDRESS) 
  #define MS561101BA_ADDRESS 0x77 //CBR=0 0xEE I2C address when pin CSB is connected to LOW (GND)
  //#define MS561101BA_ADDRESS 0x76 //CBR=1 0xEC I2C address when pin CSB is connected to HIGH (VCC)
#endif

//MPU6050 Gyro LPF setting
#if defined(MPU6050_LPF_256HZ) || defined(MPU6050_LPF_188HZ) || defined(MPU6050_LPF_98HZ) || defined(MPU6050_LPF_42HZ) || defined(MPU6050_LPF_20HZ) || defined(MPU6050_LPF_10HZ) || defined(MPU6050_LPF_5HZ)
  #if defined(MPU6050_LPF_256HZ)
    #define MPU6050_DLPF_CFG   0
  #endif
  #if defined(MPU6050_LPF_188HZ)
    #define MPU6050_DLPF_CFG   1
  #endif
  #if defined(MPU6050_LPF_98HZ)
    #define MPU6050_DLPF_CFG   2
  #endif
  #if defined(MPU6050_LPF_42HZ)
    #define MPU6050_DLPF_CFG   3
  #endif
  #if defined(MPU6050_LPF_20HZ)
    #define MPU6050_DLPF_CFG   4
  #endif
  #if defined(MPU6050_LPF_10HZ)
    #define MPU6050_DLPF_CFG   5
  #endif
  #if defined(MPU6050_LPF_5HZ)
    #define MPU6050_DLPF_CFG   6
  #endif
#else
    //Default settings LPF 256Hz/8000Hz sample
    #define MPU6050_DLPF_CFG   0
#endif

uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;
  
// ************************************************************************************************************
// I2C general functions
// ************************************************************************************************************

void i2c_init(void) {
  #if defined(INTERNAL_I2C_PULLUPS)
    I2C_PULLUPS_ENABLE
  #else
    I2C_PULLUPS_DISABLE
  #endif
  TWSR = 0;                                    // no prescaler => prescaler = 1
  TWBR = ((F_CPU / I2C_SPEED) - 16) / 2;       // change the I2C clock rate
  TWCR = 1<<TWEN;                              // enable twi module, no interrupt
}

void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) ; // send REPEAT START condition
  waitTransmissionI2C();                       // wait until transmission completed
  TWDR = address;                              // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();                       // wail until transmission completed
}

void i2c_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}

void i2c_write(uint8_t data ) {
  TWDR = data;                                 // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
}

uint8_t i2c_read(uint8_t ack) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (ack? (1<<TWEA) : 0);
  waitTransmissionI2C();
  uint8_t r = TWDR;
  if (!ack) i2c_stop();
  return r;
}

uint8_t i2c_readAck() {
  return i2c_read(1);
}

uint8_t i2c_readNak(void) {
  return i2c_read(0);
}

void waitTransmissionI2C() {
  uint16_t count = 255;
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      i2c_errors_count++;
      break;
    }
  }
}

size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size) {
  i2c_rep_start((add<<1) | 1);  // I2C read direction
  size_t bytes_read = 0;
  uint8_t *b = (uint8_t*)buf;
  while (size--) {
    /* acknowledge all but the final byte */
    *b++ = i2c_read(size > 0);
    /* TODO catch I2C errors here and abort */
    bytes_read++;
  }
  return bytes_read;
}

size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  return i2c_read_to_buf(add, buf, size);
}

/* transform a series of bytes from big endian to little
   endian and vice versa. */
void swap_endianness(void *buf, size_t size) {
  /* we swap in-place, so we only have to
  * place _one_ element on a temporary tray
  */
  uint8_t tray;
  uint8_t *from;
  uint8_t *to;
  /* keep swapping until the pointers have assed each other */
  for (from = (uint8_t*)buf, to = &from[size-1]; from < to; from++, to--) {
    tray = *from;
    *from = *to;
    *to = tray;
  }
}

void i2c_getSixRawADC(uint8_t add, uint8_t reg) {
  i2c_read_reg_to_buf(add, reg, &rawADC, 6);
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_write(val);        // value to write in register
  i2c_stop();
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg) {
  uint8_t val;
  i2c_read_reg_to_buf(add, reg, &val, 1);
  return val;
}

// ****************
// GYRO common part
// ****************
void GYRO_Common() {
  static int16_t previousGyroADC[3] = {0,0,0};
  static int32_t g[3];
  uint8_t axis, tilt=0;

#if defined MMGYRO       
  // Moving Average Gyros by Magnetron1
  //---------------------------------------------------
  static int16_t mediaMobileGyroADC[3][MMGYROVECTORLENGTH];
  static int32_t mediaMobileGyroADCSum[3];
  static uint8_t mediaMobileGyroIDX;
  //---------------------------------------------------
#endif

  if (calibratingG>0) {
    for (axis = 0; axis < 3; axis++) {
      // Reset g[axis] at start of calibration
      if (calibratingG == 512) {
        g[axis]=0;
        
        #if defined(GYROCALIBRATIONFAILSAFE)
            previousGyroADC[axis] = imu.gyroADC[axis];
          }
          if (calibratingG % 10 == 0) {
            if(abs(imu.gyroADC[axis] - previousGyroADC[axis]) > 8) tilt=1;
            previousGyroADC[axis] = imu.gyroADC[axis];
       #endif
      }
      // Sum up 512 readings
      g[axis] +=imu.gyroADC[axis];
      // Clear global variables for next reading
      imu.gyroADC[axis]=0;
      gyroZero[axis]=0;
      if (calibratingG == 1) {
        gyroZero[axis]=g[axis]>>9;
      #if defined(BUZZER)
        alarmArray[7] = 4;
      #else
        blinkLED(10,15,1); //the delay causes to beep the buzzer really long 
      #endif
      }
    }
    #if defined(GYROCALIBRATIONFAILSAFE)
      if(tilt) {
        calibratingG=1000;
        LEDPIN_ON;
      } else {
        calibratingG--;
        LEDPIN_OFF;
      }
      return;
    #else
      calibratingG--;
    #endif
    
  }

#ifdef MMGYRO       
  mediaMobileGyroIDX = ++mediaMobileGyroIDX % conf.mmgyro;
  for (axis = 0; axis < 3; axis++) {
    imu.gyroADC[axis]  -= gyroZero[axis];
    mediaMobileGyroADCSum[axis] -= mediaMobileGyroADC[axis][mediaMobileGyroIDX];
    //anti gyro glitch, limit the variation between two consecutive readings
    mediaMobileGyroADC[axis][mediaMobileGyroIDX] = constrain(imu.gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
    mediaMobileGyroADCSum[axis] += mediaMobileGyroADC[axis][mediaMobileGyroIDX];
    imu.gyroADC[axis] = mediaMobileGyroADCSum[axis] / conf.mmgyro;
#else 
  for (axis = 0; axis < 3; axis++) {
    imu.gyroADC[axis]  -= gyroZero[axis];
    //anti gyro glitch, limit the variation between two consecutive readings
    imu.gyroADC[axis] = constrain(imu.gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
#endif    
    previousGyroADC[axis] = imu.gyroADC[axis];
  }

}

// ****************
// ACC common part
// ****************
void ACC_Common() {
  static int32_t a[3];
  
  if (calibratingA>0) {
    for (uint8_t axis = 0; axis < 3; axis++) {
      // Reset a[axis] at start of calibration
      if (calibratingA == 512) a[axis]=0;
      // Sum up 512 readings
      a[axis] +=imu.accADC[axis];
      // Clear global variables for next reading
      imu.accADC[axis]=0;
      global_conf.accZero[axis]=0;
    }
    // Calculate average, shift Z down by ACC_1G and store values in EEPROM at end of calibration
    if (calibratingA == 1) {
      global_conf.accZero[ROLL]  = a[ROLL]>>9;
      global_conf.accZero[PITCH] = a[PITCH]>>9;
      global_conf.accZero[YAW]   = (a[YAW]>>9)-ACC_1G; // for nunchuk 200=1G
      conf.angleTrim[ROLL]   = 0;
      conf.angleTrim[PITCH]  = 0;
      writeGlobalSet(1); // write accZero in EEPROM
    }
    calibratingA--;
  }

  imu.accADC[ROLL]  -=  global_conf.accZero[ROLL] ;
  imu.accADC[PITCH] -=  global_conf.accZero[PITCH];
  imu.accADC[YAW]   -=  global_conf.accZero[YAW] ;
}

// ************************************************************************************************************
// I2C Barometer MS561101BA
// ************************************************************************************************************
//
// specs are here: http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
// useful info on pages 7 -> 12


#if defined(MS561101BA)

// registers of the device
#define MS561101BA_PRESSURE    0x40
#define MS561101BA_TEMPERATURE 0x50
#define MS561101BA_RESET       0x1E

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256  0x00
#define MS561101BA_OSR_512  0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define OSR MS561101BA_OSR_4096

static struct {
  // sensor registers from the MS561101BA datasheet
  uint16_t c[7];
  union {
    uint32_t val;
    uint8_t raw[4];
  } ut;  //uncompensated T
  union {
    uint32_t val;
    uint8_t raw[4];
  } up;  //uncompensated P
  uint8_t  state;
  uint32_t deadline;
} ms561101ba_ctx;

void i2c_MS561101BA_reset(){
  i2c_writeReg(MS561101BA_ADDRESS, MS561101BA_RESET, 0);
}

void i2c_MS561101BA_readCalibration(){
  union {
    uint16_t val;
    uint8_t raw[2];
  } data;

  for (uint8_t i = 0; i < 6; i++) {
    i2c_rep_start(MS561101BA_ADDRESS << 1);
    i2c_write(0xA2 + 2 * i);

    delay(10);
    i2c_rep_start((MS561101BA_ADDRESS << 1) | 1);  //I2C read direction => 1

    delay(10);
    data.raw[1] = i2c_readAck();  // read a 16 bit register
    data.raw[0] = i2c_readNak();
    ms561101ba_ctx.c[i + 1] = data.val;
  }
}

void  Baro_init() {
  delay(10);
  i2c_MS561101BA_reset();

  delay(100);
  i2c_MS561101BA_readCalibration();

  delay(10);
  i2c_MS561101BA_UT_Start(); 

  ms561101ba_ctx.deadline = currentTime + 10000; 
}

/// read uncompensated temperature value: send command first
void i2c_MS561101BA_UT_Start() {
  i2c_rep_start(MS561101BA_ADDRESS << 1);  // I2C write direction
  i2c_write(MS561101BA_TEMPERATURE + OSR);  // register selection
  i2c_stop();
}

/// read uncompensated pressure value: send command first
void i2c_MS561101BA_UP_Start () {
  i2c_rep_start(MS561101BA_ADDRESS << 1);  // I2C write direction
  i2c_write(MS561101BA_PRESSURE + OSR);  // register selection
  i2c_stop();
}

/// read uncompensated pressure value: read result bytes
void i2c_MS561101BA_UP_Read () {
  i2c_rep_start(MS561101BA_ADDRESS<<1);
  i2c_write(0);
  i2c_rep_start((MS561101BA_ADDRESS<<1) | 1);
  ms561101ba_ctx.up.raw[2] = i2c_readAck();
  ms561101ba_ctx.up.raw[1] = i2c_readAck();
  ms561101ba_ctx.up.raw[0] = i2c_readNak();
}

/// read uncompensated temperature value: read result bytes
void i2c_MS561101BA_UT_Read() {
  i2c_rep_start(MS561101BA_ADDRESS << 1);
  i2c_write(0);
  i2c_rep_start((MS561101BA_ADDRESS << 1) | 1);
  ms561101ba_ctx.ut.raw[2] = i2c_readAck();
  ms561101ba_ctx.ut.raw[1] = i2c_readAck();
  ms561101ba_ctx.ut.raw[0] = i2c_readNak();
}

void i2c_MS561101BA_Calculate() {
  int32_t off2, sens2, delt;

  int64_t dT =
      (int32_t)ms561101ba_ctx.ut.val -
      ((int32_t)ms561101ba_ctx.c[5] << 8);
  baroTemperature = 2000 + ((dT * ms561101ba_ctx.c[6]) >> 23);
  int64_t off =
      ((uint32_t)ms561101ba_ctx.c[2] << 16) +
      ((dT * ms561101ba_ctx.c[4]) >> 7);
  int64_t sens =
      ((uint32_t)ms561101ba_ctx.c[1] << 15) +
      ((dT * ms561101ba_ctx.c[3]) >> 8);

  if (baroTemperature < 2000) {  // temperature lower than 20st.C 
    delt = baroTemperature - 2000;
    delt  = 5 * delt * delt;
    off2  = delt >> 1;
    sens2 = delt >> 2;
    if (baroTemperature < -1500) {  // temperature lower than -15st.C
      delt  = baroTemperature+1500;
      delt  = delt * delt;
      off2  += 7 * delt;
      sens2 += (11 * delt) >> 1;
    }
    off  -= off2; 
    sens -= sens2;
  }

  baroPressure = (((ms561101ba_ctx.up.val * sens ) >> 21) - off) >> 15;
}

/// @return 0: no data available, no computation;
///         1: new value available;
///         2: no new value, but computation time
uint8_t Baro_update() {
  if ((int32_t)(currentTime - ms561101ba_ctx.deadline) < 0) return 0;

  // first UT conversion is started in init procedure
  // UT and UP conversion take 8.5ms so we do next reading after 10ms
  ms561101ba_ctx.deadline = currentTime + 10000;

  // change the I2C clock rate to 400kHz, MS5611 is ok with this speed
  TWBR = ((F_CPU / 400000L) - 16) / 2;

  if (ms561101ba_ctx.state == 0) {
    i2c_MS561101BA_UT_Read(); 
    i2c_MS561101BA_UP_Start();
    
    // moved here for less timecycle spike
    Baro_Common();
    ms561101ba_ctx.state = 1;
    return 1;
  } else {
    i2c_MS561101BA_UP_Read();
    i2c_MS561101BA_UT_Start(); 
    i2c_MS561101BA_Calculate();
    ms561101ba_ctx.state = 0; 
    return 2;
  }
}

void Baro_Common() {
  static int32_t baroHistTab[BARO_TAB_SIZE];
  static uint8_t baroHistIdx;
   
  uint8_t indexplus1 = (baroHistIdx + 1);
  if (indexplus1 == BARO_TAB_SIZE) indexplus1 = 0;

  baroHistTab[baroHistIdx] = baroPressure;
  baroPressureSum += baroHistTab[baroHistIdx];
  baroPressureSum -= baroHistTab[indexplus1];
  baroHistIdx = indexplus1;  
}
#endif

// ************************************************************************************************************
// I2C Compass common function
// ************************************************************************************************************

// gain for each axis, populated at sensor init
static float magGain[3] = {1.0, 1.0, 1.0};
static uint8_t magInit = 0;

/// @return 1 when news values are available, 0 otherwise
uint8_t Mag_getADC() {
  static uint32_t t, tCal = 0;
  static int16_t magZeroTempMin[3];
  static int16_t magZeroTempMax[3];
  uint8_t axis;

  // each read is spaced by 100ms
  if ((int32_t)(currentTime - t) < 0)
    return 0; 

  t = currentTime + 100000;
  TWBR = ((F_CPU / 400000L) - 16) / 2;  // change the I2C clock rate to 400kHz

  Device_Mag_getADC();

  imu.magADC[ROLL]  = imu.magADC[ROLL]  * magGain[ROLL];
  imu.magADC[PITCH] = imu.magADC[PITCH] * magGain[PITCH];
  imu.magADC[YAW]   = imu.magADC[YAW]   * magGain[YAW];

  if (f.CALIBRATE_MAG) {
    tCal = t;
    for(axis = 0; axis < 3; axis++) {
      global_conf.magZero[axis] = 0;
      magZeroTempMin[axis] = imu.magADC[axis];
      magZeroTempMax[axis] = imu.magADC[axis];
    }
    f.CALIBRATE_MAG = 0;
  }

  // we apply offset only once mag calibration is done
  if (magInit) {
    imu.magADC[ROLL]  -= global_conf.magZero[ROLL];
    imu.magADC[PITCH] -= global_conf.magZero[PITCH];
    imu.magADC[YAW]   -= global_conf.magZero[YAW];
  }
 
  if (tCal != 0) {
    // 30s: you have 30s to turn the multi in all directions
    if ((t - tCal) < 30000000) {
      LEDPIN_TOGGLE;
      for (axis = 0; axis < 3; axis++) {
        if (imu.magADC[axis] < magZeroTempMin[axis])
          magZeroTempMin[axis] = imu.magADC[axis];
        if (imu.magADC[axis] > magZeroTempMax[axis])
          magZeroTempMax[axis] = imu.magADC[axis];
      }
    } else {
      tCal = 0;
      // http://www.aichi-mi.com/old_pages/5_2_transistor_gijutu/transistor_gijutu.htm
      for (axis = 0; axis < 3; axis++)
        global_conf.magZero[axis] =
            (magZeroTempMin[axis] + magZeroTempMax[axis]) >> 1;
      writeGlobalSet(1);
    }
  } else {
#if defined(SENSORS_TILT_45DEG_LEFT)
    int16_t temp = ((imu.magADC[PITCH] - imu.magADC[ROLL] ) * 7) / 10;
    magADC[ROLL] = ((imu.magADC[ROLL]  + imu.magADC[PITCH]) * 7) / 10;
    imu.magADC[PITCH] = temp;
#endif

#if defined(SENSORS_TILT_45DEG_RIGHT)
    int16_t temp = ((imu.magADC[PITCH] + imu.magADC[ROLL]) * 7) / 10;
    imu.magADC[ROLL] = ((imu.magADC[ROLL] - imu.magADC[PITCH]) * 7) / 10;
    imu.magADC[PITCH] = temp;
#endif
  }
  return 1;
}



// ************************************************************************************************************
// I2C Compass AK8975
// ************************************************************************************************************
// I2C adress: 0x0C (7bit)
// ************************************************************************************************************
#if defined(AK8975)
  #define MAG_ADDRESS 0x0C
  #define MAG_DATA_REGISTER 0x03
  
  void Mag_init() {
    delay(100);


#if defined(MPU6050_I2C_AUX_MASTER)
    i2c_writeReg(MPU6050_ADDRESS, 0x34, 0x80); 
#else
    i2c_writeReg(MAG_ADDRESS,0x0a,0x01);  //Start the first conversion
#endif

    delay(100);
    magInit = 1;
  }

  void Device_Mag_getADC() {

#if defined(MPU6050_I2C_AUX_MASTER)
    i2c_getSixRawADC(MPU6050_ADDRESS, 0x49); 
#else
    i2c_getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER);
#endif

#if 1
    MAG_ORIENTATION( ((rawADC[1]<<8) | rawADC[0]) ,
                     ((rawADC[3]<<8) | rawADC[2]) ,
                     ((rawADC[5]<<8) | rawADC[4]) );
#else
    imu.magADC[ROLL] = ((rawADC[1]<<8) | rawADC[0] );
    imu.magADC[PITCH] =  ((rawADC[3]<<8) | rawADC[2]);
    imu.magADC[YAW] =   ((rawADC[5]<<8) | rawADC[4]);

#endif

    //Start another meassurement
#if defined(MPU6050_I2C_AUX_MASTER)
    i2c_writeReg(MPU6050_ADDRESS, 0x34, 0x80);  //no need for i2c master mode?
#else
    i2c_writeReg(MAG_ADDRESS,0x0a,0x01);  //Start the first conversion
#endif
  }
#endif


// ************************************************************************************************************
// I2C Gyroscope and Accelerometer MPU6050
// ************************************************************************************************************
#if defined(MPU6050)

void Gyro_init() {
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(5);
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2c_writeReg(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2c_writeReg(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
  // enable I2C bypass for AUX I2C
#if defined(MAG)
  i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x02);           //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
   #endif
}

void Gyro_getADC () {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x43);
#if 0 //for old version
  GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>2 , // range: +/- 8192; +/- 2000 deg/sec
                    ((rawADC[2]<<8) | rawADC[3])>>2 ,
                    ((rawADC[4]<<8) | rawADC[5])>>2 );
#else //test
  imu.gyroADC[ROLL] = ((rawADC[0]<<8) | rawADC[1])>>2 ;
  imu.gyroADC[PITCH] = ((rawADC[2]<<8) | rawADC[3])>>2 ;
  imu.gyroADC[YAW] = ((rawADC[4]<<8) | rawADC[5])>>2 ;
#endif
  GYRO_Common();
}

void ACC_init () {
  i2c_writeReg(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
  //note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
  //confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480

#if defined(MPU6050_I2C_AUX_MASTER)
  //at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
  //now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
  i2c_writeReg(MPU6050_ADDRESS, 0x6A, 0b00100000); //USER_CTRL -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
  i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x00); //INT_PIN_CFG -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
  i2c_writeReg(MPU6050_ADDRESS, 0x24, 0x0D); //I2C_MST_CTRL -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
  i2c_writeReg(MPU6050_ADDRESS, 0x25, 0x80|MAG_ADDRESS);//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
  i2c_writeReg(MPU6050_ADDRESS, 0x26, MAG_DATA_REGISTER);//I2C_SLV0_REG -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
  i2c_writeReg(MPU6050_ADDRESS, 0x27, 0x86); //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)


  i2c_writeReg(MPU6050_ADDRESS, 0x31, MAG_ADDRESS);
  i2c_writeReg(MPU6050_ADDRESS, 0x32, 0x0a);
  i2c_writeReg(MPU6050_ADDRESS, 0x33, 0x01);
  //i2c_writeReg(MPU6050_ADDRESS, 0x34, 0x80); 

#endif

}

void ACC_getADC () {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x3B);
#if 0
  ACC_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>3 ,
                   ((rawADC[2]<<8) | rawADC[3])>>3 ,
                   ((rawADC[4]<<8) | rawADC[5])>>3 );
#else
  imu.accADC[ROLL] = ((rawADC[0]<<8) | rawADC[1])>>3 ;
  imu.accADC[PITCH] = ((rawADC[2]<<8) | rawADC[3])>>3 ;
  imu.accADC[YAW] = ((rawADC[4]<<8) | rawADC[5])>>3 ;
#endif
  ACC_Common();
}

#endif


void initSensors() {
  delay(200);
  POWERPIN_ON;
  delay(100);
  i2c_init();
  delay(100);
  if (GYRO) Gyro_init();
  if (ACC) ACC_init();
  if (MAG) Mag_init();

  if (BARO) Baro_init();

  f.I2C_INIT_DONE = 1;
}
