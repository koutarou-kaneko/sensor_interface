/*
*/

#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param

#if 0
#include <stdarg.h>

void Log(char *fmt, ... ){
  char tmp[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(tmp, 128, fmt, args);
  va_end (args);
  SerialPrint(LOG_PORT, tmp);
}
#endif

static void inline SerialOpen(uint8_t port, uint32_t baud) {
  switch (port) {
  case 0: Serial.begin(baud); break;
  case 1: Serial1.begin(baud); break;
  case 2: Serial2.begin(baud); break;
  case 3: Serial3.begin(baud); break;
  }
}

static void inline SerialEnd(uint8_t port) {
  switch (port) {
  case 0: Serial.end(); break;
  case 1: Serial1.end(); break;
  case 2: Serial2.end(); break;
  case 3: Serial3.end(); break;
  }
}

uint8_t SerialRead(uint8_t port) {
  uint8_t c;
  switch (port) {
  case 0: c = Serial.read(); break;
  case 1: c = Serial1.read(); break;
  case 2: c = Serial2.read(); break;
  case 3: c = Serial3.read(); break;
  }
  return c;
}

uint8_t SerialAvailable(uint8_t port) {
  uint8_t ans=0;
  switch (port) {
  case 0: ans = Serial.available(); break;
  case 1: ans = Serial1.available(); break;
  case 2: ans = Serial2.available(); break;
  case 3: ans = Serial3.available(); break;
  }
  return ans;
}

static void inline SerialWrite(uint8_t port,uint8_t c){
  switch (port) {
  case 0: Serial.write(c); break;
  case 1: Serial1.write(c); break;
  case 2: Serial2.write(c); break;
  case 3: Serial3.write(c); break;
  }
}

static void inline SerialPrint(uint8_t port,char* c){
  switch (port) {
  case 0: Serial.print(c); break;
  case 1: Serial1.print(c); break;
  case 2: Serial2.print(c); break;
  case 3: Serial3.print(c); break;
  }
}

