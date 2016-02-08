/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
March  2013     V2.2
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

#include <avr/io.h>

#include "config.h"
#include "def.h"

#include <avr/pgmspace.h>
#define  VERSION  221


// *************************
// motor and servo functions
// *************************
static int32_t motor[8]; 
static int16_t servo[8] = {2000,1060,2000,1500,1500,1500,1500,1500};



void setup() {
  LEDPIN_PINMODE;

  initOutput();

  ros_setup();
}


// ******** Main Loop *********
void loop () 
{
  ros_loop();
  writeMotors();
  //writeAllMotors(1000);
}

