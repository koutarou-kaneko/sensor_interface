#ifndef GLOBAL_H
#define GLOBAL_H

#include "types.h"

#ifdef MAIN
#define EXTERN
#else// MAIN
#define EXTERN extern
#endif// MAIN

EXTERN int16_t acc1G[3];
EXTERN int16_t motor[8];
EXTERN int16_t servo[8];
EXTERN imu_t imu;
EXTERN alt_t alt;
EXTERN att_t att;

EXTERN int16_t debug[4];

EXTERN uint32_t currentTime;

#endif// GLOBAL_H


