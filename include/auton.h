#include "api.h"
#include "main.h"

#ifndef PIDH
#define PIDH

#define STRAIGHT_KP 1
#define STRAIGHT_KI 0
#define STRAIGHT_KD 0

#define STRAIGHT_INTEGRAL_KI 40
#define STRAIGHT_MAX_INTEGRAL 14.5

#define TURN_KP 1
#define TURN_KI 0
#define TURN_KD 0

#define TURN_INTEGRAL_KI 30
#define TURN_MAX_INTEGRAL 25

#define HEADING_KP 1
#define HEADING_KI 0
#define HEADING_KD 0

#define HEADING_INTEGRAL_KI 0
#define HEADING_MAX_INTEGRAL 0

#define ARC_KP 1
#define ARC_KI 0
#define ARC_KD 0

#define ARC_INTEGRAL_KI 40
#define ARC_MAX_INTEGRAL 14.5


extern void driveTurn(int target);
extern void driveTurn2(int target);
extern void driveStraight(int target);
extern void driveStraight2(int target);
extern void driveArcL(double theta, double radius, int timeout);
extern void driveArcLF(double theta, double radius, int timeout);
extern void driveArcR(double theta, double radius, int timeout);
extern void driveArcRF(double theta, double radius, int timeout);
extern void driveStraightC(int target);
extern void setConstants(double kp, double ki, double kd);
extern double calcPID(double target, double input, int integralKi, int maxIntegral);
extern  int LBMacro;

#endif