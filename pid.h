#include "api.h"
#include "main.h"
#include "okapi/api.hpp"
#include "pros/api_legacy.h"

#ifndef PIDH
#define PIDH

extern float error;
extern void driveStraight(int target, int customtimeout); //Write same thing for driveturn also turn chasMove2 to chasMove
extern void driveTurn(int target, int customtimeout);
extern void driveStraight2(int target, int customtimeout); //Write same thing for driveturn also turn chasMove2 to chasMove
extern void driveTurn2(int target, int customtimeout);
extern void driveArcL(double theta, double radius, int timeout);
extern void driveArcR(double theta, double radius, int timeout);
#define STRAIGHT_KP 0.9 //0.9
#define STRAIGHT_KI 0.025//0.025
#define STRAIGHT_KD 0.3//0.3

#define STRAIGHT_INTEGRAL_KI 40
#define STRAIGHT_MAX_INTEGRAL 15

#define TURN_KP 10 //if the desmos stuff isnt done change this to 10
#define TURN_KI 10//0
#define TURN_KD 40

#define TURN_INTEGRAL_KI 40
#define TURN_MAX_INTEGRAL 15

#endif