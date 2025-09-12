#include "api.h"
#include "pros/adi.hpp"
#include "main.h"
#include "pros/motors.h"
#include "pros/distance.hpp"

#ifndef ROBOTH
#define ROBOTH

extern pros::Controller con;

extern pros::Motor LF;
extern pros::Motor LB;
extern pros::Motor RF;
extern pros::Motor RB; 
extern pros::Motor LM1;
extern pros::Motor LM2;
extern pros::Motor RM1;
extern pros::Motor RM2; 
extern pros::Motor Intake1;
extern pros::Motor Intake2;
extern pros::Motor Intake3;
extern pros::ADIDigitalOut basket;
extern pros::Vision vision;
extern pros::Optical optical;
extern pros::Imu imu; 
extern pros::Motor_Group Intake;
#endif

