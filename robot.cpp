#include "api.h"
#include "robot.h"
#include "main.h"
#include "pros/distance.hpp"
#include "pros/motors.h"
using namespace pros;
using namespace std;

#define LF_PORT 9
#define LB_PORT 3
#define RF_PORT 7
#define RB_PORT 1
#define IMU_PORT 20 
#define LM1_PORT 4
#define RM1_PORT 8
#define Intake1_PORT 10 // 5.5 watt
#define Intake2_PORT 13 // 5.5 Watt
#define Intake3_PORT 14 //11 watt
// #define basket_PORT 20
#define VISION_PORT 15
#define DISTANCE_PORT 1
#define optical_PORT 7


 

pros::Controller con(pros::E_CONTROLLER_MASTER);

pros::Motor LF(LF_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LB(LB_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LM1(LM1_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RF(RF_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RB(RB_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RM1(RM1_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor Intake1(Intake1_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor Intake2(Intake2_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor Intake3(Intake3_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor_Group intake({Intake1, Intake2, Intake3});
// pros::Motor basket(basket_PORT, pros::E_MOTOR_GEARSET_06, false);


pros::Imu imu(IMU_PORT);
pros::Vision vision(VISION_PORT);
pros::Distance distance_sensor(DISTANCE_PORT);
pros::Optical optical(optical_PORT);

// if piston starts incorectlly change false to true
// pros::ADIDigitalOut intakepiston('C', true);
// pros::ADIDigitalOut wings('A', false);
// pros::ADIDigitalOut hangpiston('E', false);
// pros::ADIDigitalOut blocker('F', false);
// pros::ADIDigitalOut hanger('F', true);
pros::ADIDigitalOut basket('A', false);



pros::Distance distancesensor(2);
