#include "api.h"
#include "main.h"
#include "pid.h"
#include "robot.h"

using namespace pros;
using namespace c;
using namespace std;

double vKp;
double vKi;
double vKd;
float error;
float viewvol;
double prevpower;
double currentPower;
double preverror;

int integral; 
int derivative;

double power;

void setConstants(double kp, double ki, double kd){
    vKp = kp;
    vKi = ki;
    vKd = kd;

}

void resetEncoders() {
    LF.tare_position();
    LB.tare_position();
    RF.tare_position();
    RB.tare_position();
    RM1.tare_position();
    LM1.tare_position();
}

void chasMove(int VLF, int VLB, int VRF, int VRB, int VRM1, int VLM1) {

    LF.move(VLF);
    LB.move(VLB);
    RF.move(VRF);
    RB.move(VRB);
    RM1.move(VRM1);
    LM1.move(VLM1);
}

int slew = 3;
double calcPID(int target, double input, int integralKi, int maxIntegral, bool slewOn) {
int integral = 0; 
    preverror = error;
    error = target - input; 
    //prevpower = power; 
// preverror is the preivous error it is used for D
//error is used to calc. p
//Prevpower/power is used for printing
    if(std::abs(error) < integralKi) {
        integral += error ; 
    } else {
        integral = 0;
    }
//Integral KI is used to know when to start calculating I
    if(integral >= 0) {

        integral = std::min(integral, maxIntegral);
    } else {
       integral = std::max(integral, -maxIntegral); 
    }
//Its finding the smaller value. Dont let integral go past max integral
    derivative = preverror - error; 
//This is calculating derivitve
    power = (vKp * error) + (vKi * integral) - (vKd * derivative);
    //currentPower = power;
    return power; 
}
//Here we are calculating the final power

//DriveStraight
void driveStraight(int target, int customtimeout) {
    int timeout = customtimeout;
    if (abs(target) < 800) {
        timeout = 2700;
    } 
    //target = -target;
    //This sets the time in ms where the code will time out even if PID is not complete
    //imu.tare_hEAding();
    double voltage;
    double encoderAvg;
    int count = 0; 
    double init_heading = imu.get_heading(); 
    double heading_error = 0;
    int cycle = 0; //controller display cycle 
    int time = 0;

    con.clear();
    resetEncoders();

    while(true){
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
            
        encoderAvg = LB.get_position(); // + RB.get_position()) / 2;
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, false);
        if (voltage > 127){
            voltage = 127;
        } else if (voltage < -127){
            voltage = -127;
        }

        if(init_heading > 180) {

            init_heading = (360 - init_heading);
        }
        if(imu.get_heading() < 180) {

            heading_error = init_heading - imu.get_heading();
        } else {

            heading_error = ((360 - imu.get_heading()) - init_heading); 
        }
         //heading_error = 0; //if heading still does not work
         heading_error = heading_error * 10;

//If drivestraight is incorect direction change the + and - on voltage + heading_error
        chasMove( (voltage + heading_error), (voltage + heading_error), (voltage - heading_error), (voltage - heading_error), (voltage- heading_error), (voltage - heading_error));
        if (abs(target - encoderAvg) <= 7) count++;
        if (count >= 20 || time > timeout) {
          break; 
        }
        delay(10);
        time += 10;

    if (time % 100 == 0) con.clear();
	else if (time % 50 == 0){
cycle++;
int hh = calcPID(1500, 50, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, false);
if((cycle+1) % 3 == 0) con.print(0, 0, "ERROR: %f", (target - encoderAvg));
if((cycle+2) % 3 == 0) con.print(1, 0, "EcoderAvg %f", encoderAvg); 
if((cycle+3) % 3 == 0) con.print(3, 0, "ERROR: %i", calcPID(1500, 50, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, false));
}
    }
LF.brake();
LB.brake();
RF.brake();
RB.brake();
RM1.brake();
LM1.brake();

}
//Drivestraight Global
void driveStraight2(int target, int customtimeout) {
    int timeout = customtimeout;
    if (abs(target) < 800) {
        timeout = 2700;
    } 
    //target = -target;
    //This sets the time in ms where the code will time out even if PID is not complete
    //imu.tare_heading();
    double voltage;
    double encoderAvg;
    int count = 0; 
    double init_heading = imu.get_heading(); 
    double heading_error = 0;
    int cycle = 0; //controller display cycle 
    int time = 0;

    con.clear();
    resetEncoders();

    while(true){
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
            
        encoderAvg = LB.get_position(); //+ RB.get_position() / 2;
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, false);
        if (voltage > 127){
            voltage = 127;
        } else if (voltage < -127){
            voltage = -127;
        }

        if(init_heading > 180) {

            init_heading = (360 - init_heading);
        }
        if(imu.get_heading() < 180) {

            heading_error = init_heading - imu.get_heading();
        } else {

            heading_error = ((360 - imu.get_heading()) - init_heading); 
        }
         //heading_error = 0; //if heading still does not work
         heading_error = heading_error * 10;

//If drivestraight is incorect direction change the + and - on voltage + heading_error
        chasMove( (voltage), (voltage ), (voltage ), (voltage ), (voltage), (voltage ));
        if (abs(target - encoderAvg) <= 7) count++;
        if (count >= 20 || time > timeout) {
          break; 
        }
        delay(10);
        time += 10;

    if (time % 100 == 0) con.clear();
	else if (time % 50 == 0){
cycle++;
int hh = calcPID(1500, 50, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, false);
if((cycle+1) % 3 == 0) con.print(0, 0, "ERROR: %f", (target - encoderAvg));
if((cycle+2) % 3 == 0) con.print(1, 0, "IMU %f", imu.get_heading()); 
if((cycle+3) % 3 == 0) con.print(3, 0, "ERROR: %i", calcPID(1500, 50, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, false));
}
    }
LF.brake();
LB.brake();
RF.brake();
RB.brake();
RM1.brake();
LM1.brake();


}

void driveTurn(int target, int customtimeout)  { //target is inputted in autons

    double kpdes = 0;
    double x = 0;
    x = double(abs(target));
    kpdes = ( -0.0000000275*pow(x,4)) + (0.00000948133*pow(x,3)) + (-0.000977224*pow(x,2)) + (0.0151985*pow(x,1)) + 6.12023;


    setConstants(TURN_KP, TURN_KI, TURN_KD);
    
    int timeout = customtimeout;

    if (abs(target) < 30) {
        timeout = 1900;
    } else {
        timeout = 2100;
    }
    
    imu.tare_heading();
    int time = 0;
    int cycle = 0;
    
    

    double voltage;
    double position;
    int count = 0;
    
    while(true) {
    
        position = imu.get_heading(); //this is where the units are set to be degrees
        if(abs(target) < 170){
            if (position > 180) {
                position = ((360 - position) * -1);
        }
        } else if (target <= -170){
            if (position > 30){
                position = ((360 - position) * -1);
            }
            
        } else {
            if (position > 330) {
                position = ((360 - position) * -1);
            }
        }
        

        voltage = calcPID(target, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL, false);
        // con.print(1, 0, "%2f", voltage);
        
        chasMove(voltage, voltage, -voltage, -voltage, -voltage, voltage);
        
        if (abs(target - position) <= 0.9) count++; //0.35
        if (count >= 40 || time > timeout) {
            imu.tare_heading();
            break;
        } 
        

        
        if (time % 100 == 0) con.clear(); else if (time % 50 == 0) {
            cycle++;
            if ((cycle+1) % 3 == 0) con.print(0, 0, "Error | %2f", target-position); 
            if ((cycle+2) % 3 == 0) con.print(1, 0, "Integral | %2f", position); //autstr //%s
            // if ((cycle+3) % 3 == 0) con.print(2, 0, "Integral: %f", viewIntegral);
        }
        time += 10;
        delay(10);
    }

    chasMove(0, 0, 0, 0, 0, 0);

}
//Driveturn global
void driveTurn2(int target, int customtimeout)  { //target is inputted in autons
int turnv = 0;
double position;
position = imu.get_heading(); //this is where the units are set to be degrees
    if (position > 180){
        position = ((360 - position) * -1 );
    }

    if((target < 0) && (position > 0)){
        if((position - target) >= 180){
            target = target + 360;
            position = imu.get_heading();
            turnv = (target - position); // target + position
        } else {
             turnv = (abs(position) + abs(target));
        }
    } else if ((target > 0) && (position < 0)){
        if((target - position) >= 180){
           position = imu.get_heading();
            turnv = abs(abs(position) - abs(target));
        } else {
            turnv = (abs(position) + target);
        }
    } else {
         turnv = abs(abs(position) - abs(target));
    }
    double kpdes = 0;
    double x = 0;
    x = double(abs(turnv));
    kpdes = ( -0.0000000275*pow(x,4)) + (0.00000948133*pow(x,3)) + (-0.000977224*pow(x,2)) + (0.0151985*pow(x,1)) + 6.12023;


    setConstants(TURN_KP, TURN_KI, TURN_KD);
    
    int timeout = customtimeout;

    if (abs(target) < 30) {
        timeout = 1900;
    } else {
        timeout = 2100;
    }
    
    int time = 0;
    int cycle = 0;

    double voltage;

    int count = 0;
    
    while(true) {
    
position = imu.get_heading(); //this is where the units are set to be degrees
    if (position > 180){
        position = ((360 - position) * -1 );
    }

    if((target < 0) && (position > 0)){
        if((position - target) >= 180){
            target = target + 360;
            position = imu.get_heading();
            turnv = (target - position); // target + position
        } else {
             turnv = (abs(position) + abs(target));
        }
    } else if ((target > 0) && (position < 0)){
        if((target - position) >= 180){
           position = imu.get_heading();
            turnv = abs(abs(position) - abs(target));
        } else {
            turnv = (abs(position) + target);
        }
    } else {
         turnv = abs(abs(position) - abs(target));
    }
        

        voltage = calcPID(target, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL, false);
        // con.print(1, 0, "%2f", voltage);
        
        chasMove(voltage, voltage, -voltage, -voltage, -voltage, voltage);
        
        if (abs(target - position) <= 0.9) count++; //0.35
        if (count >= 40 || time > timeout) {
            break;
        } 
        

        
        if (time % 100 == 0) con.clear(); else if (time % 50 == 0) {
            cycle++;
            if ((cycle+1) % 3 == 0) con.print(0, 0, "Error | %2f", target); 
            if ((cycle+2) % 3 == 0) con.print(1, 0, "Integral | %2f", position); //autstr //%s
            if ((cycle+3) % 3 == 0) con.print(2, 0, "IMU: %f", 5.5);
        }
        time += 10;
        delay(10);
    }

    chasMove(0, 0, 0, 0, 0, 0);

}

//driveArcL
//radius is encoder units
// increase radius to make bigger arc
//increase degrees if you want it to turn more (ex 90 degrees, 180 degrees, etc)
//in order to go backwards set theta negitive
//Theta is the first value and radius is the second
void driveArcL(double theta, double radius, int timeout){
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD); //<--- edit the PID Values
    resetEncoders();
    int integral = 0; 
    double ltarget = 0;
    double rtarget = 0;
    double pi = 3.14159265359;
    double init_heading = imu.get_heading();
    int count = 0;
    int time = 0;

     timeout = 5000;

    ltarget = double((theta / 360) * 2 * pi * radius);
    rtarget = double((theta / 360) * 2 * pi * (radius + 800));
    while (true){
    double heading = imu.get_heading() - init_heading;
    if (theta > 0){ 
    if (heading > 30 ){
        heading = heading - 360;
    }
    } else {
        if (heading > 300 ){
        heading = heading - 360;
    }
    }
    int encoderAvgL = LB.get_position(); // + RB.get_position()) / 2;
    int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, false);
        if (voltageL > 127){ //if the PID becomes innacurate get rid of the - fix values and make voltageL > 100
            voltageL = 127;
        } else if (voltageL < -127){
            voltageL = -127;
        }
    int encoderAvgR = RB.get_position(); // + RB.get_position()) / 2;
    int voltageR = calcPID(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, false);
        if (voltageR > 127){
            voltageR = 127;
        } else if (voltageR < -127){
            voltageR = -127;
        }

    int leftcorrect = (encoderAvgL * 360) / (2 * pi * radius);
    int fix = leftcorrect + heading;
    fix = fix * 10; //reaction sensitivity
    con.print(0, 0, "ERROR: %f       ", float(ltarget));
        if ((abs(ltarget - encoderAvgL) <= 0.9) && (abs(rtarget - encoderAvgL) <= 0.9)) count++; //0.35
        if (count >= 40 || time > timeout) {
           //imu.tare_heading();
            break;
        }
    

    chasMove(voltageL - fix, voltageL - fix, voltageR + fix, voltageR + fix, voltageR + fix, voltageL - fix);
    time += 10;
    delay(10);



    } }
    //driveArcR
    //radius is encoder units
    // increase radius to make bigger arc
    //increase degrees if you want it to turn more (ex 90 degrees, 180 degrees, etc)
    //in order to go backwards set radius negitive
    //Theta is the first value and radius is the second
    void driveArcR(double theta, double radius, int timeout){
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD); //<--- edit the PID Values
    resetEncoders();
    double init_heading = imu.get_heading();
    int integral = 0; 
    double ltarget = 0;
    double rtarget = 0;
    double pi = 3.14159265359;
    //imu.tare_heading();
    int count = 0;
    int time = 0;

     //timeout = 5000;

    ltarget = double((theta / 360) * 2 * pi * (radius + 800));
    rtarget = double((theta / 360) * 2 * pi * radius);
    while (true){
    double heading = imu.get_heading() - init_heading;
        if (theta > 0){ 
    if (heading > 300 ){
        heading = heading - 360;
    }
    } else {
        if (heading > 30 ){
        heading = heading - 360;
    }
    }
    int encoderAvgL = LB.get_position(); // + RB.get_position()) / 2;
    int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, false);
        if (voltageL > 127){ //if the PID becomes innacurate get rid of the - fix values and make voltageL > 100
            voltageL = 127;
        } else if (voltageL < -127){
            voltageL = -127;
        }
    int encoderAvgR = RB.get_position(); // + RB.get_position()) / 2;
    int voltageR = calcPID(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, false);
        if (voltageR > 127){
            voltageR = 127;
        } else if (voltageR < -127){
            voltageR = -127;
        }

    int rightcorrect = (encoderAvgR * 360) / (2 * pi * radius);
    int fix = int(heading - rightcorrect);
    fix = fix * 10; //reaction sensitivity
    con.print(0, 0, "ERROR: %f       ", float(ltarget));
        if ((abs(ltarget - encoderAvgL) <= 0.9) && (abs(rtarget - encoderAvgL) <= 0.9)) count++; //0.35
        if (count >= 40 || time > timeout) {
           //imu.tare_heading();
            break;
        }
    

    chasMove(voltageL - fix, voltageL - fix, voltageR + fix, voltageR + fix, voltageR + fix, voltageL - fix);
    time += 10;
    delay(10);

    }

    }



