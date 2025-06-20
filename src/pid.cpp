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

int intergral;
int deritvative;
int time2;
double power;
double prevError;

double vKp2;
double vKi2;
double vKd2;
float error2;

int intergral2;
int deritvative2;
int time22;
double power2;
double prevError2;

double vKp3;
double vKi3;
double vKd3;
float error3;

int intergral3;
int deritvative3;
int time33;
double power3;
double prevError3;

double vKp4;
double vKi4;
double vKd4;
float error4;

int intergral4;
int deritvative4;
int time4;
double power4;
double prevError4;

void setConstants(double kp, double ki, double kd) {
    vKp = kp;
    vKi = ki;
    vKd = kd;
}
int LBMacro = 0;
double LBPos = 0;

double calcPIDlift(double target, double input, int integralKi, int maxIntegral) {

    int integral4;

    prevError4 = error4;
    error4 = target - input;

    if(abs(error2) < integralKi) {
        integral4 += error4;
    } else {
        integral4 = 0;
    }

    if(integral4 >= 0) {
        integral4 = min(integral4, maxIntegral);
    
    } else {
        integral4 = max(integral4, -maxIntegral);
    }

    deritvative4 = error4 - prevError4;

    power4 = (vKp * error4) + (vKi * integral4) + (vKd * deritvative4);

    return power4;

}

void LadyBrownMacro(){
    LBPos = roto.get_angle();
    if(LBPos > 300000){
        LBPos -= 36000;
    }
    if(LBMacro == 1){
        setConstants(1,0, 0);
        LDB.move(calcPIDlift(1500, LBPos, 0, 0));
    } else if(LBMacro == 2){
        setConstants(1,0, 0);
        LDB.move(calcPIDlift(3000, LBPos, 0, 0));
    } else if(LBMacro == 3){
        setConstants(1,0, 0);
        LDB.move(calcPIDlift(15000, LBPos, 0, 0));
    }
}


void resetEncoders() {
    LF.tare_position();
    LM.tare_position();
    LB.tare_position();
    RF.tare_position();
    RM.tare_position();
    RB.tare_position();
}

void chasMove(int voltageLF, int voltageLM, int voltageLB, int voltageRF, int voltageRM, int voltageRB) {
    LF.move(voltageLF);
    LM.move(voltageLM);
    LB.move(voltageLB);
    RF.move(voltageRF);
    RM.move(voltageRM);
    RB.move(voltageRB);
}

double calcPID(double target, double input, int integralKi, int maxIntegral) {
    LadyBrownMacro();
    int integral;
    prevError = error;
    error = target - input;

    if(abs(error) < integralKi) {
        integral += error;
    } else {
        integral = 0;
    }

    if(integral >= 0) {
        integral = min(integral, maxIntegral);
    
    } else {
        integral = max(integral, -maxIntegral);
    }

    deritvative = error - prevError;

    power = (vKp * error) + (vKi * integral) + (vKd * deritvative);

    return power;

}

double calcPID2(double target, double input, int integralKi, int maxIntegral) {

    int integral2;

    prevError2 = error2;
    error2 = target - input;

    if(abs(error2) < integralKi) {
        integral2 += error2;
    } else {
        integral2 = 0;
    }

    if(integral2 >= 0) {
        integral2 = min(integral2, maxIntegral);
    
    } else {
        integral2 = max(integral2, -maxIntegral);
    }

    deritvative2 = error2 - prevError2;

    power2 = (vKp * error2) + (vKi * integral2) + (vKd * deritvative2);

    return power2;

}

double calcPID3(double target, double input, int integralKi, int maxIntegral) {
    int integral3;

    prevError3 = error3;
    error2 = target - input;

    if(abs(error3) < integralKi) {
        integral3 += error3;
    } else {
        integral3 = 0;
    }

    if(integral3 >= 0) {
        integral3 = min(integral3, maxIntegral);
    
    } else {
        integral3 = max(integral3, -maxIntegral);
    }

    deritvative3 = error3 - prevError3;

    power3 = (vKp * error3) + (vKi * integral3) + (vKd * deritvative3);

    return power3;

}
double calcPID4(double target, double input, int integralKi, int maxIntegral) {
    int integral4;

    prevError4 = error4;
    error4 = target - input;

    if(abs(error2) < integralKi) {
        integral4 += error4;
    } else {
        integral4 = 0;
    }

    if(integral4 >= 0) {
        integral4 = min(integral4, maxIntegral);
    
    } else {
        integral4 = max(integral4, -maxIntegral);
    }

    deritvative4 = error4 - prevError4;

    power4 = (vKp * error4) + (vKi * integral4) + (vKd * deritvative4);

    return power4;

}


void driveStraight(int target) {
    double voltage;
    double encoderAvg;
    int count = 0;
    time2 = 0;

    int timeout = 30000;

    resetEncoders();
    while(true) {
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        encoderAvg = (LF.get_position() + RF.get_position()) / 2;
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        chasMove(voltage, voltage, voltage, voltage, voltage, voltage);
        if (abs(target - encoderAvg) <= 3) count++;
        if (count >= 20 || time2 > timeout){
            break;
        }

    delay(10);
    time2 += 10;
    }
}

void driveTurn2(int target) {
    double voltage;
    double position;
    int count = 0;
    time2 = 0;
    int turnv = 0;

    setConstants(TURN_KP, TURN_KI, TURN_KD);

    int timeout = 30000;

    position = imu.get_heading();

    if (position > 180){
        position = position - 360;
    }

    if((target < 0) && (position > 0)){
        if((position - target) >= 180){
            target = target + 360;
            position = imu.get_heading();
            turnv = (target - position); 
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

    while(true) {
 position = imu.get_heading();

    if (position > 180){
        position = position - 360;
    }

    if((target < 0) && (position > 0)){
        if((position - target) >= 180){
            target = target + 360;
            position = imu.get_heading();
            turnv = (target - position); 
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

        voltage = calcPID(target, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL);

        chasMove(voltage, voltage, voltage, -voltage, -voltage, -voltage);

        if (abs(target - position) <= 1.5) count++;
        if (count >= 20 || time2 > timeout) {
        break;
        }

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0) {
            con.print(0, 0, "ERROR: %f      ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "IMU:               ", float(imu.get_heading()));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f           ", float(time2));
        }

        time2 += 10;
        delay(10);
    
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}

void driveTurn(int target) {
    double voltage;
    double position;
    int count = 0;
    time2 = 0;

    setConstants(TURN_KP, TURN_KI, TURN_KD);

    int timeout = 30000;

    imu.tare_heading();

    while(true) {
        if (position > 180) {
            position = position - 360;
        }

        voltage = calcPID(target, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL);

        chasMove(voltage, voltage, voltage, -voltage, -voltage, -voltage);

        if (abs(target - position) <= 1.5) count++;
        if (count >= 20 || time2 > timeout) {
        break;
        }

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0) {
            con.print(0, 0, "ERROR: %f      ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "IMU:               ", float(imu.get_heading()));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f           ", float(time2));
        }

        time2 += 10;
        delay(10);
    
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}

void driveStraight2(int target) {
    double voltage;
    double encoderAvg;
    int count = 0;
    time2 = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    double position = 0;

    if(init_heading > 180){
        init_heading = init_heading - 360;
    }

    int timeout = 30000;

    resetEncoders();
    while(true) {
        position = imu.get_heading();

    if (position > 180){
        position = position - 360;
    }

    if((target < 0) && (position > 0)){
        if((position - target) >= 180){
            target = target + 360;
            position = imu.get_heading();
        }
    } else if ((target > 0) && (position < 0)){
        if((target - position) >= 180){
            position = imu.get_heading();
        } 
    } 
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        encoderAvg = (LF.get_position() + RF.get_position()) / 2;
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
        heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        chasMove(voltage, voltage, voltage, voltage, voltage, voltage);
        if (abs(target - encoderAvg) <= 3) count++;
        if (count >= 20 || time2 > timeout){
            break;
        }
                if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0) {
            con.print(0, 0, "ERROR: %f      ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "IMU:               ", float(imu.get_heading()));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f           ", float(time2));
        }
        delay(10);
        time2 += 10;

    }
}


void driveStraightC(int target) {
    bool over = false;
    double voltage;
    double encoderAvg;
    int count = 0;
    time2 = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    double position = 0;

    if(init_heading > 180){
        init_heading = init_heading - 360;
    }

    int timeout = 30000;

    if(target > 0){
        target = target + 500;
    } else {
        target = target - 500;
    }

    resetEncoders();
    while(true) {
        position = imu.get_heading();

    if (position > 180){
        position = position - 360;
    }

    if((target < 0) && (position > 0)){
        if((position - target) >= 180){
            target = target + 360;
            position = imu.get_heading();
        }
    } else if ((target > 0) && (position < 0)){
        if((target - position) >= 180){
            position = imu.get_heading();
        } 
    } 
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        encoderAvg = (LF.get_position() + RF.get_position()) / 2;
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
        heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        chasMove(voltage, voltage, voltage, voltage, voltage, voltage);
        if (target > 0){
            if ((encoderAvg - (target - 500)) > 0){
                over = true;
            }
        } else {
            if (((target+500) - encoderAvg) > 0){
                over = true;
            }
        }

        if (over || time2 > timeout){
            break;
        }

                if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0) {
            con.print(0, 0, "ERROR: %f      ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "IMU:               ", float(imu.get_heading()));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f           ", float(time2));
        }
        delay(10);
        time2 += 10;

    }
}

void driveArcL(double theta, double radius, int timeout){

    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);

    double ltarget = 0;
    double rtarget = 0;
    double pi = 3.14159265359;
    double init_heading = imu.get_heading();
    int count = 0;

    time2 = 0;
    resetEncoders();

    ltarget = double((theta / 360) * 2 * pi * radius);
    rtarget = double((theta / 360) * 2 * pi * (radius + 30));

    while (true){
        double encoderAvgL = (LF.get_position() + LB.get_position()) / 2;
        double encoderAvgR = (RF.get_position() + RB.get_position()) / 2;
        double leftcorrect = -(encoderAvgL * 360) / (2 * pi * radius);

        if(init_heading > 180){
            init_heading = init_heading - 360;
        }

        double position = imu.get_heading() - 360;

        if(position > 180){
            position = position - 360;
        }

        if(((init_heading + leftcorrect)< 0) && (position > 0)){
            if((position - (init_heading + leftcorrect)) >= 180){
                leftcorrect = leftcorrect + 360;
                position = imu.get_heading();
            }
        }   else if (((init_heading + leftcorrect) > 0) &&(position < 0)){
            if(((init_heading + leftcorrect) - position) >= 180){
            position = imu.get_heading();
            }
        }

        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageL > 127){
            voltageL = 127;
        } else if (voltageL < -127){
            voltageL = -127;
        }

        int voltageR = calcPID2(ltarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageR > 127){
            voltageR = 127;
        } else if (voltageR < - 127){
            voltageR = -127;
        }

        setConstants(ARC_KP, ARC_KI, ARC_KD);
        int fix = calcPID3((init_heading + leftcorrect), position, ARC_KI, ARC_MAX_INTEGRAL);

        chasMove((voltageL + fix), (voltageL + fix), (voltageL + fix), (voltageR - fix), (voltageR - fix), (voltageR - fix));
        if ((abs(ltarget - encoderAvgL) <= 4) && (abs(rtarget - encoderAvgR) <= 4)) count++;
        if (count >= 20 || time2 > timeout){
            // break;
        }

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0) {
            con.print(0, 0, "ERROR: %f      ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "IMU:               ", float(imu.get_heading()));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f           ", float(time2));
        }
        delay(10);
        time2 += 10;

    }

}

void driveArcR(double theta, double radius, int timeout){

    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);

    double ltarget = 0;
    double rtarget = 0;
    double pi = 3.14159265359;
    double init_heading = imu.get_heading();
    int count = 0;

    time2 = 0;
    resetEncoders();

    ltarget = double((theta / 360) * 2 * pi * (radius + 30));
    rtarget = double((theta / 360) * 2 * pi * (radius));

    while (true){
        double encoderAvgL = (LF.get_position() + LB.get_position()) / 2;
        double encoderAvgR = (RF.get_position() + RB.get_position()) / 2;
        double rightcorrect = (encoderAvgR * 360) / (2 * pi * radius);

        if(init_heading > 180){
            init_heading = init_heading - 360;
        }

        double position = imu.get_heading() - 360;

        if(position > 180){
            position = position - 360;
        }

        if(((init_heading + rightcorrect)< 0) && (position > 0)){
            if((position - (init_heading + rightcorrect)) >= 180){
                rightcorrect = rightcorrect + 360;
                position = imu.get_heading();
            }
        }   else if (((init_heading + rightcorrect) > 0) &&(position < 0)){
            if(((init_heading + rightcorrect) - position) >= 180){
            position = imu.get_heading();
            }
        }

        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageL > 127){
            voltageL = 127;
        } else if (voltageL < -127){
            voltageL = -127;
        }

        int voltageR = calcPID2(ltarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageR > 127){
            voltageR = 127;
        } else if (voltageR < - 127){
            voltageR = -127;
        }

        setConstants(ARC_KP, ARC_KI, ARC_KD);
        int fix = calcPID3((init_heading + rightcorrect), position, ARC_KI, ARC_MAX_INTEGRAL);

        chasMove((voltageL + fix), (voltageL + fix), (voltageL + fix), (voltageR - fix), (voltageR - fix), (voltageR - fix));
        if ((abs(ltarget - encoderAvgL) <= 4) && (abs(rtarget - encoderAvgR) <= 4)) count++;
        if (count >= 20 || time2 > timeout){
            // break;
        }

        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0) {
            con.print(0, 0, "ERROR: %f      ", float(error));
        } else if (time2 % 100 == 0 && time2 % 150 != 0){
            con.print(1, 0, "IMU:               ", float(imu.get_heading()));
        } else if (time2 % 150 == 0){
            con.print(2, 0, "Time: %f           ", float(time2));
        }
        delay(10);
        time2 += 10;

    }

}


