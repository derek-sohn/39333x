#include "api.h"
#include "main.h"

#define LF_PORT 1
#define LM_PORT 2 
#define LB_PORT 3
#define RF_PORT 4
#define RM_PORT 5
#define RB_PORT 6
#define INTAKE_PORT 7
#define ROTO_PORT 1
#define LDB_PORT 9

#define IMU_PORT 8

pros::Motor LF (LF_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LM (LM_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LB (LB_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RF (RF_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RM (RM_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RB (RB_PORT, pros::E_MOTOR_GEARSET_06, false);

pros::Rotation roto(ROTO_PORT);
pros::Motor LBD(LDB_PORT, pros::E_MOTOR_GEARSET_06, false);

pros::Motor INTAKE (INTAKE_PORT, pros::E_MOTOR_GEARSET_06, false);

pros::Controller con (pros::E_CONTROLLER_MASTER);

pros::Imu imu (IMU_PORT);