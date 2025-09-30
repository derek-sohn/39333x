#include "api.h"
#include "main.h"

#define LF_PORT 10
#define LM_PORT 13
#define LB_PORT 17
#define RF_PORT 19
#define RM_PORT 14
#define RB_PORT 18
#define INTAKE_PORT 20
// #define ROTO_PORT NA
// #define OPTICAL_PORT NA
#define IMU_PORT 12
#define INTAKE1_PORT 15
// #define DISTANCE_PORT_R NA
// #define DISTANCE_PORT_L NA
#define INTAKE2_PORT 16


pros::ADIDigitalOut scrapper ('A', false);
pros::ADIDigitalOut descore ('H', false);


pros::Motor LF (LF_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LM (LM_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LB (LB_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RF (RF_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RM (RM_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RB (RB_PORT, pros::E_MOTOR_GEARSET_06, false);

// pros::Rotation roto(ROTO_PORT);

pros::Motor INTAKE (INTAKE_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor INTAKE1 (INTAKE1_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor INTAKE2 (INTAKE2_PORT, pros::E_MOTOR_GEARSET_06, true);

pros::MotorGroup intakes({
    pros::Motor(INTAKE_PORT, pros::E_MOTOR_GEARSET_06, true),
    pros::Motor(INTAKE1_PORT, pros::E_MOTOR_GEARSET_06, false),
    pros::Motor(INTAKE2_PORT, pros::E_MOTOR_GEARSET_06, true)
});


pros::Controller con (pros::E_CONTROLLER_MASTER);

// pros::Optical OpticalC (OPTICAL_PORT);
// pros::Distance DistanceRight(DISTANCE_PORT_R);
// pros::Distance DistanceLeft(DISTANCE_PORT_L);

pros::Imu imu (IMU_PORT);

pros::ADIDigitalIn Autonselect ('F');
//letter is for port
